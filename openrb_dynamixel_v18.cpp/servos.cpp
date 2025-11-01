#include "servos.h"
#include "vertical_kinematics.h"

// -------------------------------------------------------------------
//                        GLOBAL CONSTANTS
// -------------------------------------------------------------------

const int STALL_CURRENT_mA = 1000;
const int TEMP_LIMIT_C = 70;

const float TICKS_PER_REV = 4096.0f;
const float PV_UNIT_RPM = 0.229f;
const float TIME_SAFETY_FACTOR = 1.20f;
const uint32_t MIN_WAIT_MS = 120;
const uint32_t EXTRA_SETTLE_MS = 50;

const double TICKS_PER_DEG = 4096.0 / 360.0;
const double DEG_PER_TICK = 0.087890625;
const double MM_PER_TICK = 0.0767;

// -------------------------------------------------------------------
//                         DYNAMIXEL SETUP
// -------------------------------------------------------------------

#define DXL_SERIAL Serial1
#define DXL_DIR_PIN -1
#define PROTOCOL 2.0
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
extern VerticalKinematics kin;

// -------------------------------------------------------------------
//                     ServoConfig CLASS IMPLEMENTATION
// -------------------------------------------------------------------

ServoConfig::ServoConfig(const char *key,
                         uint8_t id,
                         uint16_t zero_ticks,
                         double dir,
                         uint16_t limit_min,
                         uint16_t limit_max)
  : key_(key),
    id_(id),
    zero_ticks_(zero_ticks),
    dir_(dir),
    limit_min_(limit_min),
    limit_max_(limit_max) {}

uint8_t ServoConfig::get_id() const {
  return id_;
}
const char *ServoConfig::get_key() const {
  return key_;
}
uint16_t ServoConfig::zero_ticks() const {
  return zero_ticks_;
}
uint16_t ServoConfig::min_ticks() const {
  return limit_min_;
}
uint16_t ServoConfig::max_ticks() const {
  return limit_max_;
}
double ServoConfig::dir() const {
  return dir_;
}

// -------------------------------------------------------------------
//                     SERVO CONFIG INSTANCES
// -------------------------------------------------------------------

#define TICK_ZERO 2048
#define TICK_90 3072
#define TICK_MINUS90 1024

ServoConfig arm1("arm1", ID_ARM1, TICK_ZERO, -1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig arm2("arm2", ID_ARM2, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig grip("grip", ID_GRIP, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig grip1("grip1", ID_GRIP1, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig grip2("grip2", ID_GRIP2, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig base("base", ID_BASE, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);

ServoConfig *all_servos[] = { &arm1, &arm2, &grip, &grip1, &grip2, &base };
constexpr uint8_t SERVO_COUNT = sizeof(all_servos) / sizeof(all_servos[0]);

// -------------------------------------------------------------------
//                     ENFORCE SERVO LIMITS
// -------------------------------------------------------------------

void enforce_servo_limits() {

  for (int i = 0; i < SERVO_COUNT; i++) {
    ServoConfig *cfg = all_servos[i];
    uint8_t id = cfg->get_id();

    // ------------------------------------------------
    // 1. Read current limits from the servo
    // ------------------------------------------------
    uint16_t hw_min = dxl.readControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id);
    uint16_t hw_max = dxl.readControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id);

    uint16_t want_min = cfg->min_ticks();
    uint16_t want_max = cfg->max_ticks();
    bool changed = false;

    // ------------------------------------------------
    // 1.1 Fix corrupted limits (hw_min > hw_max)
    // ------------------------------------------------
    if (hw_min > hw_max) {
      serial_printf("[%s] ID %u: ⚠ invalid limits (%u > %u), resetting to [%u - %u]\n",
                    cfg->get_key(), id, hw_min, hw_max, want_min, want_max);

      dxl.torqueOff(id);
      dxl.writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id, want_min);
      dxl.writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id, want_max);
      dxl.torqueOn(id);

      hw_min = want_min;
      hw_max = want_max;
      changed = true;
    }

    // ------------------------------------------------
    // 2. Tighten range if wider than desired
    // ------------------------------------------------
    if (hw_min < want_min) {
      dxl.torqueOff(id);
      dxl.writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id, want_min);
      dxl.torqueOn(id);
      hw_min = want_min;
      changed = true;
    }

    if (hw_max > want_max) {
      dxl.torqueOff(id);
      dxl.writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id, want_max);
      dxl.torqueOn(id);
      hw_max = want_max;
      changed = true;
    }

    if (changed) {
      serial_printf("[%s] ID %u: limits updated to [%u - %u]\n",
                    cfg->get_key(), id, hw_min, hw_max);
    } else {
      serial_printf("[%s] ID %u: limits OK [%u - %u]\n",
                    cfg->get_key(), id, hw_min, hw_max);
    }

    // ------------------------------------------------
    // 3. Correct out-of-range positions
    // ------------------------------------------------
    uint16_t pos = dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, id);

    if (pos < hw_min) {
      serial_printf("[%s] pos=%u < min=%u → moving to min\n", cfg->get_key(), pos, hw_min);
      dxl.setGoalPosition(id, hw_min);
    } else if (pos > hw_max) {
      serial_printf("[%s] pos=%u > max=%u → moving to max\n", cfg->get_key(), pos, hw_max);
      dxl.setGoalPosition(id, hw_max);
    } else {
      serial_printf("[%s] pos=%u within limits\n", cfg->get_key(), pos);
    }
  }
}

// -------------------------------------------------------------------
//                     HELPER FUNCTIONS
// -------------------------------------------------------------------

ServoConfig *find_servo(uint8_t id) {
  for (uint8_t i = 0; i < SERVO_COUNT; i++)
    if (all_servos[i]->get_id() == id) return all_servos[i];
  return nullptr;
}

ServoConfig *find_servo(const char *name) {
  for (uint8_t i = 0; i < SERVO_COUNT; i++)
    if (strcmp(all_servos[i]->get_key(), name) == 0) return all_servos[i];
  return nullptr;
}

double ticks2deg(uint8_t id, int ticks) {
  if (auto *s = find_servo(id))
    return (ticks - s->zero_ticks()) * (360.0 / 4096.0) * s->dir();
  return 0.0;
}

int deg2ticks(uint8_t id, double deg) {
  ServoConfig *s = find_servo(id);
  if (!s) return 0;
  return s->zero_ticks() + (int)(deg / (360.0 / 4096.0) * s->dir());
}

// -------------------------------------------------------------------
//                     HELPER FUNCTIONS (extended)
// -------------------------------------------------------------------

int per2ticks(uint8_t id, double per) {
  ServoConfig *s = find_servo(id);
  if (!s) return 0;

  double min_t = s->min_ticks();
  double max_t = s->max_ticks();

  // Clamp 0–100%
  if (per < 0) per = 0;
  if (per > 100) per = 100;

  // Map percent to ticks considering direction
  double range = max_t - min_t;
  double ticks = (s->dir() > 0)
                   ? min_t + (per / 100.0) * range
                   : max_t - (per / 100.0) * range;

  return (int)round(ticks);
}

double per2deg(uint8_t id, double per) {
  ServoConfig *s = find_servo(id);
  if (!s) return 0;

  int ticks = per2ticks(id, per);
  return ticks2deg(id, ticks);
}

double ticks2per(uint8_t id, double ticks) {
  ServoConfig *s = find_servo(id);
  if (!s) return 0;

  double min_t = s->min_ticks();
  double max_t = s->max_ticks();
  double range = max_t - min_t;
  if (range == 0) return 0;

  double per;
  if (s->dir() > 0)
    per = ((ticks - min_t) / range) * 100.0;
  else
    per = ((max_t - ticks) / range) * 100.0;

  // Clamp for safety
  if (per < 0) per = 0;
  if (per > 100) per = 100;
  return per;
}

uint8_t name2id(const char *name) {
  ServoConfig *s = find_servo(name);
  return s ? s->get_id() : 0;
}

const char *id2name(uint8_t id) {
  ServoConfig *s = find_servo(id);
  return s ? s->get_key() : "";
}

// -------------------------------------------------------------------
//                           LED HELPERS
// -------------------------------------------------------------------

void lOn(uint8_t id) {
  if (dxl.ping(id)) dxl.ledOn(id);
}
void lOff(uint8_t id) {
  if (dxl.ping(id)) dxl.ledOff(id);
}

// -------------------------------------------------------------------
//                GROUP OPERATIONS (MULTIPLE SERVOS)
// -------------------------------------------------------------------
void torqueOnGroup(const std::vector<uint8_t> &ids) {
  for (auto id : ids)
    if (dxl.ping(id)) dxl.torqueOn(id);
}

void torqueOffGroup(const std::vector<uint8_t> &ids) {
  for (auto id : ids)
    if (dxl.ping(id)) {
      dxl.torqueOff(id);
      delay(30);
    }
}

void adjustPwmGroup(const std::vector<uint8_t> &ids, int pwmLimit) {
  for (auto id : ids)
    if (dxl.ping(id)) dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id, pwmLimit);
}

void ledOnGroup(const std::vector<uint8_t> &ids) {
  for (auto id : ids)
    if (dxl.ping(id)) dxl.ledOn(id);
}

void ledOffGroup(const std::vector<uint8_t> &ids) {
  for (auto id : ids)
    if (dxl.ping(id)) dxl.ledOff(id);
}

bool checkStallGroup(const std::vector<uint8_t> &ids) {
  for (auto id : ids)
    if (checkStall(id)) return true;
  return false;
}

// -------------------------------------------------------------------
//                   POSITION + STATUS HELPERS
// -------------------------------------------------------------------

double getPos_deg(int id) {
  return ticks2deg(id, dxl.getPresentPosition(id));
}

void setGoal_deg(int id, double goal_deg) {
  int goal_ticks = deg2ticks(id, goal_deg);
  dxl.setGoalPosition(id, goal_ticks);
}

bool isInPosition(uint8_t id) {
  int s = dxl.readControlTableItem(ControlTableItem::MOVING_STATUS, id);
  return (s >= 0) && (s & 0x01);
}

bool isMoving(uint8_t id) {
  int s = dxl.readControlTableItem(ControlTableItem::MOVING_STATUS, id);
  return (s >= 0) && (s & 0x02);
}

// -------------------------------------------------------------------
//                        STALL DETECTION
// -------------------------------------------------------------------

bool checkStall(uint8_t id) {
  int curr = dxl.getPresentCurrent(id);
  int temp = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, id);
  if (temp >= TEMP_LIMIT_C || curr > STALL_CURRENT_mA) {
    dxl.torqueOff(id);
    lOn(id);
    serial_printf("ERR: STALL ID %d curr=%d temp=%d\n", id, curr, temp);
    return true;
  }
  return false;
}

// -------------------------------------------------------------------
//                        READ STATUS (ALL OR SINGLE SERVO)
// -------------------------------------------------------------------
// -------------------------------------------------------------------
//                        READ STATUS (ALL OR SINGLE SERVO)
// -------------------------------------------------------------------
void print_servo_status(uint8_t id) {
  //---- basic status for each servo ----
  uint8_t startIndex = 0;
  uint8_t endIndex = SERVO_COUNT;

  if (id > 0) {
    // find servo object by ID
    ServoConfig *s = find_servo(id);

    if (s) {
      // find its index in the all_servos[] array
      for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        if (all_servos[i] == s) {
          startIndex = i;
          endIndex = i + 1;
          break;
        }
      }
    } else {
      serial_printf("STATUS id=%d not found\n", id);
      return;
    }
  }

  for (uint8_t i = startIndex; i < endIndex && i < SERVO_COUNT; i++) {
    ServoConfig *s = all_servos[i];
    uint8_t sid = s->get_id();

    if (!dxl.ping(sid)) {
      serial_printf("STATUS %s (id=%u): pos=na current=na temp=na\n", s->get_key(), sid);
    } else {
      int pos_ticks = dxl.getPresentPosition(sid);
      int curr_mA   = dxl.getPresentCurrent(sid);
      int temp_C    = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, sid);

      double pos_deg = ticks2deg(sid, pos_ticks);
      double pos_per = ticks2per(sid, pos_ticks);   // ← percentage of configured range

      serial_printf("STATUS %s (id=%2u): pos=%4d  deg=%7.2f  per=%6.2f%%  current=%4dmA  temp=%2d°C\n",
                    s->get_key(), sid, pos_ticks, pos_deg, pos_per, curr_mA, temp_C);
    }
  }

  // ---- XY metrics (for 2-arm systems) ----
  if (id == 0 && dxl.ping(ID_ARM1) && dxl.ping(ID_ARM2)) {
    kin.readPresentPositions();
    serial_printf("STATUS XY X=%.2fmm Y=%.2fmm A1=%.2fdeg A2=%.2fdeg G=%.2fdeg  G align=%.2fdeg\n",
                  kin.getXmm(), kin.getYmm(), kin.getA1deg(), kin.getA2deg(),
                  kin.getGdeg(), kin.getGdeg_aligned());
  } else if (id == 0) {
    serial_printf("STATUS XY X=na Y=na A1=na A2=na G=na\n");
  }
}
