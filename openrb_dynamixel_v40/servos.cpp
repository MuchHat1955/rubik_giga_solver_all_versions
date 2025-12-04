#include "servos.h"
#include "vertical_kinematics.h"
#include <Arduino.h>
#include <vector>
#include <math.h>
#include <string.h>

// -------------------------------------------------------------------
//                        GLOBAL CONSTANTS
// -------------------------------------------------------------------

const int STALL_CURRENT_mA = 500;
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
const float PROTOCOL = 2.0f;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

extern VerticalKinematics kin;

// -------------------------------------------------------------------
//   SAFE GOAL POSITION WRAPPER (with LED flash on fault)
// -------------------------------------------------------------------
//  Checks servo current and temperature before commanding a move.
//  If either exceeds safety limits, torque is turned off and
//  the LED flashes several times to signal a fault.
// -------------------------------------------------------------------

void reset_servo(uint8_t id) {
  Serial.print("Rebooting servo ");
  Serial.println(id);

  dxl.reboot(id);
  delay(50);

  // Optional: re-enable torque
  dxl.torqueOn(id);

  Serial.println("Servo rebooted and torque re-enabled");
}

bool servo_ok(uint8_t id) {

  // 1) Read hardware error flags
  int hw_err = dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id);

  // -1 means read failed
  if (hw_err < 0) {
    serial_printf("DXL %d: failed to read HW_ERROR_STATUS\n", id);
    return false;
  }

  // 2) Read SHUTDOWN register (which bits are configured to trigger shutdown)
  int shutdown = dxl.readControlTableItem(ControlTableItem::SHUTDOWN, id);

  if (shutdown < 0) {
    serial_printf("DXL %d: failed to read SHUTDOWN\n", id);
    return false;
  }

  // Log what we detected
  serial_printf("DXL %d: hw_err=0x%02X  shutdown=0x%02X\n", id, hw_err, shutdown);

  // If no hardware errors → OK
  if (hw_err == 0) {
    return true;
  }

  // 3) We have errors → try recovery
  serial_printf("DXL %d ERROR detected, attempting recovery...\n", id);

  // Disable torque before reboot
  dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, id, 0);
  delay(20);

  // 4) Reboot the servo to clear hardware errors
  bool reboot_ok = dxl.reboot(id);

  if (!reboot_ok) {
    serial_printf("DXL %d: reboot FAILED\n", id);
    return false;
  }

  delay(200);

  // 5) Turn torque back on
  dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, id, 1);
  delay(20);

  // 6) Re-read error register
  hw_err = dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id);

  serial_printf("DXL %d: post-reboot hw_err=0x%02X\n", id, hw_err);

  return hw_err == 0;
}

bool servoError = false;
constexpr int LED_FLASH_COUNT = 3;
constexpr int LED_FLASH_DELAY_MS = 120;

bool safeSetGoalPosition(uint8_t id, int goal_ticks) {

  if (servoError) {
    serial_printf("ERR servo error, skip everything\n");
    return false;
  }

  uint8_t hw_err = dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id);

  if (!servo_ok(id)) {
    serial_printf("ERR Servo %d error: 0x%02X\n", id, hw_err);
    for (int i = 0; i < LED_FLASH_COUNT; i++) {
      lOn(id);
      delay(LED_FLASH_DELAY_MS);
      lOff(id);
      delay(LED_FLASH_DELAY_MS);
    }
    reset_servo(id);
    for (int i = 0; i < LED_FLASH_COUNT; i++) {
      lOn(id);
      delay(LED_FLASH_DELAY_MS);
      lOff(id);
      delay(LED_FLASH_DELAY_MS);
    }
    if (!servo_ok(id)) {
      serial_printf("ERR setting the error flag for all because of servo=%d\n", id);
      servoError = true;
      return false;
    }
  }

  // Check for min and maxt
  if (goal_ticks < getMin_ticks(id) || goal_ticks > getMax_ticks(id)) {
    // Flash LED a few times as a warning
    for (int i = 0; i < LED_FLASH_COUNT; i++) {
      lOn(id);
      delay(LED_FLASH_DELAY_MS);
      lOff(id);
      delay(LED_FLASH_DELAY_MS);
    }
    if (goal_ticks < getMin_ticks(id)) {
      serial_printf_verbose(
        "[safe move do nothing, under min] id=%u goal=%d min=%d\n",
        id, goal_ticks, getMin_ticks(id));
    }
    if (goal_ticks > getMax_ticks(id)) {
      serial_printf_verbose(
        "[safe move do nothing, over max] id=%u goal=%d max=%d\n",
        id, goal_ticks, getMax_ticks(id));
    }

    for (int i = 0; i < LED_FLASH_COUNT; i++) {
      lOn(id);
      delay(LED_FLASH_DELAY_MS);
      lOff(id);
      delay(LED_FLASH_DELAY_MS);
    }
    return false;  // abort move
  }

  // Otherwise safe to move
  dxl.setGoalPosition(id, goal_ticks);
  return true;
}

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

void ServoConfig::init() {

  if (limit_max_ > 4095) limit_max_ = 4095;

  if (!dxl.ping(id_)) {
    serial_printf_verbose("ERR [servo init ping failed] | %s id=%u zero=%u min=%u max=%u dir=%d\n",
                          key_, id_, zero_ticks_, limit_min_, limit_max_, dir_);
    return;
  }

  // read min and max from the servo
  limit_min_ = dxl.readControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id_);
  limit_max_ = dxl.readControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id_);

  serial_printf_verbose("[servo init] | %s id=%u zero=%u min=%u max=%u dir=%d\n",
                        key_, id_, zero_ticks_, limit_min_, limit_max_, dir_);
}

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

void ServoConfig::set_zero_ticks(uint16_t t) {
  return;

  zero_ticks_ = t;
  // keep limits sane around update
  if (limit_min_ > limit_max_) {
    uint16_t a = limit_min_;
    limit_min_ = limit_max_;
    limit_max_ = a;
  }
  serial_printf_verbose("[set zero] %s id=%u set zero=%u\n", key_, id_, zero_ticks_);
}
void ServoConfig::set_min_ticks(uint16_t t) {

  limit_min_ = t;
  if (limit_min_ > limit_max_) {
    uint16_t a = limit_min_;
    limit_min_ = limit_max_;
    limit_max_ = a;
  }

  serial_printf_verbose("[set min] | %s id=%u set min=%u (max=%u)\n", key_, id_, limit_min_, limit_max_);
  dxl.torqueOff(id_);
  dxl.writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id_, limit_min_);
  dxl.writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id_, limit_max_);
  dxl.torqueOn(id_);

  serial_printf_verbose("[set min] servo control table min set | %s id=%u zero=%u min=%u max=%u dir=%d\n",
                        key_, id_, zero_ticks_, limit_min_, limit_max_, dir_);
}
void ServoConfig::set_max_ticks(uint16_t t) {

  limit_max_ = t;
  if (limit_min_ > limit_max_) {
    uint16_t a = limit_min_;
    limit_min_ = limit_max_;
    limit_max_ = a;
  }

  serial_printf_verbose("[set max] | %s id=%u set max=%u (min=%u)\n", key_, id_, limit_max_, limit_min_);
  dxl.torqueOff(id_);
  dxl.writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id_, limit_min_);
  dxl.writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id_, limit_max_);
  dxl.torqueOn(id_);

  serial_printf_verbose("[set max] servo control table max set | %s id=%u zero=%u min=%u max=%u dir=%d\n",
                        key_, id_, zero_ticks_, limit_min_, limit_max_, dir_);
}

// -------------------------------------------------------------------
//                     SERVO CONFIG INSTANCES
// -------------------------------------------------------------------

#define MID_TICK 2048
#define TICK_100DEG 1138
#define TICK_10DEG 113

#define DEFAULT_ZERO MID_TICK
#define DEFAULT_MIN MID_TICK - TICK_100DEG
#define DEFAULT_MAX MID_TICK + TICK_100DEG

#define WRIST_ZERO 475
#define BASE_ZERO 2735
#define WRIST_MIN WRIST_ZERO - TICK_10DEG
#define WRIST_MAX WRIST_ZERO + TICK_100DEG + TICK_100DEG

// TODO fix those based on the HW
ServoConfig arm1("arm1", ID_ARM1, DEFAULT_ZERO, -1.0, DEFAULT_MIN, DEFAULT_MAX);
ServoConfig arm2("arm2", ID_ARM2, DEFAULT_ZERO, 1.0, DEFAULT_MIN, DEFAULT_MAX);
ServoConfig wrist("wrist", ID_WRIST, WRIST_ZERO, 1.0, WRIST_MIN, WRIST_MAX);
ServoConfig grip1("grip1", ID_GRIP1, DEFAULT_ZERO, 1.0, DEFAULT_MIN, DEFAULT_MAX);
ServoConfig grip2("grip2", ID_GRIP2, DEFAULT_ZERO, -1.0, DEFAULT_MIN, DEFAULT_MAX);
ServoConfig base("base", ID_BASE, BASE_ZERO, 1.0, DEFAULT_MIN, DEFAULT_MAX);

ServoConfig *all_servos[] = { &arm1, &arm2, &wrist, &grip1, &grip2, &base };
constexpr uint8_t SERVO_COUNT = sizeof(all_servos) / sizeof(all_servos[0]);

// -------------------------------------------------------------------
//                     ENFORCE SERVO LIMITS
// -------------------------------------------------------------------

void init_servo_limits() {

  // load from eeprom
  for (int i = 0; i < SERVO_COUNT; i++) {
    ServoConfig *cfg = all_servos[i];
    cfg->init();
  }

  /*
  // enforce servo limits
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
      serial_printf_verbose("[%s] ID %u: limits updated to [%u - %u]\n",
                            cfg->get_key(), id, hw_min, hw_max);
    } else {
      serial_printf_verbose("[%s] ID %u: limits OK [%u - %u]\n",
                            cfg->get_key(), id, hw_min, hw_max);
    }
    // ------------------------------------------------
    // 3. Correct out-of-range positions
    // ------------------------------------------------
    uint16_t pos = dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, id);
    if (pos < hw_min) {
      serial_printf_verbose("[%s] pos=%u < min=%u → moving to min\n", cfg->get_key(), pos, hw_min);
      //safeSetGoalPosition(id, hw_min);
    } else if (pos > hw_max) {
      serial_printf_verbose("[%s] pos=%u > max=%u → moving to max\n", cfg->get_key(), pos, hw_max);
      //safeSetGoalPosition(id, hw_max);
    } else {
      // inside limits
    }
  }
  */
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
  return s->zero_ticks() + (int)round(deg / (360.0 / 4096.0) * s->dir());
}

int per2ticks(uint8_t id, double per) {
  ServoConfig *s = find_servo(id);
  if (!s) return 0;

  double min_t = s->min_ticks();
  double max_t = s->max_ticks();
  double range = max_t - min_t;
  double dir = s->dir();

  // Clamp 0–100%
  if (per < 0) per = 0;
  if (per > 100) per = 100;

  // Map percent to ticks considering direction
  double ticks = (dir > 0)
                   ? min_t + (per / 100.0) * range
                   : max_t - (per / 100.0) * range;

  return (int)round(ticks);
}

double ticks2per(uint8_t id, int ticks) {
  ServoConfig *s = find_servo(id);
  if (!s) return 0.0;

  double min_t = s->min_ticks();
  double max_t = s->max_ticks();
  double dir = s->dir();
  double range = max_t - min_t;

  double per = (dir > 0)
                 ? (ticks - min_t) * 100.0 / range
                 : (max_t - ticks) * 100.0 / range;

  // Clamp to [0, 100]
  if (per < 0) per = 0;
  if (per > 100) per = 100;
  return per;
}

double per2deg(uint8_t id, double per) {
  ServoConfig *s = find_servo(id);
  if (!s) return 0;
  int ticks = per2ticks(id, per);
  return ticks2deg(id, ticks);
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

double getPos_per(int id) {
  return ticks2per(id, dxl.getPresentPosition(id));
}

void setGoal_deg(int id, double goal_deg) {
  int goal_ticks = deg2ticks(id, goal_deg);
  safeSetGoalPosition(id, goal_ticks);
}

bool isInPosition(uint8_t id) {
  int s = dxl.readControlTableItem(ControlTableItem::MOVING_STATUS, id);
  return (s >= 0) && (s & 0x01);
}

bool isMoving(uint8_t id) {
  int s = dxl.readControlTableItem(ControlTableItem::MOVING_STATUS, id);
  return (s >= 0) && (s & 0x02);
}

int getMin_ticks(int id) {
  ServoConfig *s = find_servo(id);
  if (!s) return 0;
  return s->min_ticks();
}

int getMax_ticks(int id) {
  ServoConfig *s = find_servo(id);
  if (!s) return 0;
  return s->max_ticks();
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
    serial_printf_verbose("ERR STALL id=%d curr=%d temp=%d\n", id, curr, temp);
    return true;
  }
  return false;
}

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
    }
  }

  for (uint8_t i = startIndex; i < endIndex && i < SERVO_COUNT; i++) {
    ServoConfig *s = all_servos[i];
    uint8_t sid = s->get_id();

    if (!dxl.ping(sid)) {
      serial_printf_verbose("ERR no ping name=%s id=%d\n", s->get_key(), sid);
    } else {
      int pos_ticks = dxl.getPresentPosition(sid);
      int curr_mA = dxl.getPresentCurrent(sid);
      int temp_C = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, sid);

      double pos_deg = ticks2deg(sid, pos_ticks);
      double pos_per = ticks2per(sid, pos_ticks);  // percentage of configured range

      serial_printf_verbose("STATUS SERVO name=%s id=%d pos=%d deg=%.2f per=%.2f current_ma=%d temp_deg=%d min_ticks=%d, zero_ticks=%d, max_ticks=%d\n",
                            s->get_key(), sid, pos_ticks, pos_deg, pos_per, curr_mA, temp_C, s->min_ticks(), s->zero_ticks(), s->max_ticks());
    }
  }

  // ---- XY metrics (for 2-arm systems) ----
  if (id == 0 && dxl.ping(ID_ARM1) && dxl.ping(ID_ARM2)) {
    double _a1_servo_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
    double _a2_servo_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
    kin.solve_x_y_from_a1_a2(_a1_servo_deg, _a2_servo_deg);
    print_xy_status();

  } else if (id == 0) {
    print_xy_status();
  }
}
