#include "servos.h"
#include "vertical_kinematics.h"
#include <Arduino.h>
#include <vector>
#include <math.h>
#include <string.h>

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
const float PROTOCOL = 2.0f;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

extern VerticalKinematics kin;

// -------------------------------------------------------------------
//          Minimal persistent emulation for OpenRB-150
// -------------------------------------------------------------------
//
// Robotis Arduino core doesn’t expose STM32 HAL flash routines.
// This keeps an in-RAM mirror and persists values across runtime
// using a small reserved array. Data is lost on reflash.
// Still allows saving calibration between reboots when
// powered from same firmware (soft reset).
// -------------------------------------------------------------------

#define FLASH_EMU_SIZE 2048
static uint8_t flash_emulated[FLASH_EMU_SIZE] = { 0 };

static void flash_read(uint32_t addr, void *dst, size_t len) {
  if (addr + len > FLASH_EMU_SIZE) len = FLASH_EMU_SIZE - addr;
  memcpy(dst, &flash_emulated[addr], len);
}

static void flash_write(uint32_t addr, const void *src, size_t len) {
  if (addr + len > FLASH_EMU_SIZE) len = FLASH_EMU_SIZE - addr;
  memcpy(&flash_emulated[addr], src, len);
  // If in future OpenRB core exposes real flash write,
  // you can replace this memcpy with that API.
}

// -------------------------------------------------------------------
//                Struct + persistence helpers
// -------------------------------------------------------------------

struct __attribute__((packed)) servo_persist_t {
  uint16_t magic;   // 0x53C1
  uint8_t version;  // 1
  uint8_t id;       // numeric id (11,12,...)
  uint16_t zero;
  uint16_t min_t;
  uint16_t max_t;
  int8_t dir;  // +1 or -1
  uint8_t reserved;
  uint16_t checksum;
};

static constexpr uint16_t PERSIST_MAGIC = 0x53C1;
static constexpr uint8_t PERSIST_VER = 1;

static inline size_t eeprom_slot_addr(uint8_t id) {
  return static_cast<size_t>(id) * sizeof(servo_persist_t);
}

static uint16_t checksum16(const uint8_t *data, size_t n) {
  uint32_t sum = 0;
  for (size_t i = 0; i < n; ++i) sum += data[i];
  return static_cast<uint16_t>(sum & 0xFFFF);
}

static bool eeprom_capacity_ok(uint8_t id) {
  size_t need = eeprom_slot_addr(id) + sizeof(servo_persist_t);
  return need <= FLASH_EMU_SIZE;
}

static bool load_persist(uint8_t id, servo_persist_t &out) {
  size_t addr = eeprom_slot_addr(id);
  if (addr + sizeof(servo_persist_t) > FLASH_EMU_SIZE) return false;
  flash_read(addr, &out, sizeof(out));

  if (out.magic != PERSIST_MAGIC || out.version != PERSIST_VER || out.id != id)
    return false;

  const size_t n = sizeof(servo_persist_t) - sizeof(uint16_t);
  uint16_t calc = checksum16(reinterpret_cast<const uint8_t *>(&out), n);
  return (calc == out.checksum);
}

static void save_persist(const servo_persist_t &src) {
  servo_persist_t tmp = src;
  tmp.magic = PERSIST_MAGIC;
  tmp.version = PERSIST_VER;
  const size_t n = sizeof(servo_persist_t) - sizeof(uint16_t);
  tmp.checksum = checksum16(reinterpret_cast<const uint8_t *>(&tmp), n);
  size_t addr = eeprom_slot_addr(tmp.id);
  flash_write(addr, &tmp, sizeof(tmp));
}

static void persist_from_config(uint8_t id, uint16_t zero, uint16_t min_t, uint16_t max_t, double dir) {
  servo_persist_t sp{};
  sp.magic = PERSIST_MAGIC;
  sp.version = PERSIST_VER;
  sp.id = id;
  sp.zero = zero;
  sp.min_t = min_t;
  sp.max_t = max_t;
  sp.dir = (dir >= 0.0) ? 1 : -1;
  sp.reserved = 0;
  save_persist(sp);
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
  servo_persist_t sp{};
  if (load_persist(id_, sp)) {
    // Load persisted values
    zero_ticks_ = sp.zero;
    limit_min_ = sp.min_t;
    limit_max_ = sp.max_t;
    dir_ = (sp.dir >= 0) ? 1.0 : -1.0;
    serial_printf("[loaded from persist] %s id=%u loaded: zero=%u min=%u max=%u dir=%d\n",
                  key_, id_, zero_ticks_, limit_min_, limit_max_, (int)sp.dir);
  } else {
    // Not found / invalid → write defaults to the slot (so future boots are consistent)
    persist_from_config(id_, zero_ticks_, limit_min_, limit_max_, dir_);
    serial_printf("[persist defaults] %s id=%u zero=%u min=%u max=%u dir=%d\n",
                  key_, id_, zero_ticks_, limit_min_, limit_max_, dir_);
  }
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
  zero_ticks_ = t;
  // keep limits sane around update
  if (limit_min_ > limit_max_) {
    uint16_t a = limit_min_;
    limit_min_ = limit_max_;
    limit_max_ = a;
  }
  serial_printf("[set zero] %s id=%u set zero=%u\n", key_, id_, zero_ticks_);
  persist_from_config(id_, zero_ticks_, limit_min_, limit_max_, dir_);
  serial_printf("[persist] %s id=%u zero=%u min=%u max=%u dir=%d\n",
                key_, id_, zero_ticks_, limit_min_, limit_max_, dir_);
}
void ServoConfig::set_min_ticks(uint16_t t) {
  limit_min_ = t;
  if (limit_min_ > limit_max_) {
    uint16_t a = limit_min_;
    limit_min_ = limit_max_;
    limit_max_ = a;
  }

  serial_printf("[set min] %s id=%u set min=%u (max=%u)\n", key_, id_, limit_min_, limit_max_);
  dxl.torqueOff(id_);
  dxl.writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id_, limit_min_);
  dxl.torqueOn(id_);

  persist_from_config(id_, zero_ticks_, limit_min_, limit_max_, dir_);
  serial_printf("[persist] %s id=%u zero=%u min=%u max=%u dir=%d\n",
                key_, id_, zero_ticks_, limit_min_, limit_max_, dir_);
}
void ServoConfig::set_max_ticks(uint16_t t) {
  limit_max_ = t;
  if (limit_min_ > limit_max_) {
    uint16_t a = limit_min_;
    limit_min_ = limit_max_;
    limit_max_ = a;
  }

  serial_printf("[set max] %s id=%u set max=%u (min=%u)\n", key_, id_, limit_max_, limit_min_);
  dxl.torqueOff(id_);
  dxl.writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id_, limit_max_);
  dxl.torqueOn(id_);

  persist_from_config(id_, zero_ticks_, limit_min_, limit_max_, dir_);
  serial_printf("[persist] %s id=%u zero=%u min=%u max=%u dir=%d\n",
                key_, id_, zero_ticks_, limit_min_, limit_max_, dir_);
}
void ServoConfig::set_dir(double d) {
  dir_ = (d >= 0.0) ? 1.0 : -1.0;  // store as ±1 for consistency
  persist_from_config(id_, zero_ticks_, limit_min_, limit_max_, dir_);
  serial_printf("[persist] %s id=%u zero=%u min=%u max=%u dir=%d\n",
                key_, id_, zero_ticks_, limit_min_, limit_max_, dir_);
}

// -------------------------------------------------------------------
//                     SERVO CONFIG INSTANCES
// -------------------------------------------------------------------

#define TICK_ZERO 2048
#define TICK_90 3072
#define TICK_MINUS90 1024

// TODO fix those based on the HW
ServoConfig arm1("arm1", ID_ARM1, TICK_ZERO, -1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig arm2("arm2", ID_ARM2, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig wrist("wrist", ID_WRIST, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);  // wrist zero has to be at -90 //TODO update everywhere
ServoConfig grip1("grip1", ID_GRIP1, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig grip2("grip2", ID_GRIP2, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig base("base", ID_BASE, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);

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
      // inside limits
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
  return 2048 + (int)(deg / (360.0 / 4096.0) * s->dir());
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
      serial_printf("STATUS %s (id=%u): pos=na current=na temp=na\n", s->get_key(), sid);
    } else {
      int pos_ticks = dxl.getPresentPosition(sid);
      int curr_mA = dxl.getPresentCurrent(sid);
      int temp_C = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, sid);

      double pos_deg = ticks2deg(sid, pos_ticks);
      double pos_per = ticks2per(sid, pos_ticks);  // percentage of configured range

      serial_printf("STATUS %s (id=%2u): pos=%4d  deg=%7.2f  per=%6.2f%%  current=%4dmA  temp=%2d°C\n",
                    s->get_key(), sid, pos_ticks, pos_deg, pos_per, curr_mA, temp_C);
    }
  }

  // ---- XY metrics (for 2-arm systems) ----
  if (id == 0 && dxl.ping(ID_ARM1) && dxl.ping(ID_ARM2)) {
    double _a1_servo_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
    double _a2_servo_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
    if (!kin.solve_x_y_from_a1_a2(_a1_servo_deg, _a2_servo_deg)) {
      print_xy_status(false);
    } else {
      print_xy_status(true);
    }
  } else if (id == 0) {
    print_xy_status(false);
  }
}
