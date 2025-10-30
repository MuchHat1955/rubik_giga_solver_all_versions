#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <cmath>
#include <initializer_list>
#include <algorithm>

// -------------------------------------------------------------------
//                     FORWARD DECLARATION - COMPILER ISSUE
// -------------------------------------------------------------------

class ServoConfig;

// -------------------------------------------------------------------
//                     GLOBAL CONSTANTS
// -------------------------------------------------------------------

const int STALL_CURRENT_mA = 1000;
const int TEMP_LIMIT_C = 70;

const int PV_MIN = 40, PV_MAX = 600;
const int PA_MIN = 40, PA_MAX = 600;

const float TICKS_PER_REV = 4096.0f;
const float PV_UNIT_RPM = 0.229f;
const float TIME_SAFETY_FACTOR = 1.20f;
const uint32_t MIN_WAIT_MS = 120;
const uint32_t EXTRA_SETTLE_MS = 50;

const int SMALL_THRESH_ABS = 20;
const int TEST_DEFAULT_COUNT = 40;
const int TEST_PAUSE_MS = 20;
const int STOW_TICK = 2000;
const int RAND_INCS[] = { 5, 19, 30, 50 };
const int RAND_INCS_N = sizeof(RAND_INCS) / sizeof(RAND_INCS[0]);

constexpr double TICKS_PER_DEG = 4096.0 / 360.0;
constexpr double DEG_PER_TICK = 0.087890625;
constexpr double MM_PER_TICK = 0.0767;

// -------------------------------------------------------------------
//                   GLOBAL STRUCTS & HELPERS
// -------------------------------------------------------------------

static inline double clamp(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi ? hi : v);
}

// ---------------- Test stats ----------------
struct TestStats {
  long cntSmall = 0;
  long cntLarge = 0;
  long sumAbsErrSmall = 0;
  long sumAbsErrLarge = 0;
};

// Forward declarations to satisfy the Arduino pre-parser
void print_status(uint8_t id);
void cmdInfo(uint8_t id);

// ---------------- Verbose mode ----------------
bool verboseOn = true;  // default at boot = ON

// -------------------------------------------------------------------
//                       SERVO CONFIG
// -------------------------------------------------------------------

#define DXL_SERIAL Serial1
#define DXL_DIR_PIN -1
#define PROTOCOL 2.0
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ----------------------------------------------------------
//   ServoConfig class: one instance per servo
// ----------------------------------------------------------

float arm_length_mm = 60;

/* EXAMPLES OF USAGE
  int id = ID_ARM1;
  const char *name = id2name(12);
  double deg = ticks2deg(11, 2300);
  int ticks = deg2ticks(11, 45.0);
*/
class ServoConfig {
public:
  ServoConfig(const char *key,
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

  uint8_t get_id() const {
    return id_;
  }
  const char *get_key() const {
    return key_;
  }
  uint16_t zero_ticks() const {
    return zero_ticks_;
  }
  uint16_t min_ticks() const {
    return limit_min_;
  }
  uint16_t max_ticks() const {
    return limit_max_;
  }
  double dir() const {
    return dir_;
  }

private:
  const char *key_;
  uint8_t id_;
  uint16_t zero_ticks_;
  double dir_;
  uint16_t limit_min_;
  uint16_t limit_max_;
};

// ----------------------------------------------------------
//   Global servo instances
// ----------------------------------------------------------
#define ID_ARM1 11
#define ID_ARM2 12
#define ID_GRIP 13
#define ID_GRIP1 14
#define ID_GRIP2 15
#define ID_BASE 16
#define ID_XM 17

#define TICK_ZERO 2048
#define TICK_90 3072
#define TICK_MINUS90 1024

// TODO update below once the HW is working
ServoConfig arm1("arm1", ID_ARM1, TICK_ZERO, -1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig arm2("arm2", ID_ARM2, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig grip("grip", ID_GRIP, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig grip1("grip1", ID_GRIP1, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig grip2("grip2", ID_GRIP2, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);
ServoConfig base("base", ID_BASE, TICK_ZERO, 1.0, TICK_MINUS90 - 100, TICK_90 + 100);

ServoConfig *all_servos[] = {
  &arm1, &arm2, &grip, &grip1, &grip2, &base
};
constexpr uint8_t SERVO_COUNT = sizeof(all_servos) / sizeof(all_servos[0]);

// ----------------------------------------------------------
//   INIT SERVOS LIMITS
// ----------------------------------------------------------

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
    // 2. If hardware range is wider than desired, tighten it
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
    // 3. Read current position and correct if out of bounds
    // ------------------------------------------------
    uint16_t pos = dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, id);

    if (pos < hw_min) {
      serial_printf("[%s] pos=%u < min=%u → moving to min\n",
                    cfg->get_key(), pos, hw_min);
      dxl.setGoalPosition(id, hw_min);
    } else if (pos > hw_max) {
      serial_printf("[%s] pos=%u > max=%u → moving to max\n",
                    cfg->get_key(), pos, hw_max);
      dxl.setGoalPosition(id, hw_max);
    } else {
      serial_printf("[%s] pos=%u within limits\n", cfg->get_key(), pos);
    }
  }
}

// ----------------------------------------------------------
//   Global helper functions (no class prefix)
// ----------------------------------------------------------
inline double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}
inline double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

// Find servo object by ID
inline ServoConfig *find_servo(uint8_t id) {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (all_servos[i]->get_id() == id) return all_servos[i];
  }
  return nullptr;
}

// Find servo object by name
inline ServoConfig *find_servo(const char *name) {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (strcmp(all_servos[i]->get_key(), name) == 0) return all_servos[i];
  }
  return nullptr;
}

// Convert ticks to degrees using servo config
inline double ticks2deg(uint8_t id, int ticks) {
  if (auto *s = find_servo(id))
    return (ticks - s->zero_ticks()) * (360.0 / 4096.0) * s->dir();
  return 0.0;
}

// Convert degrees to ticks using servo config
inline int deg2ticks(uint8_t id, double deg) {
  ServoConfig *s = find_servo(id);
  if (!s) return 0;
  return s->zero_ticks() + (int)(deg / (360.0 / 4096.0) * s->dir());
}

// Simple name <-> id lookups (no prefix)
inline uint8_t name2id(const char *name) {
  ServoConfig *s = find_servo(name);
  return s ? s->get_id() : 0;
}

inline const char *id2name(uint8_t id) {
  ServoConfig *s = find_servo(id);
  return s ? s->get_key() : "";
}

// -------------------------------------------------------------------
//                      LED HELPERS
// -------------------------------------------------------------------

void lOn(uint8_t id) {
  if (dxl.ping(id)) dxl.ledOn(id);
}
void lOff(uint8_t id) {
  if (dxl.ping(id)) dxl.ledOff(id);
}

// -------------------------------------------------------------------
//                      SERIAL PRINTF HELPER
// -------------------------------------------------------------------
template<typename... Args>
void serial_printf(const char *fmt, Args... args) {
  char buf[200];
  snprintf(buf, sizeof(buf), fmt, args...);
  Serial.print(buf);
}

// -------------------------------------------------------------------
//    COMMON HELPERS FOR SAME CALL ON MULTIPLE SERVERS
// -------------------------------------------------------------------
inline void torqueOnGroup(std::initializer_list<uint8_t> ids) {
  for (auto id : ids) {
    if (!dxl.ping(id)) continue;
    dxl.torqueOn(id);
  }
}

inline void adjustPwmGroup(std::initializer_list<uint8_t> ids, int pwmLimit) {
  for (auto id : ids) {
    if (!dxl.ping(id)) continue;
    dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id, pwmLimit);
  }
}

inline void torqueOffGroup(std::initializer_list<uint8_t> ids) {
  for (auto id : ids) {
    if (!dxl.ping(id)) continue;
    dxl.torqueOff(id);
    delay(30);
  }
}

inline void ledOnGroup(std::initializer_list<uint8_t> ids) {
  for (auto id : ids) {
    if (!dxl.ping(id)) continue;
    lOn(id);
  }
}

inline void ledOffGroup(std::initializer_list<uint8_t> ids) {
  for (auto id : ids) {
    if (!dxl.ping(id)) continue;
    lOff(id);
  }
}

inline bool checkStallGroup(std::initializer_list<uint8_t> ids) {
  for (auto id : ids) {
    if (checkStall(id)) return true;
  }
  return false;
}

double getPos_deg(int id) {
  return ticks2deg(id, dxl.getPresentPosition(id));
}
void setGoal_deg(int id, double goal_deg) {
  int goal_ticks = deg2ticks(id, goal_deg);
  dxl.setGoalPosition(id, goal_ticks);
}

// -------------------------------------------------------------------
//                      STATUS & UTILS
// -------------------------------------------------------------------

bool isInPosition(uint8_t id) {
  int s = dxl.readControlTableItem(ControlTableItem::MOVING_STATUS, id);
  return (s >= 0) && (s & 0x01);
}
bool isMoving(uint8_t id) {
  int s = dxl.readControlTableItem(ControlTableItem::MOVING_STATUS, id);
  return (s >= 0) && (s & 0x02);
}

// ---------------- Stall detection ----------------
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
//        VERTICAL KINEMATICS CLASS
// -------------------------------------------------------------------
class VerticalKinematics {
public:
  // -------------------------------------------------------------------
  // Geometry conventions:
  //   • Arm1 (shoulder): a1_deg = 0° when vertical up (+Y)
  //   • Arm2 (elbow): a2_deg = 0° when perpendicular to Arm1 on its left
  //   • For a1=0°, a2=0° → endpoint X=-l, Y=+l
  //   • X grows right, Y grows upward.
  // -------------------------------------------------------------------
  VerticalKinematics() {}

  // ---------------- Independent Setters ----------------
  void setA1deg(double a1) {
    a1_deg = a1;
    update_from_angles();
  }
  void setA2deg(double a2) {
    a2_deg = a2;
    update_from_angles();
  }
  void setGdeg(double g) {
    g_deg = g;
    update_g_alignment();
  }
  void setA1ticks(int ticks) {
    a1_deg = ticks2deg(ID_ARM1, ticks);
    update_from_angles();
  }
  void setA2ticks(int ticks) {
    a2_deg = ticks2deg(ID_ARM2, ticks);
    update_from_angles();
  }
  void setGticks(int ticks) {
    g_deg = ticks2deg(ID_GRIP, ticks);
    update_g_alignment();
  }

  // -------------------------------------------------------------------
  // setXYmm(x, y)
  //   • setXYmm(0, Y)   -> vertical up/down at given Y
  //   • setXYmm(X, -1)  -> horizontal offset X, preserving current Y
  //   • full inverse kinematics (updates both angles)
  // -------------------------------------------------------------------
  // ----------------------------------------------------------
  //   Solve for a1r, a2r given x_mm, y_mm, and link length arm_length_mm
  // ----------------------------------------------------------
  void solve_deg_angles_from_xy(double x_mm, double y_mm,
                                double &a1r, double &a2r) {
    double A = x_mm / arm_length_mm;
    double B = y_mm / arm_length_mm;

    // step 1: compute q
    double q = atan2(-A, B);

    // step 2: compute sin(p)
    double cos_q = cos(q);
    double sin_p = 0.0;
    if (fabs(cos_q) > 1e-9) {
      sin_p = clamp(B / (2.0 * cos_q), -1.0, 1.0);
    }
    if (sin_p > 1.0) sin_p = 1.0;
    if (sin_p < -1.0) sin_p = -1.0;

    double p = asin(sin_p);

    // step 3: compute angles
    double a1abs_right_rad = p + q;
    double a2abs_left_rad = p - q;

    /*
    double a1r = deg2rad(a1_deg);           // 0
    double a2r = deg2rad(a2_deg);           // 0
    double a1abs_right = M_PI / 2.0 - a1r;  // 90
    double a2abs_left = a1r + a2r;          // 0
    double a2abs_right = M_PI - a2r - a1r;  // 180
    */

    double a1abs_right_deg = rad2deg(a1abs_right_rad);
    double a2abs_left_deg = rad2deg(a2abs_left_rad);

    serial_printf("a1 abs right=%.2f a2 abs left=%.2f\n", a1abs_right_deg, a2abs_left_deg);

    a1r = 90 - a1abs_right_deg;
    a2r = 2 * a1abs_right_deg - 90;

    serial_printf("a1 r=%.2f a2 r=%.2f\n", a1r, a2r);
  }

  void setXYmm(double x, double y) {
    if (x < 0 && y < 0) return;
    if (x < 0) x = x_mm;
    if (y < 0) y = y_mm;

    solve_deg_angles_from_xy(x, y, a1_deg, a2_deg);

    update_from_angles();
  }

  // ---------------- Getters ----------------
  double getA1deg() const {
    return a1_deg;
  }  // absolute wrt vertical
  double getA2deg() const {
    return a2_deg;
  }  // relative (servo sees this)

  int getA1ticks() const {
    return deg2ticks(ID_ARM1, a1_deg);
  }
  int getA2ticks() const {
    return deg2ticks(ID_ARM2, a2_deg);
  }

  void readPresentPositions() {
    setA1ticks(dxl.getPresentPosition(ID_ARM1));
    setA2ticks(dxl.getPresentPosition(ID_ARM2));
    setGticks(dxl.getPresentPosition(ID_GRIP));
  }

  void setGoalPositions(double x, double y, double gdeg) {
    setXYmm(x, y);
    setGdeg(gdeg);
    update_from_angles();
    dxl.setGoalPosition(deg2ticks(ID_ARM1, a1_deg),  //
                        deg2ticks(ID_ARM2, a1_deg),  //
                        deg2ticks(ID_GRIP, getGdeg()));
  }

  double getXmm() const {
    return x_mm;
  }
  double getYmm() const {
    return y_mm;
  }
  // ---------------- Gripper math ----------------
  double getGdeg() const {
    return g_deg;
  }
  double getGdeg_aligned() const {
    // 0° = gripper vertical; positive tilts along Arm2
    // fix to account for current gripper position
    if (g_horizontal) return 180 - a1_deg - a2_deg;
    return 90 - a1_deg - a2_deg;
  }
  void update_g_alignment() {
    double g_vert_deg = 180 - a1_deg - a2_deg;
    double g_horiz_deg = 90 - a1_deg - a2_deg;
    double err_vert = fabs(g_deg - g_vert_deg);
    double err_horiz = fabs(g_deg - g_horiz_deg);
    if (err_horiz < err_vert) g_horizontal = true;
    else g_horizontal = true;
  }
  int getGticks() const {
    return deg2ticks(ID_GRIP, getGdeg());
  }
  int getGticksAligned() const {
    return deg2ticks(ID_GRIP, getGdeg_aligned());
  }

private:
  // state
  double a1_deg = 0.0;
  double a2_deg = 0.0;
  double g_deg = 0.0;
  double x_mm, y_mm;
  bool g_horizontal = false;

  // ---------------- Forward kinematics ----------------
  void update_from_angles() {
    // a1 from vertical; a2 relative (0° = left)
    double a1r_rad = deg2rad(a1_deg);                   // 0
    double a2r_rad = deg2rad(a2_deg);                   // 0
    double a1abs_right_rad = M_PI / 2.0 - a1r_rad;      // 90
    double a2abs_left_rad = a1r_rad + a2r_rad;          // 0
    double a2abs_right_rad = M_PI - a1r_rad - a2r_rad;  // 0

    // compute using absolute, 0 is vertical, 90 is horizontal towards right
    y_mm = arm_length_mm * sin(a1abs_right_rad) + arm_length_mm * sin(a2abs_left_rad);  // sin 0 + sin 0
    x_mm = arm_length_mm * cos(a1abs_right_rad) - arm_length_mm * cos(a2abs_left_rad);  // cos 90 - cos 0
  }
};

VerticalKinematics kin;

// -------------------------------------------------------------------
//                   TEST STATS HELPERS
// -------------------------------------------------------------------

namespace tst {
static inline void addErr(TestStats &s, int plannedDist, int absErr) {
  if (plannedDist < SMALL_THRESH_ABS) {
    s.cntSmall++;
    s.sumAbsErrSmall += absErr;
  } else if (plannedDist > SMALL_THRESH_ABS) {
    s.cntLarge++;
    s.sumAbsErrLarge += absErr;
  }
}
static inline void printStats(const char *label, uint8_t id, const TestStats &s) {
  long avgSmall = (s.cntSmall ? (s.sumAbsErrSmall + s.cntSmall / 2) / s.cntSmall : 0);
  long avgLarge = (s.cntLarge ? (s.sumAbsErrLarge + s.cntLarge / 2) / s.cntLarge : 0);
  serial_printf("STATS %s id=%d small_cnt=%ld avg_abs_err_small=%ld  large_cnt=%ld avg_abs_err_large=%ld\n",
                label, id, s.cntSmall, avgSmall, s.cntLarge, avgLarge);
}
}

// -------------------------------------------------------------------
//              PROFILE & TIMING UTILS (Dynamic Wait)
// -------------------------------------------------------------------

static inline float pvToTicksPerSec(int pvLSB) {
  float rpm = (pvLSB <= 0 ? 0.0f : pvLSB * PV_UNIT_RPM);
  float rps = rpm / 60.0f;
  float tps = rps * TICKS_PER_REV;
  return max(tps, 1.0f);
}
static inline uint32_t estimateTravelTimeMs(uint8_t id, int deltaTicks) {
  int pv = dxl.readControlTableItem(ControlTableItem::PROFILE_VELOCITY, id);
  float tps = pvToTicksPerSec(pv);
  float moveMs = (fabs((float)deltaTicks) / tps) * 1000.0f * TIME_SAFETY_FACTOR;
  float total = MIN_WAIT_MS + moveMs + EXTRA_SETTLE_MS;
  return (uint32_t)total;
}

// -------------------------------------------------------------------
//                   SMOOTH MOVE: slow start, coast, slow end
// -------------------------------------------------------------------
bool cmdMoveSmooth(uint8_t id, int goal) {
  if (!dxl.ping(id)) return false;

  int start = dxl.getPresentPosition(id);
  int totalDiff = goal - start;
  int dir = (totalDiff >= 0) ? 1 : -1;
  int dist = abs(totalDiff);
  if (dist < 3) return true;

  dxl.torqueOn(id);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id, 880);
  dxl.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, id, 2);
  lOn(id);

  // --- profile parameters ---
  const int stepInterval_ms = 15;  // time between micro-steps
  const int accelSteps = 20;
  const int decelSteps = 20;
  const int minStep = 1;
  const int maxStep = 25;

  int accelSpan = accelSteps * (minStep + maxStep) / 2;
  int decelSpan = decelSteps * (minStep + maxStep) / 2;
  int coastSpan = dist - (accelSpan + decelSpan);
  if (coastSpan < 0) coastSpan = 0;

  int pos = start;

  // --- acceleration phase ---
  for (int i = 0; i < accelSteps && abs(pos - start) < dist / 2; i++) {
    int step = map(i, 0, accelSteps - 1, minStep, maxStep);
    pos += dir * step;
    dxl.setGoalPosition(id, pos);
    delay(stepInterval_ms);
  }

  if (checkStall(id)) return false;

  // --- constant-speed (coast) phase ---
  int coastTicks = coastSpan / maxStep;
  for (int i = 0; i < coastTicks; i++) {
    pos += dir * maxStep;
    dxl.setGoalPosition(id, pos);
    delay(stepInterval_ms);
  }

  if (checkStall(id)) return false;

  // --- deceleration phase ---
  for (int i = decelSteps - 1; i >= 0; i--) {
    int step = map(i, 0, decelSteps - 1, minStep, maxStep);
    pos += dir * step;
    if ((dir > 0 && pos > goal) || (dir < 0 && pos < goal)) pos = goal;
    dxl.setGoalPosition(id, pos);
    delay(stepInterval_ms);
  }

  if (checkStall(id)) return false;

  // --- final settle ---
  int posNow = dxl.getPresentPosition(id);
  int err = goal - posNow;
  const int tol = 4;
  const int maxNudges = 6;
  const int nudgeDelay = 85;

  int count = 0;
  int prevPos = posNow;
  bool samePos = false;

  while (abs(err) > tol && count < maxNudges) {
    int nudge = (err > 0 ? err + 4 : err - 4);
    if (abs(err) > 6) nudge = (err > 0 ? err + 8 : err - 8);
    if (samePos) nudge = (err > 0 ? nudge + 4 : nudge - 4);

    // clamp and soften if close
    nudge = constrain(nudge, -35, 35);
    if (!samePos && abs(err) < 15 && count < 3) nudge /= 2;
    else if (abs(err) < 8 && count < 3) nudge /= 4;

    if (verboseOn)
      serial_printf("    %d) err=%d nudge=%d\n", count, err, nudge);

    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, posNow + nudge);
    delay(nudgeDelay);

    posNow = dxl.getPresentPosition(id);
    err = goal - posNow;
    samePos = (prevPos == posNow);
    prevPos = posNow;

    if (verboseOn)
      serial_printf("    %d) err=%d nudge=%d\n", count, err, nudge);
    count++;
  }

  delay(85);  // final settle
  posNow = dxl.getPresentPosition(id);
  err = goal - posNow;

  lOff(id);

  if (checkStall(id)) return false;

  if (verboseOn) {
    serial_printf("SMOOTH MOVE id=%d start=%d goal=%d final=%d err=%d (nudges=%d)\n",
                  id, start, goal, posNow, err, count);
  }
  return true;
}

// -------------------------------------------------------------------
//                        READXY using kinematics
// -------------------------------------------------------------------
void print_status(uint8_t id) {
  //---- basic status for each server ----
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
      int pos = dxl.getPresentPosition(sid);
      int curr = dxl.getPresentCurrent(sid);
      int temp = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, sid);
      double deg = ticks2deg(sid, pos);

      serial_printf("STATUS %s (id=%2u): pos=%4d deg=%.1f current=%dmA temp=%d°C\n",
                    s->get_key(), sid, pos, deg, curr, temp);
    }
  }
  if (id > 0) return;

  // ---- xy metrics ----
  if (dxl.ping(ID_ARM1) && dxl.ping(ID_ARM2)) {
    kin.readPresentPositions();
    serial_printf("STATUS XY X=%.2fmm Y=%.2fmm A1=%.2fdeg A2=%.2fdeg G=%.2fdeg  G align=%.2fdeg\n",  //
                  kin.getXmm(), kin.getYmm(), kin.getA1deg(), kin.getA2deg(), kin.getGdeg(), kin.getGdeg_aligned());
  } else {
    serial_printf("STATUS XY X=na Y=na A1=na A2=na G=na");
  }
}

void print_pos(uint8_t id) {
  if (!dxl.ping(id)) {
    serial_printf("READ id=%2d ticks=na\n", id);
    return;
  }
  int pos = dxl.getPresentPosition(id);
  serial_printf("READ id=%2d ticks=%5d\n", id, pos);
}

// -------------------------------------------------------------------
// INFO <id> : print key control-table data for one servo
// -------------------------------------------------------------------
void cmdInfo(uint8_t id) {
  if (!dxl.ping(id)) {
    serial_printf("Servo %d not found\n", id);
    return;
  }

  int op = dxl.readControlTableItem(ControlTableItem::OPERATING_MODE, id);
  int drv = dxl.readControlTableItem(ControlTableItem::DRIVE_MODE, id);
  int pv = dxl.readControlTableItem(ControlTableItem::PROFILE_VELOCITY, id);
  int pa = dxl.readControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id);
  int pos = dxl.getPresentPosition(id);
  int minL = dxl.readControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id);
  int maxL = dxl.readControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id);

  float rpm = pv * PV_UNIT_RPM;
  float tps = pvToTicksPerSec(pv);
  float spanTicks = (maxL > minL) ? (float)(maxL - minL) : TICKS_PER_REV;
  float spanDeg = spanTicks * DEG_PER_TICK;

  serial_printf("INFO id=%d\n", id);
  serial_printf("  OperatingMode : %d\n", op);
  serial_printf("  DriveMode     : %d (bit0=%s-profile)\n", drv, (drv & 0x01) ? "TIME" : "VELOCITY");
  serial_printf("  Profile Vel   : %d  (≈ %.3f rpm, %.1f ticks/s)\n", pv, rpm, tps);
  serial_printf("  Profile Accel : %d\n", pa);
  serial_printf("  PosLimits     : min=%d  max=%d  (span≈ %.1f°)\n", minL, maxL, spanDeg);
  serial_printf("  Present Pos   : %d (≈ %.2f°)\n", pos, pos * DEG_PER_TICK);
}

// -------------------------------------------------------------------
//                   SETLIMIT / SETLEN / SETZERO
// -------------------------------------------------------------------

void cmdSetLimit(uint8_t id, int minL, int maxL) {
  if (!dxl.ping(id)) return;
  dxl.writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id, minL);
  dxl.writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id, maxL);
  serial_printf("SETLIMIT id=%d min=%d max=%d\n", id, minL, maxL);
}

// -------------------------------------------------------------------
// TESTMOVE  <id> [cycles]  – test single-servo adaptive moves
// -------------------------------------------------------------------
void cmdTestMove(uint8_t id, int count = TEST_DEFAULT_COUNT) {
  if (!dxl.ping(id)) {
    serial_printf("Servo %d not found\n", id);
    return;
  }
  if (count <= 0) count = TEST_DEFAULT_COUNT;

  TestStats stats;
  serial_printf("TESTMOVE start id=%d cycles=%d\n", id, count);

  for (int iter = 0; iter < count; ++iter) {
    // small ± moves
    cmdMoveSmooth(id, STOW_TICK);
    cmdMoveSmooth(id, STOW_TICK);
    int baseline = dxl.getPresentPosition(id);
    for (int i = 0; i < RAND_INCS_N; ++i) {
      int goalUp = baseline + RAND_INCS[i];
      int goalDn = baseline - RAND_INCS[i];
      cmdMoveSmooth(id, goalUp);
      cmdMoveSmooth(id, goalDn);
      int final = dxl.getPresentPosition(id);
      tst::addErr(stats, RAND_INCS[i], abs(final - goalDn));
    }
    // large excursion
    cmdMoveSmooth(id, STOW_TICK + 1000);
    cmdMoveSmooth(id, STOW_TICK);
  }

  tst::printStats("TESTMOVE", id, stats);
  serial_printf("TESTMOVE end id=%d\n", id);
}

// -------------------------------------------------------------------
// TESTMOVEX <ticks>  – move both arms laterally
// -------------------------------------------------------------------
void cmdTestMoveX(int rel_ticks) {
  Serial.println("=== TESTMOVEX ===");

  int start1 = dxl.getPresentPosition(ID_ARM1);
  int start2 = dxl.getPresentPosition(ID_ARM2);
  cmdMoveXSyncSmooth(rel_ticks, 0, 0);  // TODO add test angles for arm2 and w
  int end1 = dxl.getPresentPosition(ID_ARM1);
  int end2 = dxl.getPresentPosition(ID_ARM2);
  serial_printf("TESTMOVEX Δticks=%d | Arm1:%d→%d  Arm2:%d→%d\n",
                rel_ticks, start1, end1, start2, end2);
  Serial.println("=================\n");
}

// -------------------------------------------------------------------
// Smooth synchronized vertical move (Arm1 + Arm2 + Gripper)
// -------------------------------------------------------------------

bool cmdMoveYSyncSmooth(double goal1_deg, double goal2_deg, double goalG_deg) {
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2) || !dxl.ping(ID_GRIP))
    return false;

  double start1_deg = getPos_deg(ID_ARM1);
  double start2_deg = getPos_deg(ID_ARM2);
  double startG_deg = getPos_deg(ID_GRIP);

  serial_printf("    START   | A1=%.2f A2=%.2f G=%.2f\n", start1_deg, start2_deg, startG_deg);
  serial_printf("    GOAL    | A1=%.2f A2=%.2f G=%.2f\n", goal1_deg, goal2_deg, goalG_deg);

  // -----------------------------------------------------------------
  // Constants and tolerances (degrees)
  // -----------------------------------------------------------------
  constexpr double DEG_PER_TICK = 360.0 / 4096.0;
  constexpr double TICK_PER_DEG = 1.0 / DEG_PER_TICK;
  constexpr double syncTol_deg = 4.0 * DEG_PER_TICK;  // about 0.35°
  constexpr double tol_deg = 4.0 * DEG_PER_TICK;      // final settle tolerance

  // Motion profile
  const int stepInterval_ms = 15;
  int accelSteps = 20;
  int decelSteps = 20;
  const int minStep = 1;
  const int maxStep = 25;

  // -----------------------------------------------------------------
  // Read starting positions (degrees)
  // -----------------------------------------------------------------

  int totalTicks = abs(deg2ticks(ID_ARM1, goal1_deg) - deg2ticks(ID_ARM1, start1_deg));
  if (totalTicks < 3) return true;

  // -----------------------------------------------------------------
  // Servo setup
  // -----------------------------------------------------------------
  torqueOnGroup({ ID_ARM1, ID_ARM2, ID_GRIP });
  ledOnGroup({ ID_ARM1, ID_ARM2, ID_GRIP });
  adjustPwmGroup({ ID_ARM1, ID_ARM2, ID_GRIP }, 880);

  // -----------------------------------------------------------------
  // Compute motion spans (trapezoidal profile)
  // -----------------------------------------------------------------
  double accelSpan = accelSteps * (minStep + maxStep) / 2.0;
  double decelSpan = decelSteps * (minStep + maxStep) / 2.0;
  double coastSpan = totalTicks - (accelSpan + decelSpan);

  // --- if the move is too short, scale down accel/decel proportionally ---
  if (accelSpan + decelSpan > totalTicks) {
    double scale = (double)totalTicks / (accelSpan + decelSpan);
    accelSpan *= scale;
    decelSpan *= scale;
    coastSpan = 0.0;

    // Adjust number of iterations proportionally
    accelSteps = (int)round(accelSteps * scale);
    decelSteps = (int)round(decelSteps * scale);
  }

  // --- compute loop iteration counts ---
  int accelSteps_i = (int)accelSteps;
  int decelSteps_i = (int)decelSteps;
  int coastSteps_i = (int)round(coastSpan / maxStep);

  // --- ensure non-negative values ---
  if (coastSteps_i < 0) coastSteps_i = 0;

  // Debug print
  serial_printf("    ACCEL STEPS=%d\n", accelSteps_i);
  serial_printf("    COAST STEPS=%d\n", coastSteps_i);
  serial_printf("    DECEL STEPS=%d\n", decelSteps_i);

  // -----------------------------------------------------------------
  // Working variables
  // -----------------------------------------------------------------
  double pos1_deg = start1_deg;
  double pos2_deg = start2_deg;
  double posG_deg = startG_deg;

  double ratio1 = 0;
  double ratio2 = 0;
  double ratioG = 0;

  if (abs(deg2ticks(ID_ARM1, goal1_deg) - deg2ticks(ID_ARM1, start1_deg)) > 3) ratio1 = 1.0;
  if (abs(deg2ticks(ID_ARM2, goal2_deg) - deg2ticks(ID_ARM2, start2_deg)) > 3) ratio2 = abs(goal2_deg - start2_deg) / abs(goal1_deg - start1_deg);
  if (abs(deg2ticks(ID_GRIP, goalG_deg) - deg2ticks(ID_GRIP, startG_deg)) > 3) ratioG = abs(goalG_deg - startG_deg) / abs(goal1_deg - start1_deg);

  if (ratio1 == 0.0 && ratio2 == 0 && ratioG == 0) return true;

  double dir1 = (goal1_deg > start1_deg) ? ratio1 : -ratio1;
  double dir2 = (goal2_deg > start2_deg) ? ratio2 : -ratio2;
  double dirG = (goalG_deg > startG_deg) ? ratioG : -ratioG;

  // -----------------------------------------------------------------
  // Sync-nudge helper (uses cached positions)
  // -----------------------------------------------------------------
  // per-axis micro-nudge to its own planned position (no cross coupling)
  auto syncNudge = [&](double &p1, double &p2, double &pg) {
    constexpr double syncTol_deg = 4.0 * (360.0 / 4096.0);  // ~0.35°
    auto nudge_one = [&](int id, double planned_deg) {
      double now = getPos_deg(id);
      double err = now - planned_deg;  // positive => ahead
      if (fabs(err) > syncTol_deg) {
        double nudge = constrain(err / 2.0, -3.0, 3.0);  // small, damped
        setGoal_deg(id, now - nudge);                    // pull back toward plan
      }
    };
    nudge_one(ID_ARM1, p1);
    nudge_one(ID_ARM2, p2);
    nudge_one(ID_GRIP, pg);
  };

  // -----------------------------------------------------------------
  // Acceleration phase
  // -----------------------------------------------------------------
  for (int i = 0; i < accelSteps_i; i++) {
    int step = map(i, 0, accelSteps_i - 1, minStep, maxStep);
    double step_deg = step * DEG_PER_TICK;

    pos1_deg += dir1 * step_deg;
    pos2_deg += dir2 * step_deg;
    posG_deg += dirG * step_deg;

    serial_printf("    %d) accell step=%.2f to A1=%.2f A2=%.2f G=%.2f\n", i, step_deg, pos1_deg, pos2_deg, posG_deg);

    setGoal_deg(ID_ARM1, pos1_deg);
    setGoal_deg(ID_ARM2, pos2_deg);
    setGoal_deg(ID_GRIP, posG_deg);
    delay(stepInterval_ms);
    syncNudge(pos1_deg, pos2_deg, posG_deg);
  }

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_GRIP })) return false;

  // -----------------------------------------------------------------
  // Coast phase
  // -----------------------------------------------------------------
  for (int i = 0; i < coastSteps_i; i++) {
    double step_deg = maxStep * DEG_PER_TICK;

    pos1_deg += dir1 * step_deg;
    pos2_deg += dir2 * step_deg;
    posG_deg += dirG * step_deg;

    serial_printf("    %d) coast step=%.2f to A1=%.2f A2=%.2f G=%.2f\n", i, step_deg, pos1_deg, pos2_deg, posG_deg);

    setGoal_deg(ID_ARM1, pos1_deg);
    setGoal_deg(ID_ARM2, pos2_deg);
    setGoal_deg(ID_GRIP, posG_deg);
    delay(stepInterval_ms);
    syncNudge(pos1_deg, pos2_deg, posG_deg);
  }

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_GRIP })) return false;

  // -----------------------------------------------------------------
  // Deceleration phase
  // -----------------------------------------------------------------
  for (int i = 0; i < decelSteps_i; i++) {
    int step = map(i, decelSteps_i - 1, 0, minStep, maxStep);
    double step_deg = step * DEG_PER_TICK;

    pos1_deg += dir1 * step_deg;
    pos2_deg += dir2 * step_deg;
    posG_deg += dirG * step_deg;

    serial_printf("    %d) decell step=%.2f to A1=%.2f A2=%.2f G=%.2f\n", i, step_deg, pos1_deg, pos2_deg, posG_deg);

    setGoal_deg(ID_ARM1, pos1_deg);
    setGoal_deg(ID_ARM2, pos2_deg);
    setGoal_deg(ID_GRIP, posG_deg);

    delay(stepInterval_ms);
    syncNudge(pos1_deg, pos2_deg, posG_deg);
  }

  // -----------------------------------------------------------------
  // Final correction phase
  // -----------------------------------------------------------------
  delay(80);  // settle

  double p1f_deg = getPos_deg(ID_ARM1);
  double p2f_deg = getPos_deg(ID_ARM2);
  double pGf_deg = getPos_deg(ID_GRIP);

  double e1_deg = goal1_deg - p1f_deg;
  double e2_deg = goal2_deg - p2f_deg;
  double eG_deg = goalG_deg - pGf_deg;

  for (int n = 0; n < 6; n++) {
    if (fabs(e1_deg) <= tol_deg &&  //
        fabs(e2_deg) <= tol_deg &&  //
        fabs(eG_deg) <= tol_deg)
      break;

    double adj1_deg = constrain(e1_deg / 3.0, -10.0 * DEG_PER_TICK, 10.0 * DEG_PER_TICK);
    double adj2_deg = constrain(e2_deg / 3.0, -10.0 * DEG_PER_TICK, 10.0 * DEG_PER_TICK);
    double adjG_deg = constrain(eG_deg / 3.0, -10.0 * DEG_PER_TICK, 10.0 * DEG_PER_TICK);

    serial_printf("    %d) final to A1=%.2f A2=%.2f G=%.2f\n", n, pos1_deg, pos2_deg, posG_deg);
    serial_printf("            --- err1=%.2f err2=%.2f errG=%.2f\n", e1_deg, e2_deg, eG_deg);
    serial_printf("            --- adj1=%.2f adj2=%.2f adjG=%.2f\n", adj1_deg, adj2_deg, adjG_deg);

    if (fabs(e1_deg) > tol_deg) setGoal_deg(ID_ARM1, p1f_deg + adj1_deg);
    if (fabs(e2_deg) > tol_deg) setGoal_deg(ID_ARM2, p2f_deg + adj2_deg);
    if (fabs(eG_deg) > tol_deg) setGoal_deg(ID_GRIP, pGf_deg + adjG_deg);

    delay(30);
    p1f_deg = getPos_deg(ID_ARM1);
    p2f_deg = getPos_deg(ID_ARM2);
    pGf_deg = getPos_deg(ID_GRIP);

    e1_deg = goal1_deg - p1f_deg;
    e2_deg = goal2_deg - p2f_deg;
    eG_deg = goalG_deg - pGf_deg;
  }

  // -----------------------------------------------------------------
  // Finish: torque release and report
  // -----------------------------------------------------------------
  adjustPwmGroup({ ID_ARM1, ID_ARM2, ID_GRIP }, 400);
  delay(30);
  ledOffGroup({ ID_ARM1, ID_ARM2, ID_GRIP });

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_GRIP })) return false;

  if (verboseOn) {
    serial_printf("MOVEY SYNC done | "
                  "A1=%.2f(err Δ%.2f)  A2=%.2f(err Δ%.2f)  G=%.2f(err Δ%.2f)\n",
                  p1f_deg, e1_deg, p2f_deg, e2_deg, pGf_deg, eG_deg);
  }

  return true;
}

// -------------------------------------------------------------------
// Smooth synchronized lateral move (Arm1 + Arm2 + Gripper)
// Each axis can have its own small delta (±ticks)
// -------------------------------------------------------------------
bool cmdMoveXSyncSmooth(int delta1, int delta2, int deltaG) {
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2) || !dxl.ping(ID_GRIP)) return false;

  // --- Starting positions and goals ---
  int start1 = dxl.getPresentPosition(ID_ARM1);
  int start2 = dxl.getPresentPosition(ID_ARM2);
  int startG = dxl.getPresentPosition(ID_GRIP);

  int goal1 = constrain(start1 + delta1, 0, 4095);
  int goal2 = constrain(start2 + delta2, 0, 4095);
  int goalG = constrain(startG + deltaG, 0, 4095);

  int maxDiff = max(max(abs(delta1), abs(delta2)), abs(deltaG));
  if (maxDiff < 2) return true;

  // --- Basic setup ---
  torqueOnGroup({ ID_ARM1, ID_ARM2, ID_GRIP });
  ledOnGroup({ ID_ARM1, ID_ARM2, ID_GRIP });
  adjustPwmGroup({ ID_ARM1, ID_ARM2, ID_GRIP }, 880);

  // --- Shared motion profile parameters ---
  const int stepInterval_ms = 15;
  const int accelSteps = 20;
  const int decelSteps = 20;
  const int minStep = 1;
  const int maxStep = 20;
  const int syncTol = 4;

  int accelSpan = accelSteps * (minStep + maxStep) / 2;
  int decelSpan = decelSteps * (minStep + maxStep) / 2;
  int coastSpan = maxDiff - (accelSpan + decelSpan);
  if (coastSpan < 0) coastSpan = 0;
  int coastTicks = coastSpan / maxStep;

  // --- Working positions ---
  int pos1 = start1, pos2 = start2, posG = startG;

  // lambda for selective sync correction
  auto selectiveNudge = [&](int g1, int g2, int g3) {
    int p1 = dxl.getPresentPosition(ID_ARM1);
    int p2 = dxl.getPresentPosition(ID_ARM2);
    int p3 = dxl.getPresentPosition(ID_GRIP);

    int e1 = g1 - p1;
    int e2 = g2 - p2;
    int e3 = g3 - p3;

    if (abs(e1) > syncTol)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ID_ARM1, p1 + constrain(e1 / 3, -3, 3));
    if (abs(e2) > syncTol)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ID_ARM2, p2 + constrain(e2 / 3, -3, 3));
    if (abs(e3) > syncTol)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ID_GRIP, p3 + constrain(e3 / 3, -3, 3));
  };

  // --- Helper to compute scaled step for each axis ---
  auto scaledStep = [&](int delta, int step) -> int {
    if (maxDiff == 0) return 0;
    return (int)round((float)delta / (float)maxDiff * step);
  };

  // --- Acceleration phase ---
  for (int i = 0; i < accelSteps; i++) {
    int step = map(i, 0, accelSteps - 1, minStep, maxStep);
    pos1 += scaledStep(delta1, step);
    pos2 += scaledStep(delta2, step);
    posG += scaledStep(deltaG, step);
    dxl.setGoalPosition(ID_ARM1, pos1);
    dxl.setGoalPosition(ID_ARM2, pos2);
    dxl.setGoalPosition(ID_GRIP, posG);
    delay(stepInterval_ms);
    selectiveNudge(goal1, goal2, goalG);
  }

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_GRIP })) return false;

  // --- Coast phase ---
  for (int i = 0; i < coastTicks; i++) {
    pos1 += scaledStep(delta1, maxStep);
    pos2 += scaledStep(delta2, maxStep);
    posG += scaledStep(deltaG, maxStep);
    dxl.setGoalPosition(ID_ARM1, pos1);
    dxl.setGoalPosition(ID_ARM2, pos2);
    dxl.setGoalPosition(ID_GRIP, posG);
    delay(stepInterval_ms);
    selectiveNudge(goal1, goal2, goalG);
  }

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_GRIP })) return false;

  // --- Deceleration phase ---
  for (int i = decelSteps - 1; i >= 0; i--) {
    int step = map(i, 0, decelSteps - 1, minStep, maxStep);
    pos1 += scaledStep(delta1, step);
    pos2 += scaledStep(delta2, step);
    posG += scaledStep(deltaG, step);
    dxl.setGoalPosition(ID_ARM1, pos1);
    dxl.setGoalPosition(ID_ARM2, pos2);
    dxl.setGoalPosition(ID_GRIP, posG);
    delay(stepInterval_ms);
    selectiveNudge(goal1, goal2, goalG);
  }

  // --- Final correction phase ---
  delay(60);
  int p1f = dxl.getPresentPosition(ID_ARM1);
  int p2f = dxl.getPresentPosition(ID_ARM2);
  int p3f = dxl.getPresentPosition(ID_GRIP);
  int e1 = goal1 - p1f;
  int e2 = goal2 - p2f;
  int e3 = goalG - p3f;

  const int tol = 4;
  for (int n = 0; n < 6; n++) {
    if (abs(e1) <= tol && abs(e2) <= tol && abs(e3) <= tol) break;
    if (abs(e1) > tol)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ID_ARM1, p1f + constrain(e1 / 3, -3, 3));
    if (abs(e2) > tol)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ID_ARM2, p2f + constrain(e2 / 3, -3, 3));
    if (abs(e3) > tol)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ID_GRIP, p3f + constrain(e3 / 3, -3, 3));
    delay(30);
    p1f = dxl.getPresentPosition(ID_ARM1);
    p2f = dxl.getPresentPosition(ID_ARM2);
    p3f = dxl.getPresentPosition(ID_GRIP);
    e1 = goal1 - p1f;
    e2 = goal2 - p2f;
    e3 = goalG - p3f;
  }

  // --- Gentle torque release ---
  adjustPwmGroup({ ID_ARM1, ID_ARM2, ID_GRIP }, 400);
  delay(30);
  ledOffGroup({ ID_ARM1, ID_ARM2, ID_GRIP });

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_GRIP })) return false;

  if (verboseOn)
    serial_printf("MOVEX SYNC done | A1=%d(%d) A2=%d(%d) G=%d(%d)\n",
                  p1f, e1, p2f, e2, p3f, e3);

  return true;
}

// -------------------------------------------------------------------
//                   SMOOTH MOVE (XYG version in mm + deg)
//                 with proportional angular motion
// -------------------------------------------------------------------

static inline double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
  if (fabs(in_max - in_min) < 1e-9) return out_min;  // prevent div0
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline double constrainf(double val, double min_val, double max_val) {
  return (val < min_val) ? min_val : (val > max_val) ? max_val
                                                     : val;
}

// -------------------------------------------------------------------

bool cmdMoveSmoothXYG(double goal_xmm, double goal_ymm, double goal_gdeg,
                      double tol_xmm, double tol_ymm, double tol_gdeg,
                      double oneTickMm, double oneTickDeg,
                      bool nudge_x_enabled = true,
                      bool nudge_y_enabled = false,
                      bool nudge_g_enabled = false) {
  const double zero_tol_mm = oneTickMm / 2.0;
  const double zero_tol_deg = oneTickDeg / 2.0;

  kin.readPresentPositions();
  double start_xmm = kin.getXmm();
  double start_ymm = kin.getYmm();
  double start_gdeg = kin.getGdeg();

  double total_dx = goal_xmm - start_xmm;
  double total_dy = goal_ymm - start_ymm;
  double total_dg = goal_gdeg - start_gdeg;

  double dist_mm = sqrt(total_dx * total_dx + total_dy * total_dy);
  if (dist_mm < (3 * oneTickMm) && fabs(total_dg) < (3 * oneTickDeg)) return true;

  const int stepInterval_ms = 15;
  const int accelSteps = 20;
  const int decelSteps = 20;
  const int minStep_ticks = 1;
  const int maxStep_ticks = 25;

  const double minStep_mm = minStep_ticks * oneTickMm;
  const double maxStep_mm = maxStep_ticks * oneTickMm;

  double accelSpan = accelSteps * (minStep_mm + maxStep_mm) / 2.0;
  double decelSpan = decelSteps * (minStep_mm + maxStep_mm) / 2.0;
  double coastSpan = dist_mm - (accelSpan + decelSpan);
  if (coastSpan < 0) coastSpan = 0;

  double x = start_xmm;
  double y = start_ymm;
  double g = start_gdeg;

  double ux = (dist_mm > 0) ? (total_dx / dist_mm) : 0;
  double uy = (dist_mm > 0) ? (total_dy / dist_mm) : 0;

  // angular increment scaled by total XY distance
  // ensures G moves proportionally with XY progress
  const double deg_per_mm = (dist_mm > 1e-6) ? (total_dg / dist_mm) : 0.0;

  double prev_err_x = 0, prev_err_y = 0, prev_err_g = 0;
  int samePosX_count = 0, samePosY_count = 0, samePosG_count = 0;

  if (verboseOn) {
    serial_printf("START SMOOTH MOVE XYG start(%.2f,%.2f,%.2f) goal(%.2f,%.2f,%.2f)\n",
                  start_xmm, start_ymm, start_gdeg, goal_xmm, goal_ymm, goal_gdeg);
    serial_printf("Nudge flags X=%s Y=%s G=%s\n",
                  nudge_x_enabled ? "ON" : "OFF",
                  nudge_y_enabled ? "ON" : "OFF",
                  nudge_g_enabled ? "ON" : "OFF");
  }

  // ---------------------------------------------------------------
  //                   helper for movement + logging
  // ---------------------------------------------------------------
  auto do_step = [&](const char *phase, int i, double step_mm) {
    x += ux * step_mm;
    y += uy * step_mm;
    g += deg_per_mm * step_mm;  // proportional angular update

    kin.setGoalPositions(x, y, g);
    delay(stepInterval_ms);

    kin.readPresentPositions();
    double curr_x = kin.getXmm();
    double curr_y = kin.getYmm();
    double curr_g = kin.getGdeg();

    double err_x = goal_xmm - curr_x;
    double err_y = goal_ymm - curr_y;
    double err_g = goal_gdeg - curr_g;

    // --- adaptive stagnation detection ---
    if (fabs(err_x - prev_err_x) < zero_tol_mm) samePosX_count++;
    else samePosX_count = 0;
    if (fabs(err_y - prev_err_y) < zero_tol_mm) samePosY_count++;
    else samePosY_count = 0;
    if (fabs(err_g - prev_err_g) < zero_tol_deg) samePosG_count++;
    else samePosG_count = 0;

    samePosX_count = min(samePosX_count, 6);
    samePosY_count = min(samePosY_count, 6);
    samePosG_count = min(samePosG_count, 6);

    prev_err_x = err_x;
    prev_err_y = err_y;
    prev_err_g = err_g;

    // --- apply nudges during motion if enabled ---
    if (nudge_x_enabled && samePosX_count > 0) {
      double correction_x = constrainf(err_x * samePosX_count * 0.5, -1.5, 1.5);
      x += correction_x;
    }
    if (nudge_y_enabled && samePosY_count > 0) {
      double correction_y = constrainf(err_y * samePosY_count * 0.5, -1.5, 1.5);
      y += correction_y;
    }
    if (nudge_g_enabled && samePosG_count > 0) {
      double correction_g = constrainf(err_g * samePosG_count * 0.5, -1.0, 1.0);
      g += correction_g;
    }

    if (verboseOn) {
      serial_printf("%s iteration %d | pos x=%.2f y=%.2f g=%.2f | errx=%.2f erry=%.2f errg=%.2f | goal x=%.2f y=%.2f g=%.2f\n",
                    phase, i, curr_x, curr_y, curr_g,
                    err_x, err_y, err_g, goal_xmm, goal_ymm, goal_gdeg);
    }
  };

  // --- acceleration phase ---
  for (int i = 0; i < accelSteps && hypot(x - start_xmm, y - start_ymm) < dist_mm / 2.0; i++) {
    double step_mm = mapf(i, 0, accelSteps - 1, minStep_mm, maxStep_mm);
    do_step("accel", i, step_mm);
  }

  // --- constant-speed (coast) phase ---
  int coastSteps = (coastSpan / maxStep_mm);
  for (int i = 0; i < coastSteps; i++) {
    do_step("coast", i, maxStep_mm);
  }

  // --- deceleration phase ---
  for (int i = decelSteps - 1; i >= 0; i--) {
    double step_mm = mapf(i, 0, decelSteps - 1, minStep_mm, maxStep_mm);
    do_step("decel", i, step_mm);
    if ((ux > 0 && x > goal_xmm) || (ux < 0 && x < goal_xmm)) x = goal_xmm;
    if ((uy > 0 && y > goal_ymm) || (uy < 0 && y < goal_ymm)) y = goal_ymm;
  }

  // ----------------------------------------------------------------
  //                       FINAL NUDGE PHASE
  // ----------------------------------------------------------------
  kin.readPresentPositions();
  double curr_x = kin.getXmm();
  double curr_y = kin.getYmm();
  double curr_g = kin.getGdeg();
  double err_xmm = goal_xmm - curr_x;
  double err_ymm = goal_ymm - curr_y;
  double err_gdeg = goal_gdeg - curr_g;

  const int maxNudges = 6;
  const int nudgeDelay = 85;
  int count = 0;
  double prev_x = curr_x, prev_y = curr_y, prev_g = curr_g;
  int samePosX = 0, samePosY = 0, samePosG = 0;

  while ((fabs(err_xmm) > tol_xmm || fabs(err_ymm) > tol_ymm || fabs(err_gdeg) > tol_gdeg) && count < maxNudges) {
    double nudge_x = (err_xmm > 0 ? err_xmm + 0.3 : err_xmm - 0.3);
    double nudge_y = (err_ymm > 0 ? err_ymm + 0.3 : err_ymm - 0.3);
    double nudge_g = (err_gdeg > 0 ? err_gdeg + 0.2 : err_gdeg - 0.2);

    // independent stagnation check
    if (fabs(curr_x - prev_x) < zero_tol_mm) samePosX++;
    else samePosX = 0;
    if (fabs(curr_y - prev_y) < zero_tol_mm) samePosY++;
    else samePosY = 0;
    if (fabs(curr_g - prev_g) < zero_tol_deg) samePosG++;
    else samePosG = 0;

    samePosX = min(samePosX, 6);
    samePosY = min(samePosY, 6);
    samePosG = min(samePosG, 6);

    nudge_x *= (1.0 + 0.5 * samePosX);
    nudge_y *= (1.0 + 0.5 * samePosY);
    nudge_g *= (1.0 + 0.5 * samePosG);

    nudge_x = constrainf(nudge_x, -1.5, 1.5);
    nudge_y = constrainf(nudge_y, -1.5, 1.5);
    nudge_g = constrainf(nudge_g, -1.0, 1.0);

    if (fabs(err_xmm) < 0.6 && count < 3) nudge_x /= 2;
    if (fabs(err_ymm) < 0.6 && count < 3) nudge_y /= 2;
    if (fabs(err_gdeg) < 0.4 && count < 3) nudge_g /= 2;

    // final always all three
    kin.setGoalPositions(curr_x + nudge_x, curr_y + nudge_y, curr_g + nudge_g);
    delay(nudgeDelay);

    prev_x = curr_x;
    prev_y = curr_y;
    prev_g = curr_g;

    kin.readPresentPositions();
    curr_x = kin.getXmm();
    curr_y = kin.getYmm();
    curr_g = kin.getGdeg();

    err_xmm = goal_xmm - curr_x;
    err_ymm = goal_ymm - curr_y;
    err_gdeg = goal_gdeg - curr_g;

    if (verboseOn) {
      serial_printf("final iteration %d | pos x=%.2f y=%.2f g=%.2f | errx=%.2f erry=%.2f errg=%.2f | goal x=%.2f y=%.2f g=%.2f\n",
                    count, curr_x, curr_y, curr_g,
                    err_xmm, err_ymm, err_gdeg,
                    goal_xmm, goal_ymm, goal_gdeg);
    }
    count++;
  }

  delay(85);
  kin.readPresentPositions();
  curr_x = kin.getXmm();
  curr_y = kin.getYmm();
  curr_g = kin.getGdeg();
  err_xmm = goal_xmm - curr_x;
  err_ymm = goal_ymm - curr_y;
  err_gdeg = goal_gdeg - curr_g;

  if (verboseOn) {
    serial_printf("SMOOTH MOVE XYG DONE start(%.2f,%.2f,%.2f) goal(%.2f,%.2f,%.2f) final(%.2f,%.2f,%.2f)\n",
                  start_xmm, start_ymm, start_gdeg,
                  goal_xmm, goal_ymm, goal_gdeg,
                  curr_x, curr_y, curr_g);
    serial_printf("FINAL ERROR errx=%.3f erry=%.3f errg=%.3f (nudges=%d)\n",
                  err_xmm, err_ymm, err_gdeg, count);
  }

  return true;
}

// -------------------------------------------------------------------
//                   SMOOTH MOVE (Gripper-only version)
//             no mid-move nudging, only final settle
// -------------------------------------------------------------------
bool cmdMoveSmoothG(double goal_gdeg,
                    double tol_gdeg,
                    double oneTickDeg) {
  const double zero_tol_deg = oneTickDeg / 2.0;

  kin.readPresentPositions();
  double start_gdeg = kin.getGdeg();
  double total_dg = goal_gdeg - start_gdeg;
  double dist_deg = fabs(total_dg);
  if (dist_deg < (3 * oneTickDeg)) return true;  // skip tiny moves

  const int stepInterval_ms = 15;
  const int accelSteps = 20;
  const int decelSteps = 20;
  const int minStep_ticks = 1;
  const int maxStep_ticks = 25;

  const double minStep_deg = minStep_ticks * oneTickDeg;
  const double maxStep_deg = maxStep_ticks * oneTickDeg;

  double accelSpan = accelSteps * (minStep_deg + maxStep_deg) / 2.0;
  double decelSpan = decelSteps * (minStep_deg + maxStep_deg) / 2.0;
  double coastSpan = dist_deg - (accelSpan + decelSpan);
  if (coastSpan < 0) coastSpan = 0;

  double g = start_gdeg;
  int dir = (total_dg >= 0) ? 1 : -1;

  if (verboseOn) {
    serial_printf("START SMOOTH MOVE G start=%.2f goal=%.2f\n", start_gdeg, goal_gdeg);
  }

  // ---------------------------------------------------------------
  //                     Step helper with logging
  // ---------------------------------------------------------------
  auto do_step = [&](const char *phase, int i, double step_deg) {
    g += dir * step_deg;
    kin.readPresentPositions();
    double xCrr = kin.getXmm();
    double yCrr = kin.getYmm();
    kin.setGoalPositions(xCrr, yCrr, g);

    delay(stepInterval_ms);

    kin.readPresentPositions();
    double curr_g = kin.getGdeg();
    double err_g = goal_gdeg - curr_g;

    if (verboseOn) {
      serial_printf("%s iteration %d | pos g=%.2f | errg=%.2f | goal g=%.2f\n",
                    phase, i, curr_g, err_g, goal_gdeg);
    }
  };

  // --- acceleration phase ---
  for (int i = 0; i < accelSteps && fabs(g - start_gdeg) < dist_deg / 2.0; i++) {
    double step_deg = mapf(i, 0, accelSteps - 1, minStep_deg, maxStep_deg);
    do_step("accel", i, step_deg);
  }

  // --- constant-speed (coast) phase ---
  int coastSteps = (coastSpan / maxStep_deg);
  for (int i = 0; i < coastSteps; i++) {
    do_step("coast", i, maxStep_deg);
  }

  // --- deceleration phase ---
  for (int i = decelSteps - 1; i >= 0; i--) {
    double step_deg = mapf(i, 0, decelSteps - 1, minStep_deg, maxStep_deg);
    do_step("decel", i, step_deg);
    if ((dir > 0 && g > goal_gdeg) || (dir < 0 && g < goal_gdeg)) g = goal_gdeg;
  }

  // ----------------------------------------------------------------
  //                       FINAL NUDGE PHASE
  // ----------------------------------------------------------------
  kin.readPresentPositions();
  double curr_g = kin.getGdeg();
  double err_gdeg = goal_gdeg - curr_g;

  const int maxNudges = 6;
  const int nudgeDelay = 85;
  int count = 0;
  double prev_g = curr_g;
  int samePosG = 0;

  while (fabs(err_gdeg) > tol_gdeg && count < maxNudges) {
    double nudge_g = (err_gdeg > 0 ? err_gdeg + 0.2 : err_gdeg - 0.2);
    if (fabs(curr_g - prev_g) < zero_tol_deg) samePosG++;
    else samePosG = 0;
    if (samePosG > 6) samePosG = 6;

    nudge_g *= (1.0 + 0.5 * samePosG);
    nudge_g = constrainf(nudge_g, -1.0, 1.0);
    if (fabs(err_gdeg) < 0.4 && count < 3) nudge_g /= 2;

    kin.readPresentPositions();
    double xCrr = kin.getXmm();
    double yCrr = kin.getYmm();
    kin.setGoalPositions(xCrr, yCrr, curr_g + nudge_g);

    delay(nudgeDelay);

    kin.readPresentPositions();
    prev_g = curr_g;
    curr_g = kin.getGdeg();
    err_gdeg = goal_gdeg - curr_g;

    if (verboseOn) {
      serial_printf("final iteration %d | pos g=%.2f | errg=%.2f | goal g=%.2f\n",
                    count, curr_g, err_gdeg, goal_gdeg);
    }
    count++;
  }

  delay(85);

  kin.readPresentPositions();
  curr_g = kin.getGdeg();
  err_gdeg = goal_gdeg - curr_g;

  if (verboseOn) {
    serial_printf("SMOOTH MOVE G DONE start=%.2f goal=%.2f final=%.2f | errg=%.3f (nudges=%d)\n",
                  start_gdeg, goal_gdeg, curr_g, err_gdeg, count);
  }

  return true;
}

// -------------------------------------------------------------------
// MOVE TO ABSOLUTE Y (mm) USING KINEMATICS + SMOOTH SYNC MOVE
// -------------------------------------------------------------------
bool cmdMoveYmm(double y_mm) {

  // Clamp Y to mechanical limits
  if (y_mm < 40) return false;
  if (y_mm > 110) return false;

  kin.readPresentPositions();
  double x_mm_curr = kin.getXmm();
  kin.setXYmm(x_mm_curr, y_mm);
  double g_goal_deg = kin.getGdeg_aligned();

  return cmdMoveSmoothXYG(x_mm_curr, y_mm, g_goal_deg,
                          3 * MM_PER_TICK, 6 * MM_PER_TICK, 3 * DEG_PER_TICK,  // TOL in MM, DEG
                          3 * MM_PER_TICK, 3 * DEG_PER_TICK,                   // size of steps in MM, DEG
                          true,                                                // nudge x during move
                          false,                                               // nudge y during move
                          false);                                              // nudge g during move
}

bool cmdMoveXmm(double x_mm) {

  // Clamp Y to mechanical limits
  if (x_mm > 55) return false;

  kin.readPresentPositions();
  double y_mm_curr = kin.getYmm();
  kin.setXYmm(x_mm, y_mm_curr);
  double g_goal_deg = kin.getGdeg_aligned();

  return cmdMoveSmoothXYG(x_mm, y_mm_curr, g_goal_deg,
                          3 * MM_PER_TICK, 6 * MM_PER_TICK, 3 * DEG_PER_TICK,  // TOL in MM, DEG
                          3 * MM_PER_TICK, 3 * DEG_PER_TICK,                   // size of steps in MM, DEG
                          false,                                               // nudge x during move
                          false,                                               // nudge y during move
                          false);                                              // nudge g during move
}

// -------------------------------------------------------------------
//                          SETUP
// -------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  dxl.begin(57600);
  dxl.setPortProtocolVersion(PROTOCOL);

  Serial.println("---- Dynamixel xl430 Controller v16 -------------------------------------");
  Serial.println();

  // quick test for all servos
  for (auto *s : all_servos) {
    uint8_t id = s->get_id();

    if (dxl.ping(id)) {
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_POSITION);
      dxl.torqueOn(id);

      // Read current limits from servo
      uint16_t hw_min = dxl.readControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id);
      uint16_t hw_max = dxl.readControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id);

      // Read configured limits from your class
      uint16_t cfg_min = s->min_ticks();
      uint16_t cfg_max = s->max_ticks();

      serial_printf("Servo %s (id=%u) OK | cfg[%u-%u] hw[%u-%u]\n",
                    s->get_key(), id, cfg_min, cfg_max, hw_min, hw_max);
    } else {
      serial_printf("Servo %s (id=%u) NOT RESPONDING\n", s->get_key(), id);
    }
  }

  Serial.println();
  Serial.println("Supported Commands (VERBOSE ON):");
  Serial.println("  VERBOSEON / VERBOSEOFF         - toggle console verbosity");
  Serial.println("  SETLIMIT <id> <min> <max>      - set soft position limits");
  Serial.println("  MOVE  <id> <absgoal|±rel>      - adaptive move one servo");
  Serial.println("  MOVEDEG  <id> <deg>            - adaptive move one servo");
  Serial.println("  MOVEY <float mm>               - vertical (using curr Z)");
  Serial.println("  MOVEX <float mm>               - lateral (keep y)");
  Serial.println("  MOVECENTER                     - move all to mid at TICK_ZERO ticks");
  Serial.println("  MOVEXDEG <int ang>             - vertical (1 relative angle arm1/arm2)");
  Serial.println("  MOVEYDEG <ang1> <ang2> <angw>  - lateral (3 rel angles for 1,2,w)");
  Serial.println("  READ <id> | 0 for all          - servos basics");
  Serial.println("  TESTMOVE  <id> [cycles]        - adaptive single-servo test");
  Serial.println("  INFO  <id>                     - show servo mode/limits/profiles");
  Serial.println("  LEDON <id> / LEDOFF <id>");
  Serial.println("  (Boot default = VERBOSE ON)");
  Serial.println();

  enforce_servo_limits();
  Serial.println();

  Serial.println("------------------ End Setup --------------------------------------------");
  Serial.println();
}

#pragma once
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// arduino_scanf(line, format, ...)
// Lightweight sscanf alternative for Arduino platforms.
//
// Supported formats (space-delimited):
//   %d   int
//   %ld  long
//   %f   float / double
//
// Notes:
//   - Ignores leading/trailing whitespace
//   - Case-insensitive command match
//   - Returns number of successfully parsed arguments
// ---------------------------------------------------------------------------

int arduino_sscanf(const String &line, const char *fmt, void *out) {
  // make a trimmed copy
  String s = line;
  s.trim();

  serial_printf("line is %s\n", s.c_str());

  // Skip command word (everything up to first space)
  int space_idx = s.indexOf(' ');
  if (space_idx < 0) return 0;  // no space found
  int i = space_idx + 1;
  while (i < (int)s.length() && isspace((unsigned char)s[i])) i++;
  if (i >= (int)s.length()) return 0;

  const char *p = s.c_str() + i;
  char *endp = nullptr;

  serial_printf("number string is %s\n", p);
  serial_printf("fmt is %s\n", fmt);
  serial_printf("float is %f\n", strtod(p, &endp));

  // Handle type
  if (strcmp(fmt, "%d") == 0) {
    *(int *)out = strtol(p, &endp, 10);
  } else if (strcmp(fmt, "%ld") == 0) {
    *(long *)out = strtol(p, &endp, 10);
  } else if (strcmp(fmt, "%f") == 0 || strcmp(fmt, "%lf") == 0) {
    *(double *)out = strtod(p, &endp);
  } else {
    return 0;  // unsupported
  }

  serial_printf("p is %s\n", p);
  serial_printf("endp is %s\n", endp);

  return (endp != p) ? 1 : 0;
}


// -------------------------------------------------------------------
//                            LOOP
// -------------------------------------------------------------------

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  String U = line;
  U.toUpperCase();

  // -------------- VERBOSE MODE --------------
  if (U.startsWith("VERBOSEON")) {
    verboseOn = true;
    Serial.println("Verbose ON");
  } else if (U.startsWith("VERBOSEOFF")) {
    verboseOn = false;
    Serial.println("Verbose OFF");
  }

  // -------------- MOVE SINGLE SERVO --------------
  else if (U.startsWith("MOVE ")) {
    int id = 0;
    char valStr[16] = { 0 };
    if (sscanf(line.c_str(), "MOVE %d %15s", &id, valStr) == 2) {
      int current = dxl.getPresentPosition(id);
      int goal = 0;
      if (valStr[0] == '+' || valStr[0] == '-') {  // relative move
        goal = current + atoi(valStr);
        serial_printf("Relative move: start=%d rel=%d goal=%d\n",
                      current, atoi(valStr), goal);
      } else {  // absolute move
        goal = atoi(valStr);
      }
      if (!cmdMoveSmooth((uint8_t)id, goal)) Serial.println("MOVE ERR");
      else Serial.println("MOVE OK");
      print_status(id);
    } else {
      if (verboseOn) Serial.println("Usage: MOVE <id> <absgoal|±rel>");
    }
  }

  // -------------- MOVE SINGLE SERVO BY ANGLE
  else if (U.startsWith("MOVEDEG ")) {
    int angle = 0;
    int id = 0;
    if (sscanf(line.c_str(), "MOVEDEG %d %d", &id, &angle) == 2) {
      int goal = deg2ticks(id, angle);
      if (!cmdMoveSmooth((uint8_t)id, goal)) Serial.println("MOVE ERR");
      else Serial.println("MOVE OK");
      print_status(id);
    } else {
      if (verboseOn) Serial.println("Usage: MOVE <id> <absgoal|±rel>");
    }
  }

  // -------------- MOVE ALL TO CENTER --------------
  else if (U.startsWith("MOVECENTER")) {
    if (verboseOn) Serial.println("Moving servos to TICK_ZERO");
    for (int i = 0; i < SERVO_COUNT; i++) {
      auto *s = all_servos[i];
      if (verboseOn) serial_printf("Moving servo [%d] to %d\n", s->get_id(), TICK_ZERO);
      dxl.setGoalPosition(s->get_id(), TICK_ZERO);
    }
  }

  // -------------- MOVEXDEG (3 relative tick deltas: a1, a2, aw) --------------
  else if (U.startsWith("MOVEXDEG")) {
    int a1, a2, aw;
    if (sscanf(line.c_str(), "MOVEXDEG %d %d %d", &a1, &a2, &aw) == 3) {
      if (!cmdMoveXSyncSmooth(a1, a2, aw)) Serial.println("MOVEXDEG ERR");
      else Serial.println("MOVEXDEG OK");
      print_status(0);
    } else if (verboseOn) {
      Serial.println("Usage: MOVEXDEG <a1_ticks> <a2_ticks> <aw_ticks>");
    }
  }

  // -------------- MOVE Y AXIS (absolute mm) --------------
  else if (U.startsWith("MOVEY")) {
    double ymm = 0;
    if (arduino_sscanf(line.c_str(), "%lf", &ymm) == 1) {
      if (!cmdMoveYmm(ymm)) Serial.println("MOVEY ERR");
      else Serial.println("MOVEY OK");
      print_status(0);
    } else if (verboseOn) {
      Serial.println("Usage: MOVEY <target_y_mm>");
    }
  }

  // -------------- MOVE X AXIS (relative mm offset) --------------
  else if (U.startsWith("MOVEX")) {
    double val = 0;
    if (sscanf(line.c_str(), "MOVEX %lf", &val) == 1) {
      if (!cmdMoveXmm(val)) Serial.println("MOVEX ERR");
      else Serial.println("MOVEX OK");
      print_status(0);
    } else if (verboseOn) {
      Serial.println("Usage: MOVEX <delta_x_mm>");
    }
  }

  // -------------- READ STATUS (all servos) --------------
  else if (U.startsWith("READ")) {
    int id = 0;
    if (sscanf(line.c_str(), "READ %d", &id) == 1)
      print_status(id);
    else if (verboseOn)
      Serial.println("Usage: READ <id> | 0 for all");
  }

  // -------------- INFO (servo parameters) --------------
  else if (U.startsWith("INFO")) {
    int id = 0;
    if (sscanf(line.c_str(), "INFO %d", &id) == 1)
      cmdInfo(id);
    else if (verboseOn)
      Serial.println("Usage: INFO <id>");
  }

  // -------------- LED CONTROL --------------
  else if (U.startsWith("LEDON")) {
    int id = line.substring(6).toInt();
    dxl.ledOn(id);
    if (verboseOn) serial_printf("LED ON %d\n", id);
  } else if (U.startsWith("LEDOFF")) {
    int id = line.substring(7).toInt();
    dxl.ledOff(id);
    if (verboseOn) serial_printf("LED OFF %d\n", id);
  }

  // -------------- TEST MOVE (single servo cycles) --------------
  else if (verboseOn && U.startsWith("TESTMOVE")) {
    int id = 0, cnt = TEST_DEFAULT_COUNT;
    if (sscanf(line.c_str(), "TESTMOVE %d %d", &id, &cnt) >= 1)
      cmdTestMove((uint8_t)id, cnt);
    else if (verboseOn)
      Serial.println("Usage: TESTMOVE <id> [cycles]");
  }

  // -------------- UNKNOWN COMMAND --------------
  else if (verboseOn) {
    Serial.println("ERR: Unknown command");
  }

  Serial.println();
}
