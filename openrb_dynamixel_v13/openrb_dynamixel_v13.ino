#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <cmath>
#include <initializer_list>
#include <algorithm>

class ServoConfig;

// -------------------------------------------------------------------
//                   GLOBAL STRUCTS & HELPERS
// -------------------------------------------------------------------

// constants
static constexpr double TICKS_PER_DEG = 4096.0 / 360.0;
static constexpr double DEG_PER_TICK = 360.0 / 4096.0;

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
#define ID_WRIST 13
#define ID_GRIP1 14
#define ID_GRIP2 15
#define ID_BASE 16
#define ID_XM 17

// TODO update below once the HW is working
ServoConfig arm1("arm1", ID_ARM1, 2048, 1.0, 0, 4096);
ServoConfig arm2("arm2", ID_ARM2, 2048, 1.0, 0, 4096);
ServoConfig wrist("wrist", ID_WRIST, 2048, 1.0, 0, 4096);
ServoConfig grip1("grip1", ID_GRIP1, 2048, 1.0, 0, 4096);
ServoConfig grip2("grip2", ID_GRIP2, 2048, 1.0, 0, 4096);
ServoConfig base("base", ID_BASE, 2048, 1.0, 0, 4096);
ServoConfig xm("xm", ID_XM, 2048, 1.0, 0, 4096);

ServoConfig *all_servos[] = {
  &arm1, &arm2, &wrist, &grip1, &grip2, &base, &xm
};
constexpr uint8_t SERVO_COUNT = sizeof(all_servos) / sizeof(all_servos[0]);

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
//                     GLOBAL CONSTANTS
// -------------------------------------------------------------------

const int STALL_CURRENT_mA = 1000;
const int TEMP_LIMIT_C = 70;

const int PV_MIN = 40, PV_MAX = 600;
const int PA_MIN = 40, PA_MAX = 600;

const int SMALL_MOVE_TICKS = 10;
const int FINE_WINDOW_TICKS = 40;
const int SMALL_OVER_UP_TICKS = 6;
const int SMALL_OVER_DOWN_TICKS = 2;
const int OVER_DELAY_MS_PER_TICK = 8;

const int COARSE_PV = 300;
const int COARSE_PA = 200;
const int COARSE_PG = 900;
const int COARSE_DG = 100;

const int FINE_PV = 80;
const int FINE_PA = 80;
const int FINE_PG = 1200;
const int FINE_DG = 10;
const int FINE_IG = 300;

const int FINISH_TOL_TICKS = 1;
const int MICROJOG_MAX_MS = 600;
const int MICROJOG_SAMPLE_DELAY = 8;

const uint32_t SMALL_MOVE_TIMEOUT_MS = 600;
const uint32_t LARGE_MOVE_TIMEOUT_MS = 3000;

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

// -------------------------------------------------------------------
//                      CALIBRATION FLAGS
// -------------------------------------------------------------------

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
  void
  setA1deg(double a1) {
    a1_deg = a1;
    update_from_angles();
  }
  void setA2deg(double a2) {
    a2_deg = a2;
    update_from_angles();
  }
  void setA1ticks(int ticks) {
    a1_deg = ticks2deg(ID_ARM1, ticks);
    update_from_angles();
  }
  void setA2ticks(int ticks) {
    a2_deg = ticks2deg(ID_ARM2, ticks);
    update_from_angles();
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
  void solve_angles_from_xy(double x_mm, double y_mm,
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

    int a1abs_right = rad2deg(a1abs_right_rad);
    int a2abs_left = rad2deg(a2abs_left_rad);

    a1r = 90 - a1abs_right;
    a2r = a2abs_left - a1r;
  }

  void setXYmm(double x, double y) {
    if (x < 0 && y < 0) return;
    if (x < 0) x = x_mm;
    if (y < 0) y = y_mm;

    double a1_rel = 0;
    double a2_rel = 0;

    solve_angles_from_xy(x, y, a1_rel, a2_rel);

    a1_deg = rad2deg(a1_rel);
    a2_deg = rad2deg(a2_rel);
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

  double getXmm() const {
    return x_mm;
  }
  double getYmm() const {
    return y_mm;
  }

  // ---------------- Gripper math ----------------
  double getGripperAng() const {
    // 0° = gripper vertical; positive tilts along Arm2
    return a1_deg + a2_deg - 90.0;
  }
  int getGripperTicks() const {
    return deg2ticks(ID_WRIST, getGripperAng());
  }

private:
  // state
  double a1_deg = 0.0;
  double a2_deg = 0.0;
  double x_mm, y_mm;

  // ---------------- Forward kinematics ----------------
  void update_from_angles() {
    // a1 from vertical; a2 relative (0° = left)
    double a1r = deg2rad(a1_deg);           // 0
    double a2r = deg2rad(a2_deg);           // 0
    double a1abs_right = M_PI / 2.0 - a1r;  // 90
    double a2abs_left = a1r + a2r;          // 0

    // compute using absolute, 0 is vertical, 90 is horizontal towards right
    y_mm = arm_length_mm * sin(a1abs_right) + arm_length_mm * sin(a2abs_left);  // sin 0 + sin 0
    x_mm = arm_length_mm * cos(a1abs_right) - arm_length_mm * cos(a2abs_left);  // cos 90 - cos 0
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

    if (!dxl.ping(id)) {
      serial_printf("STATUS %s (id=%u): pos=na current=na temp=na\n", s->get_key(), sid);
    } else {
      int pos = dxl.getPresentPosition(sid);
      int curr = dxl.getPresentCurrent(sid);
      int temp = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, sid);
      double deg = ticks2deg(id, pos);

      serial_printf("STATUS %s (id=%2u): pos=%4d deg=%.1f current=%dmA temp=%d°C\n",
                    s->get_key(), sid, pos, deg, curr, temp);
    }
  }
  if (id > 0) return;

  // ---- xy metrics ----
  int a1_ticks = dxl.getPresentPosition(ID_ARM1);
  int a2_ticks = dxl.getPresentPosition(ID_ARM2);

  if (dxl.ping(ID_ARM1) && dxl.ping(ID_ARM2)) {
    kin.setA1ticks(a1_ticks);
    kin.setA2ticks(a2_ticks);
    serial_printf("STATUS XY X=%.2fmm Y=%.2fmm A1=%.2fdeg A2=%.2fdeg\n", kin.getXmm(), kin.getYmm(), kin.getA1deg(), kin.getA2deg());
  } else {
    serial_printf("STATUS XY X=na Y=na A1=na A2=na");
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
bool cmdMoveYSyncSmooth(int relTicks) {
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2) || !dxl.ping(ID_WRIST)) return false;

  // --- Read starting positions ---
  int start1 = dxl.getPresentPosition(ID_ARM1);
  int start2 = dxl.getPresentPosition(ID_ARM2);
  int startG = dxl.getPresentPosition(ID_WRIST);

  // directions: arm1 (+), arm2 (-), gripper (+)
  int goal1 = constrain(start1 + relTicks, 0, 4095);
  int goal2 = constrain(start2 - relTicks, 0, 4095);
  int goalG = constrain(startG + relTicks, 0, 4095);

  const int totalTicks = abs(relTicks);
  if (totalTicks < 3) return true;

  // --- Servo config ---
  torqueOnGroup({ ID_ARM1, ID_ARM2, ID_WRIST });
  ledOnGroup({ ID_ARM1, ID_ARM2, ID_WRIST });
  adjustPwmGroup({ ID_ARM1, ID_ARM2, ID_WRIST }, 880);

  // --- Motion profile ---
  const int stepInterval_ms = 15;
  const int accelSteps = 20;
  const int decelSteps = 20;
  const int minStep = 1;
  const int maxStep = 25;
  const int syncTol = 4;  // allowable tick error between axes

  // Compute acceleration, coast, decel spans
  int accelSpan = accelSteps * (minStep + maxStep) / 2;
  int decelSpan = decelSteps * (minStep + maxStep) / 2;
  int coastSpan = totalTicks - (accelSpan + decelSpan);
  if (coastSpan < 0) coastSpan = 0;
  int coastTicks = coastSpan / maxStep;

  // --- Working positions ---
  int pos1 = start1;
  int pos2 = start2;
  int posG = startG;

  // --- Function to apply micro sync correction ---
  auto syncNudge = [&](int &p1, int &p2, int &pg) {
    int e21 = (dxl.getPresentPosition(ID_ARM2) - p2) - (dxl.getPresentPosition(ID_ARM1) - p1);
    int eG2 = (dxl.getPresentPosition(ID_WRIST) - pg) - (dxl.getPresentPosition(ID_ARM2) - p2);

    if (abs(e21) > syncTol) {
      int nudge = constrain(e21 / 2, -3, 3);
      dxl.setGoalPosition(ID_ARM2, dxl.getPresentPosition(ID_ARM2) - nudge);
    }
    if (abs(eG2) > syncTol) {
      int nudge = constrain(eG2 / 2, -3, 3);
      dxl.setGoalPosition(ID_WRIST, dxl.getPresentPosition(ID_WRIST) - nudge);
    }
  };

  // --- Acceleration phase ---
  for (int i = 0; i < accelSteps; i++) {
    int step = map(i, 0, accelSteps - 1, minStep, maxStep);
    pos1 += (relTicks > 0 ? +step : -step);
    pos2 -= (relTicks > 0 ? +step : -step);
    posG += (relTicks > 0 ? +step : -step);

    dxl.setGoalPosition(ID_ARM1, pos1);
    dxl.setGoalPosition(ID_ARM2, pos2);
    dxl.setGoalPosition(ID_WRIST, posG);
    delay(stepInterval_ms);
    syncNudge(pos1, pos2, posG);
  }

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_WRIST })) return false;

  // --- Coast phase ---
  for (int i = 0; i < coastTicks; i++) {
    pos1 += (relTicks > 0 ? +maxStep : -maxStep);
    pos2 -= (relTicks > 0 ? +maxStep : -maxStep);
    posG += (relTicks > 0 ? +maxStep : -maxStep);

    dxl.setGoalPosition(ID_ARM1, pos1);
    dxl.setGoalPosition(ID_ARM2, pos2);
    dxl.setGoalPosition(ID_WRIST, posG);
    delay(stepInterval_ms);
    syncNudge(pos1, pos2, posG);
  }

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_WRIST })) return false;

  // --- Deceleration phase ---
  for (int i = decelSteps - 1; i >= 0; i--) {
    int step = map(i, 0, decelSteps - 1, minStep, maxStep);
    pos1 += (relTicks > 0 ? +step : -step);
    pos2 -= (relTicks > 0 ? +step : -step);
    posG += (relTicks > 0 ? +step : -step);

    if (relTicks > 0) {
      if (pos1 > goal1) pos1 = goal1;
      if (pos2 < goal2) pos2 = goal2;
      if (posG > goalG) posG = goalG;
    } else {
      if (pos1 < goal1) pos1 = goal1;
      if (pos2 > goal2) pos2 = goal2;
      if (posG < goalG) posG = goalG;
    }

    dxl.setGoalPosition(ID_ARM1, pos1);
    dxl.setGoalPosition(ID_ARM2, pos2);
    dxl.setGoalPosition(ID_WRIST, posG);
    delay(stepInterval_ms);
    syncNudge(pos1, pos2, posG);
  }

  // --- Final correction phase ---
  delay(80);  // allow settle
  int p1f = dxl.getPresentPosition(ID_ARM1);
  int p2f = dxl.getPresentPosition(ID_ARM2);
  int pGf = dxl.getPresentPosition(ID_WRIST);

  int e1 = goal1 - p1f;
  int e2 = goal2 - p2f;
  int eG = goalG - pGf;

  const int tol = 4;
  for (int n = 0; n < 6; n++) {
    if (abs(e1) <= tol && abs(e2) <= tol && abs(eG) <= tol) break;

    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ID_ARM1, p1f + constrain(e1 / 3, -3, 3));
    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ID_ARM2, p2f + constrain(e2 / 3, -3, 3));
    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ID_WRIST, pGf + constrain(eG / 3, -3, 3));

    delay(30);
    p1f = dxl.getPresentPosition(ID_ARM1);
    p2f = dxl.getPresentPosition(ID_ARM2);
    pGf = dxl.getPresentPosition(ID_WRIST);
    e1 = goal1 - p1f;
    e2 = goal2 - p2f;
    eG = goalG - pGf;
  }

  // --- Gentle torque release ---
  adjustPwmGroup({ ID_ARM1, ID_ARM2, ID_WRIST }, 400);
  delay(30);
  ledOffGroup({ ID_ARM1, ID_ARM2, ID_WRIST });

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_WRIST })) return false;

  if (verboseOn) {
    serial_printf("MOVEY SYNC done | A1=%d(%d)  A2=%d(%d)  G=%d(%d)\n",
                  p1f, e1, p2f, e2, pGf, eG);
  }

  return true;
}

// -------------------------------------------------------------------
// Smooth synchronized lateral move (Arm1 + Arm2 + Gripper)
// Each axis can have its own small delta (±ticks)
// -------------------------------------------------------------------
bool cmdMoveXSyncSmooth(int delta1, int delta2, int deltaG) {
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2) || !dxl.ping(ID_WRIST)) return false;

  // --- Starting positions and goals ---
  int start1 = dxl.getPresentPosition(ID_ARM1);
  int start2 = dxl.getPresentPosition(ID_ARM2);
  int startG = dxl.getPresentPosition(ID_WRIST);

  int goal1 = constrain(start1 + delta1, 0, 4095);
  int goal2 = constrain(start2 + delta2, 0, 4095);
  int goalG = constrain(startG + deltaG, 0, 4095);

  int maxDiff = max(max(abs(delta1), abs(delta2)), abs(deltaG));
  if (maxDiff < 2) return true;

  // --- Basic setup ---
  torqueOnGroup({ ID_ARM1, ID_ARM2, ID_WRIST });
  ledOnGroup({ ID_ARM1, ID_ARM2, ID_WRIST });
  adjustPwmGroup({ ID_ARM1, ID_ARM2, ID_WRIST }, 880);

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
    int p3 = dxl.getPresentPosition(ID_WRIST);

    int e1 = g1 - p1;
    int e2 = g2 - p2;
    int e3 = g3 - p3;

    if (abs(e1) > syncTol)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ID_ARM1, p1 + constrain(e1 / 3, -3, 3));
    if (abs(e2) > syncTol)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ID_ARM2, p2 + constrain(e2 / 3, -3, 3));
    if (abs(e3) > syncTol)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ID_WRIST, p3 + constrain(e3 / 3, -3, 3));
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
    dxl.setGoalPosition(ID_WRIST, posG);
    delay(stepInterval_ms);
    selectiveNudge(goal1, goal2, goalG);
  }

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_WRIST })) return false;

  // --- Coast phase ---
  for (int i = 0; i < coastTicks; i++) {
    pos1 += scaledStep(delta1, maxStep);
    pos2 += scaledStep(delta2, maxStep);
    posG += scaledStep(deltaG, maxStep);
    dxl.setGoalPosition(ID_ARM1, pos1);
    dxl.setGoalPosition(ID_ARM2, pos2);
    dxl.setGoalPosition(ID_WRIST, posG);
    delay(stepInterval_ms);
    selectiveNudge(goal1, goal2, goalG);
  }

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_WRIST })) return false;

  // --- Deceleration phase ---
  for (int i = decelSteps - 1; i >= 0; i--) {
    int step = map(i, 0, decelSteps - 1, minStep, maxStep);
    pos1 += scaledStep(delta1, step);
    pos2 += scaledStep(delta2, step);
    posG += scaledStep(deltaG, step);
    dxl.setGoalPosition(ID_ARM1, pos1);
    dxl.setGoalPosition(ID_ARM2, pos2);
    dxl.setGoalPosition(ID_WRIST, posG);
    delay(stepInterval_ms);
    selectiveNudge(goal1, goal2, goalG);
  }

  // --- Final correction phase ---
  delay(60);
  int p1f = dxl.getPresentPosition(ID_ARM1);
  int p2f = dxl.getPresentPosition(ID_ARM2);
  int p3f = dxl.getPresentPosition(ID_WRIST);
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
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ID_WRIST, p3f + constrain(e3 / 3, -3, 3));
    delay(30);
    p1f = dxl.getPresentPosition(ID_ARM1);
    p2f = dxl.getPresentPosition(ID_ARM2);
    p3f = dxl.getPresentPosition(ID_WRIST);
    e1 = goal1 - p1f;
    e2 = goal2 - p2f;
    e3 = goalG - p3f;
  }

  // --- Gentle torque release ---
  adjustPwmGroup({ ID_ARM1, ID_ARM2, ID_WRIST }, 400);
  delay(30);
  ledOffGroup({ ID_ARM1, ID_ARM2, ID_WRIST });

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_WRIST })) return false;

  if (verboseOn)
    serial_printf("MOVEX SYNC done | A1=%d(%d) A2=%d(%d) G=%d(%d)\n",
                  p1f, e1, p2f, e2, p3f, e3);

  return true;
}

// -------------------------------------------------------------------
// MOVE TO ABSOLUTE Y (mm) USING KINEMATICS + SMOOTH SYNC MOVE
// -------------------------------------------------------------------
bool cmdMoveYmm(double y_mm) {

  // Clamp Y to mechanical limits
  if (y_mm < 0) y_mm = 0;
  if (y_mm > 2 * arm_length_mm) y_mm = 2 * arm_length_mm;

  // Read current position
  int currA1 = dxl.getPresentPosition(ID_ARM1);
  kin.setA1ticks(currA1);

  // Compute desired tick deltas from target Y
  double currY = kin.getYmm();
  double deltaY = y_mm - currY;
  kin.setXYmm(0, y_mm);
  int goalA1 = kin.getA1ticks();
  int goalA2 = kin.getA2ticks();
  int goalG = kin.getGripperTicks();

  int deltaA1 = goalA1 - currA1;
  int currA2 = dxl.getPresentPosition(ID_ARM2);
  int deltaA2 = goalA2 - currA2;
  int currG = dxl.getPresentPosition(ID_WRIST);
  int deltaG = goalG - currG;

  // Verbose summary
  if (verboseOn) {
    serial_printf("MOVEY START mm=%.2f  currY=%.2f ΔY=%.2f\n", y_mm, currY, deltaY);
    serial_printf("Δticks A1=%d  A2=%d  G=%d\n", deltaA1, deltaA2, deltaG);
  }

  // Align to vertical first (ensure arms in sync)
  cmdMoveXSyncSmooth(0, 0, 0);

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_WRIST })) return false;

  // Perform vertical synchronized move
  cmdMoveYSyncSmooth(deltaA1);

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_WRIST })) return false;

  return true;
}

// -------------------------------------------------------------------
// MOVE TO RELATIVE X (mm OFFSET FROM VERTICAL) USING KINEMATICS
// -------------------------------------------------------------------
bool cmdMoveXmm(double x_mm) {

  // Read current geometry from servos
  int currA1 = dxl.getPresentPosition(ID_ARM1);
  int currA2 = dxl.getPresentPosition(ID_ARM2);
  int currG = dxl.getPresentPosition(ID_WRIST);
  kin.setA1ticks(currA1);
  kin.setA2ticks(currA2);

  // Compute current X and target X
  double currX = kin.getXmm();
  double targetX = currX + x_mm;
  kin.setXYmm(targetX, -1);  // ✅ move horizontally, preserve current Y
  int goalA1 = kin.getA1ticks();
  int goalA2 = kin.getA2ticks();
  int goalG = kin.getGripperTicks();

  int deltaA1 = goalA1 - currA1;
  int deltaA2 = goalA2 - currA2;
  int deltaG = goalG - currG;

  if (verboseOn) {
    serial_printf("MOVEX START mm=%.2f  currX=%.2f targetX=%.2f\n", x_mm, currX, targetX);
    serial_printf("Δticks A1=%d  A2=%d  G=%d\n", deltaA1, deltaA2, deltaG);
  }

  // Execute the small lateral coordinated motion
  cmdMoveXSyncSmooth(deltaA1, deltaA2, deltaG);

  if (checkStallGroup({ ID_ARM1, ID_ARM2, ID_WRIST })) return false;

  return true;
}

// -------------------------------------------------------------------
// TESTMOVEY <y1_mm> <y2_mm> – geometric Y-axis test (kinematics-based)
// -------------------------------------------------------------------
void cmdTestMoveY(float yStart_mm, float yEnd_mm, int steps = 10) {

  Serial.println("=== TESTMOVEY (vertical geometry) ===");
  serial_printf("Arm length=%.1fmm\n", arm_length_mm);

  // Compute tick start and end from kinematics
  kin.setXYmm(0, yStart_mm);
  int ticksStart1 = kin.getA1ticks();
  int ticksStart2 = kin.getA2ticks();

  kin.setXYmm(0, yEnd_mm);
  int ticksEnd1 = kin.getA1ticks();
  int ticksEnd2 = kin.getA2ticks();

  serial_printf("Ystart=%.2fmm  Yend=%.2fmm  ΔY=%.2fmm\n",
                yStart_mm, yEnd_mm, (yEnd_mm - yStart_mm));

  // bring both arms to start
  cmdMoveSmooth(ID_ARM1, ticksStart1);
  cmdMoveSmooth(ID_ARM2, ticksStart2);
  delay(300);

  int pos1 = dxl.getPresentPosition(ID_ARM1);
  int pos2 = dxl.getPresentPosition(ID_ARM2);
  kin.setA1ticks(pos1);
  kin.setA2ticks(pos2);

  float x0 = kin.getXmm();
  float y0 = kin.getYmm();
  serial_printf("Start measured: A1=%d A2=%d  X=%.2f  Y=%.2f\n", pos1, pos2, x0, y0);

  // move incrementally between start and end
  for (int i = 1; i <= steps; i++) {
    float y_step = yStart_mm + (yEnd_mm - yStart_mm) * (float)i / (float)steps;
    kin.setXYmm(0, y_step);
    int goal1 = kin.getA1ticks();
    int goal2 = kin.getA2ticks();

    cmdMoveSmooth(ID_ARM1, goal1);
    cmdMoveSmooth(ID_ARM2, goal2);

    int a1 = dxl.getPresentPosition(ID_ARM1);
    int a2 = dxl.getPresentPosition(ID_ARM2);
    kin.setA1ticks(a1);
    kin.setA2ticks(a2);

    float x_mm = kin.getXmm();
    float y_mm = kin.getYmm();
    float x_err = x_mm - x0;
    serial_printf("Step %2d | A1=%4d A2=%4d | X=%.3fmm  Y=%.3fmm  Xerr=%.3fmm\n",
                  i, a1, a2, x_mm, y_mm, x_err);
  }

  int f1 = dxl.getPresentPosition(ID_ARM1);
  int f2 = dxl.getPresentPosition(ID_ARM2);
  kin.setA1ticks(f1);
  kin.setA2ticks(f2);
  float xF = kin.getXmm();
  float yF = kin.getYmm();

  serial_printf("End measured: A1=%d A2=%d  X=%.3f  Y=%.3f  ΔY=%.3f  X drift=%.3f\n",
                f1, f2, xF, yF, yF - y0, xF - x0);
  Serial.println("=== end test ===\n");
}

// -------------------------------------------------------------------
//                          SETUP
// -------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  dxl.begin(57600);
  dxl.setPortProtocolVersion(PROTOCOL);

  Serial.println("---- OpenRB Arm Motion Controller v13 (Adaptive + Verbose + GripperComp) ----");
  for (auto *s : all_servos) {
    uint8_t id = s->get_id();
    if (dxl.ping(id)) {
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_POSITION);
      dxl.torqueOn(id);
      serial_printf("Servo %s (id=%u) OK\n", s->get_key(), id);
    }
  }

  Serial.println("\nDynamixel xl430 controller v13");
  Serial.println("Supported Commands (VERBOSE ON):");
  Serial.println("  VERBOSEON / VERBOSEOFF         - toggle console verbosity");
  Serial.println("  SETLIMIT <id> <min> <max>      - set soft position limits");
  Serial.println("  MOVE  <id> <absgoal|±rel>      - adaptive move one servo");
  Serial.println("  MOVEDEG  <id> <deg>            - adaptive move one servo");
  Serial.println("  MOVEY <float mm>               - vertical (mirror arm1/arm2)");
  Serial.println("  MOVEX <float mm>               - lateral (keep y)");
  Serial.println("  MOVECENTER                     - move all to 2048 ticks");
  Serial.println("  MOVEXDEG <int ang>             - vertical (1 relative angle arm1/arm2)");
  Serial.println("  MOVEYDEG <ang1> <ang2> <angw>  - lateral (3 rel angles for 1,2,w)");
  Serial.println("  READ <id> | 0 for all          - servos basics");
  Serial.println("  TESTMOVE  <id> [cycles]        - adaptive single-servo test");
  Serial.println("  TESTMOVEX <ticks>              - lateral X-axis test");
  Serial.println("  TESTMOVEY <y1_mm> <y2_mm>      - vertical Y-axis geometry test");
  Serial.println("  INFO  <id>                     - show servo mode/limits/profiles");
  Serial.println("  LEDON <id> / LEDOFF <id>");
  Serial.println("  (Boot default = VERBOSE ON)");
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
    bool hasError = false;
    for (int i = 0; i < SERVO_COUNT; i++) {
      auto *s = all_servos[i];
      if (!cmdMoveSmooth(s->get_id(), 2048)) {
        serial_printf("Error moving %s (id=%u)\n", s->get_key(), s->get_id());
        hasError = true;
      }
    }
  }

  // -------------- MOVE Y AXIS --------------
  else if (U.startsWith("MOVEYDEG")) {
    int val = 0;
    if (sscanf(line.c_str(), "MOVEYDEG %d", &val) == 1) {
      // Perform vertical synchronized move
      if (!cmdMoveYSyncSmooth(val)) Serial.println("MOVEYDEG ERR");
      else Serial.println("MOVEYDEG OK");
      print_status(0);
    } else if (verboseOn) Serial.println("Usage: MOVEYDEG <ticks>");
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
    double val = 0;
    if (sscanf(line.c_str(), "MOVEY %lf", &val) == 1) {
      if (!cmdMoveYmm(val)) Serial.println("MOVEY ERR");
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

  // -------------- TEST MOVE Y --------------
  else if (verboseOn && U.startsWith("TESTMOVEY")) {
    float y1 = 0, y2 = 0;
    if (sscanf(line.c_str(), "TESTMOVEY %f %f", &y1, &y2) == 2)
      cmdTestMoveY(y1, y2);
    else if (verboseOn)
      Serial.println("Usage: TESTMOVEY <yStart_mm> <yEnd_mm>");
  }

  // -------------- TEST MOVE X --------------
  else if (verboseOn && U.startsWith("TESTMOVEX")) {
    int v = line.substring(9).toInt();
    cmdTestMoveX(v);
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
