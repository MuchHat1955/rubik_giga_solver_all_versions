#include <Dynamixel2Arduino.h>
#include <math.h>
#include <cmath>

// -------------------------------------------------------------------
//                   GLOBAL STRUCTS & HELPERS
// -------------------------------------------------------------------

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

// ---------------- Serial printf helper ----------------
template<typename... Args>
void serial_printf(const char *fmt, Args... args) {
  char buf[200];
  snprintf(buf, sizeof(buf), fmt, args...);
  Serial.print(buf);
}

// ---------------- Verbose mode ----------------
bool verboseOn = true;  // default at boot = ON

// -------------------------------------------------------------------
//                       SERVO CONFIG
// -------------------------------------------------------------------

#define DXL_SERIAL Serial1
#define DXL_DIR_PIN -1
#define PROTOCOL 2.0
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// Servo IDs (customize for your robot)
#define ID_ARM1 11
#define ID_ARM2 12
#define ID_WRIST 13
#define ID_GRIP1 14
#define ID_GRIP2 15
#define ID_BASE 16
uint8_t servo_ids[] = { ID_ARM1, ID_ARM2, ID_WRIST, ID_GRIP1, ID_GRIP2, ID_BASE };
const uint8_t SERVO_COUNT = sizeof(servo_ids) / sizeof(servo_ids[0]);

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
const float DEG_PER_TICK = 360.0f / TICKS_PER_REV;
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

float arm_length_mm = 0;
int tick_zero_arm1 = -1;
int tick_zero_arm2 = -1;
int tick_zero_gripper = -1;
bool has_arm_len = false;
bool has_zero_arm1 = false;
bool has_zero_arm2 = false;
bool has_zero_gripper = false;

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
//                   VERTICAL KINEMATICS CLASS
// -------------------------------------------------------------------

class VerticalKinematics {
public:
  VerticalKinematics(double arm_len_mm = 60.0)
    : l_mm(arm_len_mm) {
    tick0_arm1 = 2048;
    tick0_arm2 = 2048;
    tick0_grip = 2048;
    dir1 = +1.0;
    dir2 = +1.0;
    dirg = +1.0;
    a1_deg = 0;
    a2_deg = 90;
    update_from_angles();
  }

  // ---------------- Configuration ----------------
  void setArmLength(double mm) {
    l_mm = mm;
    update_from_angles();
  }

  void setTickZeroArm1(int t0) {
    tick0_arm1 = t0;
    update_from_angles();
  }
  void setTickZeroArm2(int t0) {
    tick0_arm2 = t0;
    update_from_angles();
  }
  void setTickZeroGripper(int t0) {
    tick0_grip = t0;
    update_from_angles();
  }

  void setDirArm1(double d) {
    dir1 = (d >= 0) ? +1.0 : -1.0;
    update_from_angles();
  }
  void setDirArm2(double d) {
    dir2 = (d >= 0) ? +1.0 : -1.0;
    update_from_angles();
  }
  void setDirGripper(double d) {
    dirg = (d >= 0) ? +1.0 : -1.0;
    update_from_angles();
  }

  // ---------------- Setters (servo positions) ----------------
  void setA1deg(double a1) {
    a1_deg = a1;
    a2_deg = 90.0 - 2.0 * a1;
    update_from_angles();
  }
  void setA2deg(double a2) {
    a2_deg = a2;
    a1_deg = (90.0 - a2) / 2.0;
    update_from_angles();
  }

  void setA1ticks(int ticks) {
    a1_deg = (ticks - tick0_arm1) * DEG_PER_TICK * dir1;
    a2_deg = 90.0 - 2.0 * a1_deg;
    update_from_angles();
  }

  void setA2ticks(int ticks) {
    a2_deg = (ticks - tick0_arm2) * DEG_PER_TICK * dir2;
    a1_deg = (90.0 - a2_deg) / 2.0;
    update_from_angles();
  }

  // ---------------- Setters (geometry positions) ----------------
  void setYmm(double y) {
    y_mm = y;
    double c = clamp(y_mm / (2.0 * l_mm), -1.0, 1.0);
    double beta_rad = 2.0 * std::acos(c);
    a1_deg = rad2deg(beta_rad / 2.0);
    a2_deg = 90.0 - 2.0 * a1_deg;
    x_mm = l_mm * std::sin(deg2rad(a1_deg));
  }

  void setXmm(double x) {
    a1_deg = rad2deg(std::asin(clamp(x / l_mm, -1.0, 1.0)));
    a2_deg = 90.0 - 2.0 * a1_deg;
    x_mm = x;
    y_mm = 2.0 * l_mm * std::cos(deg2rad(a1_deg));
  }

  // ---------------- Getters ----------------
  double getArmLength() const {
    return l_mm;
  }
  double getA1deg() const {
    return a1_deg;
  }
  double getA2deg() const {
    return a2_deg;
  }

  int getA1ticks() const {
    return static_cast<int>(std::round(tick0_arm1 + dir1 * a1_deg * TICKS_PER_DEG));
  }
  int getA2ticks() const {
    return static_cast<int>(std::round(tick0_arm2 + dir2 * a2_deg * TICKS_PER_DEG));
  }

  double getXmm() const {
    return x_mm;
  }
  double getYmm() const {
    return y_mm;
  }

  // ---------------- Gripper math ----------------
  double getGripperAng() const {
    // 0° = gripper vertical; positive = tilted with arm2
    return a1_deg + 90.0 - a2_deg;
  }

  int getGripperTicks() const {
    return static_cast<int>(std::round(tick0_grip + dirg * getGripperAng() * TICKS_PER_DEG));
  }

  // Record current gripper zero based on present arms geometry
  void setZeroGripper(int currentTicks) {
    tick0_grip = currentTicks - static_cast<int>(std::round(dirg * getGripperAng() * TICKS_PER_DEG));
  }

  int getTickZeroArm1() const {
    return tick0_arm1;
  }
  int getTickZeroArm2() const {
    return tick0_arm2;
  }
  int getTickZeroGripper() const {
    return tick0_grip;
  }

  double getDirArm1() const {
    return dir1;
  }
  double getDirArm2() const {
    return dir2;
  }
  double getDirGripper() const {
    return dirg;
  }

private:
  // constants
  static constexpr double TICKS_PER_DEG = 4096.0 / 360.0;
  static constexpr double DEG_PER_TICK = 360.0 / 4096.0;

  // state
  double l_mm;
  double a1_deg, a2_deg;
  double x_mm, y_mm;
  int tick0_arm1, tick0_arm2, tick0_grip;
  double dir1, dir2, dirg;

  // helpers
  static inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
  }
  static inline double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
  }

  static inline double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi
                                    : v;
  }

  void update_from_angles() {
    x_mm = l_mm * std::sin(deg2rad(a1_deg));
    y_mm = 2.0 * l_mm * std::cos(deg2rad(a1_deg));
  }
};

VerticalKinematics kin(60.0);

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
  const int nudgeDelay = 35;

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
      serial_printf("    err=%d nudge=%d\n", err, nudge);

    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, posNow + nudge);
    delay(nudgeDelay);

    posNow = dxl.getPresentPosition(id);
    err = goal - posNow;
    samePos = (prevPos == posNow);
    prevPos = posNow;
    count++;
  }

  delay(45);  // final settle
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
void print_status(int id) {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    uint8_t crrid = servo_ids[i];
    if (id <= 0) crrid = id;
    if (!dxl.ping(crrid)) {
      Serial.println("STATUS id=na pos=na current=na temp=na");
    } else {
      int pos = dxl.getPresentPosition(crrid);
      int curr = dxl.getPresentCurrent(crrid);
      int temp = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, id);
      serial_printf("STATUS id=%2d pos=%5d current=%6dmA temp=%3d\n", crrid, pos, curr, temp);
    }
    if (id <= 0) break;
  }
  int a1_ticks = dxl.getPresentPosition(ID_ARM1);
  int a2_ticks = dxl.getPresentPosition(ID_ARM2);

  if (dxl.ping(ID_ARM1) && dxl.ping(ID_ARM2)) {
    kin.setA1ticks(a1_ticks);
    kin.setA2ticks(a2_ticks);
    serial_printf("STATUS XY X=%.2fmm Y=%.2mm\n", kin.getXmm(), kin.getYmm());
  } else {
    Serial.println("STATUS XY X=na Y=na");
  }
}

void print_pos(int id) {
  if (!dxl.ping(id)) {
    double currY = kin.getYmm();
    double currX = kin.getXmm();
    serial_printf("READ ERR=%2d ERR", id);
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
  if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2) {
    Serial.println("ERR: not calibrated");
    return;
  }
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
  dxl.torqueOn(ID_ARM1);
  dxl.torqueOn(ID_ARM2);
  dxl.torqueOn(ID_WRIST);
  lOn(ID_ARM1);
  lOn(ID_ARM2);
  lOn(ID_WRIST);

  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, ID_ARM1, 880);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, ID_ARM2, 880);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, ID_WRIST, 880);

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

  if (checkStall(ID_ARM1)) return false;
  if (checkStall(ID_ARM2)) return false;
  if (checkStall(ID_WRIST)) return false;

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

  if (checkStall(ID_ARM1)) return false;
  if (checkStall(ID_ARM2)) return false;
  if (checkStall(ID_WRIST)) return false;

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
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, ID_ARM1, 400);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, ID_ARM2, 400);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, ID_WRIST, 400);
  delay(30);
  lOff(ID_ARM1);
  lOff(ID_ARM2);
  lOff(ID_WRIST);

  if (checkStall(ID_ARM1)) return false;
  if (checkStall(ID_ARM2)) return false;
  if (checkStall(ID_WRIST)) return false;

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

  // --- Basic setup ---
  dxl.torqueOn(ID_ARM1);
  dxl.torqueOn(ID_ARM2);
  dxl.torqueOn(ID_WRIST);
  lOn(ID_ARM1);
  lOn(ID_ARM2);
  lOn(ID_WRIST);

  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, ID_ARM1, 880);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, ID_ARM2, 880);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, ID_WRIST, 880);

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

  if (checkStall(ID_ARM1)) return false;
  if (checkStall(ID_ARM2)) return false;
  if (checkStall(ID_WRIST)) return false;

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

  if (checkStall(ID_ARM1)) return false;
  if (checkStall(ID_ARM2)) return false;
  if (checkStall(ID_WRIST)) return false;

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
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, ID_ARM1, 400);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, ID_ARM2, 400);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, ID_WRIST, 400);
  delay(30);
  lOff(ID_ARM1);
  lOff(ID_ARM2);
  lOff(ID_WRIST);

  if (checkStall(ID_ARM1)) return false;
  if (checkStall(ID_ARM2)) return false;
  if (checkStall(ID_WRIST)) return false;

  if (verboseOn)
    serial_printf("MOVEX SYNC done | A1=%d(%d) A2=%d(%d) G=%d(%d)\n",
                  p1f, e1, p2f, e2, p3f, e3);
}

// -------------------------------------------------------------------
// MOVE TO ABSOLUTE Y (mm) USING KINEMATICS + SMOOTH SYNC MOVE
// -------------------------------------------------------------------
bool cmdMoveYmm(double y_mm) {
  if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2 || !has_zero_gripper) {
    Serial.println("ERR: Missing calibration (SETLEN / SETZERO)");
    return false;
  }

  // Clamp Y to mechanical limits
  if (y_mm < 0) y_mm = 0;
  if (y_mm > 2 * kin.getArmLength()) y_mm = 2 * kin.getArmLength();

  // Read current position
  int currA1 = dxl.getPresentPosition(ID_ARM1);
  kin.setA1ticks(currA1);

  // Compute desired tick deltas from target Y
  double currY = kin.getYmm();
  double deltaY = y_mm - currY;
  kin.setYmm(y_mm);
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
    serial_printf("MOVEY START mm=%.2f  crrY=%.2f ΔY=%.2f\n", y_mm, currY, deltaY);
    serial_printf("Δticks A1=%d  A2=%d  G=%d\n", deltaA1, deltaA2, deltaG);
  }

  // Align to vertical first (ensure arms in sync)
  cmdMoveXSyncSmooth(0, 0, 0);

  if (checkStall(ID_ARM1)) return false;
  if (checkStall(ID_ARM2)) return false;
  if (checkStall(ID_WRIST)) return false;

  // Perform vertical synchronized move
  cmdMoveYSyncSmooth(deltaA1);

  if (checkStall(ID_ARM1)) return false;
  if (checkStall(ID_ARM2)) return false;
  if (checkStall(ID_WRIST)) return false;

  return true;
}

// -------------------------------------------------------------------
// MOVE TO RELATIVE X (mm OFFSET FROM VERTICAL) USING KINEMATICS
// -------------------------------------------------------------------
bool cmdMoveXmm(double x_mm) {
  if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2 || !has_zero_gripper) {
    Serial.println("ERR: Missing calibration (SETLEN / SETZERO)");
    return false;
  }

  // Read current geometry from servos
  int currA1 = dxl.getPresentPosition(ID_ARM1);
  int currA2 = dxl.getPresentPosition(ID_ARM2);
  int currG = dxl.getPresentPosition(ID_WRIST);
  kin.setA1ticks(currA1);
  kin.setA2ticks(currA2);

  // Compute current X and target X
  double currX = kin.getXmm();
  double targetX = currX + x_mm;
  kin.setXmm(targetX);

  int goalA1 = kin.getA1ticks();
  int goalA2 = kin.getA2ticks();
  int goalG = kin.getGripperTicks();

  int deltaA1 = goalA1 - currA1;
  int deltaA2 = goalA2 - currA2;
  int deltaG = goalG - currG;

  if (verboseOn) {
    serial_printf("MOVEX START mm=%.2f  crrX=%.2f targetX=%.2f\n", x_mm, currX, targetX);
    serial_printf("Δticks A1=%d  A2=%d  G=%d\n", deltaA1, deltaA2, deltaG);
  }

  // Execute the small lateral coordinated motion
  cmdMoveXSyncSmooth(deltaA1, deltaA2, deltaG);

  if (checkStall(ID_ARM1)) return false;
  if (checkStall(ID_ARM2)) return false;
  if (checkStall(ID_WRIST)) return false;

  return true;
}

// -------------------------------------------------------------------
// TESTMOVEY <y1_mm> <y2_mm> – geometric Y-axis test (kinematics-based)
// -------------------------------------------------------------------
void cmdTestMoveY(float yStart_mm, float yEnd_mm, int steps = 10) {
  if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2) {
    Serial.println("ERR: Calibration missing (SETLEN, SETZERO 11, SETZERO 12)");
    return;
  }

  Serial.println("=== TESTMOVEY (vertical geometry) ===");
  serial_printf("Arm length=%.1fmm\n", arm_length_mm);

  // Compute tick start and end from kinematics
  kin.setYmm(yStart_mm);
  int ticksStart1 = kin.getA1ticks();
  int ticksStart2 = kin.getA2ticks();

  kin.setYmm(yEnd_mm);
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
    kin.setYmm(y_step);
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

  Serial.println("---- OpenRB Arm Motion Controller v12 (Adaptive + Verbose + GripperComp) ----");
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    uint8_t id = servo_ids[i];
    if (dxl.ping(id)) {
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_POSITION);
      dxl.torqueOn(id);
      serial_printf("Servo %d OK\n", id);
    }
  }

  Serial.println("\nDynamixel xl430 controller v10");
  Serial.println("Supported Commands (VERBOSE ON):");
  Serial.println("  VERBOSEON / VERBOSEOFF         - toggle console verbosity");
  Serial.println("  SETLIMIT <id> <min> <max>      - set soft position limits");
  Serial.println("  SETLEN  <mm>                   - set arm length");
  Serial.println("  SETZERO <id> <ticks> [±1]      - set zero tick and dir");
  Serial.println("  MOVE  <id> <absgoal|±rel>      - adaptive move one servo");
  Serial.println("  MOVEY <float mm>               - vertical (mirror arm1/arm2)");
  Serial.println("  MOVEX <float mm>               - lateral (keep y)");
  Serial.println("  ANGX <int ang>                 - vertical (1 relative angle arm1/arm2)");
  Serial.println("  ANGY <ang1> <ang2> <angw>      - lateral (3 rel angles for 1,2,w)");
  Serial.println("  READ                           - diagnostics table");
  Serial.println("  READXY                         - compute current X/Y midpoint");
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

  // -------------- SET LIMITS --------------
  else if (U.startsWith("SETLIMIT")) {
    int id, minL, maxL;
    if (sscanf(line.c_str(), "SETLIMIT %d %d %d", &id, &minL, &maxL) == 3)
      cmdSetLimit(id, minL, maxL);
    else
      Serial.println("Usage: SETLIMIT <id> <min> <max>");
  }

  // -------------- SET ARM LENGTH --------------
  else if (U.startsWith("SETLEN")) {
    arm_length_mm = line.substring(6).toFloat();
    has_arm_len = arm_length_mm > 0;
    kin.setArmLength(arm_length_mm);
    serial_printf("Arm length set to %.1fmm\n", arm_length_mm);
  }

  // -------------- SET ZERO (with optional direction) --------------
  else if (U.startsWith("SETZERO")) {
    int id = 0, ticks = 0;
    int dir_i = 999;  // sentinel if not provided
    char dir_s[8] = { 0 };

    // Accept any of:
    //   SETZERO <id> <ticks>
    //   SETZERO <id> <ticks> <dir_int>  (e.g., -1 or +1)
    //   SETZERO <id> <ticks> <dir_str>  (e.g., "-1" or "+1")
    bool ok = false;
    if (sscanf(line.c_str(), "SETZERO %d %d %d", &id, &ticks, &dir_i) == 3) {
      ok = true;
    } else if (sscanf(line.c_str(), "SETZERO %d %d %7s", &id, &ticks, dir_s) == 3) {
      dir_i = (dir_s[0] == '-') ? -1 : +1;
      ok = true;
    } else if (sscanf(line.c_str(), "SETZERO %d %d", &id, &ticks) == 2) {
      dir_i = +1;  // default if direction omitted
      ok = true;
    }

    if (!ok) {
      Serial.println("Usage: SETZERO <id> <ticks> [±1]");
    } else {
      double dir = (dir_i < 0) ? -1.0 : +1.0;

      if (id == ID_ARM1) {  // ---- Arm1 zero ----
        tick_zero_arm1 = ticks;
        has_zero_arm1 = true;
        kin.setTickZeroArm1(tick_zero_arm1);
        kin.setDirArm1(dir);
        serial_printf("Zero tick for %d = %d  dir=%d\n", id, ticks, (dir_i < 0 ? -1 : +1));

      } else if (id == ID_ARM2) {  // ---- Arm2 zero ----
        tick_zero_arm2 = ticks;
        has_zero_arm2 = true;
        kin.setTickZeroArm2(tick_zero_arm2);
        kin.setDirArm2(dir);
        serial_printf("Zero tick for %d = %d  dir=%d\n", id, ticks, (dir_i < 0 ? -1 : +1));

      } else if (id == ID_WRIST) {  // ---- Gripper / wrist zero ----
        tick_zero_gripper = ticks;
        has_zero_gripper = true;
        kin.setTickZeroGripper(tick_zero_gripper);
        kin.setDirGripper(dir);
        serial_printf("Zero tick for %d = %d  dir=%d\n", id, ticks, (dir_i < 0 ? -1 : +1));

      } else {
        serial_printf("SETZERO: unknown id %d (use %d/%d/%d)\n",
                      id, ID_ARM1, ID_ARM2, ID_WRIST);
      }
    }
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

  // -------------- MOVE Y AXIS --------------
  else if (U.startsWith("ANGY")) {
    int val = 0;
    if (sscanf(line.c_str(), "ANGY %d", &val) == 1) {
      // Perform vertical synchronized move
      if (!cmdMoveYSyncSmooth(val)) Serial.println("ANGY ERR");
      else Serial.println("ANGY OK");
      print_status(0);
    } else if (verboseOn) Serial.println("Usage: ANGY <ticks>");
  }

  // -------------- ANGX (3 relative tick deltas: a1, a2, aw) --------------
  else if (U.startsWith("ANGX")) {
    int a1, a2, aw;
    if (sscanf(line.c_str(), "ANGX %d %d %d", &a1, &a2, &aw) == 3) {
      if (!cmdMoveXSyncSmooth(a1, a2, aw)) Serial.println("ANGX ERR");
      else Serial.println("ANGX OK");
      print_status(0);
    } else if (verboseOn) {
      Serial.println("Usage: ANGX <a1_ticks> <a2_ticks> <aw_ticks>");
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
    print_status(0);
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
