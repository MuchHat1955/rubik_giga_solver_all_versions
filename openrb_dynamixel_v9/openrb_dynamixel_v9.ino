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
void print_status();
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
    serial_printf("STALL ID %d curr=%d temp=%d\n", id, curr, temp);
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
//                     MOTION CONTROL CORE
// -------------------------------------------------------------------

void moveServoSmart(uint8_t id, int rel_ticks) {
  if (!dxl.ping(id) || rel_ticks == 0) return;

  int start = dxl.getPresentPosition(id);
  int goal = start + rel_ticks;
  int absMove = abs(rel_ticks);
  bool smallMove = (absMove <= SMALL_MOVE_TICKS);

  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id, 885);
  dxl.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, id, 1);
  dxl.writeControlTableItem(ControlTableItem::DRIVE_MODE, id, 0);
  if (!dxl.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id)) dxl.torqueOn(id);
  lOn(id);

  if (abs(goal - start) > FINE_WINDOW_TICKS) {
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, COARSE_PV);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, COARSE_PA);
    dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, COARSE_PG);
    dxl.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, id, COARSE_DG);
  } else {
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, FINE_PV);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, FINE_PA);
    dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, FINE_PG);
    dxl.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, id, FINE_DG);
    dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, id, FINE_IG);
  }

  if (smallMove) {
    int over = (rel_ticks > 0 ? SMALL_OVER_UP_TICKS : -SMALL_OVER_DOWN_TICKS);
    dxl.setGoalPosition(id, goal + over);
    delay(abs(over) * OVER_DELAY_MS_PER_TICK);
  }

  dxl.setGoalPosition(id, goal);
  uint32_t maxTime = estimateTravelTimeMs(id, goal - start);
  uint32_t t0 = millis();

  while (millis() - t0 < maxTime) {
    if (checkStall(id)) return;
    if (!isMoving(id) && isInPosition(id)) break;
    delay(10);
  }

  // Micro-jog finisher
  int pos = dxl.getPresentPosition(id);
  int diff = goal - pos;
  uint32_t t_trim = millis();
  while (millis() - t_trim < MICROJOG_MAX_MS) {
    if (abs(diff) <= FINISH_TOL_TICKS) break;
    int nudge = (diff > 0 ? 1 : -1);
    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, pos + nudge);
    delay(MICROJOG_SAMPLE_DELAY);
    pos = dxl.getPresentPosition(id);
    diff = goal - pos;
  }

  delay(20);
  pos = dxl.getPresentPosition(id);
  int err = pos - goal;
  lOff(id);

  if (verboseOn)
    serial_printf("MOVE END id=%d start=%d goal=%d final=%d err=%d\n",
                  id, start, goal, pos, err);
}

// -------------------------------------------------------------------
//   GRIPPER COMPENSATION (maintain relative angle to vertical)
// -------------------------------------------------------------------

void updateGripperKeepBaseRef(int gripper_start) {
  if (!has_zero_arm1 || !has_zero_arm2 || !has_zero_gripper) return;

  // read current arm positions
  int a1_ticks = dxl.getPresentPosition(ID_ARM1);
  int a2_ticks = dxl.getPresentPosition(ID_ARM2);

  // update kinematics with live arm positions
  kin.setA1ticks(a1_ticks);
  kin.setA2ticks(a2_ticks);

  // compute new ideal gripper ticks for perfect vertical
  int ideal_vert_ticks = kin.getGripperTicks();

  // compute the starting offset (difference from vertical)
  // this preserves any tilt the gripper had when the move started
  int offset = gripper_start - ideal_vert_ticks;

  // apply offset to keep same relative angle
  int goal_ticks = ideal_vert_ticks + offset;
  goal_ticks = constrain(goal_ticks, 0, 4095);

  // send command to gripper
  dxl.setGoalPosition(ID_WRIST, goal_ticks);

  if (verboseOn) {
    serial_printf(
      "GRIPPER keep-base: A1=%d A2=%d | vert=%d start=%d offset=%d -> goal=%d (%.2f°)\n",
      a1_ticks, a2_ticks,
      ideal_vert_ticks, gripper_start, offset, goal_ticks, kin.getGripperAng());
  }
}

// -------------------------------------------------------------------
//                   ADAPTIVE MOVE - single servo
// -------------------------------------------------------------------

void cmdMoveAdaptive(uint8_t id, int goal, int final_goal, uint8_t depth = 0) {
  if (!dxl.ping(id)) return;
  if (depth > 2) return;

  int start = dxl.getPresentPosition(id);
  int diff = final_goal - start;
  if (abs(diff) <= 3) return;
  dxl.torqueOn(id);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id, 1023);
  lOn(id);

  dxl.setGoalPosition(id, goal);
  uint32_t maxTime = estimateTravelTimeMs(id, final_goal - start);
  uint32_t t0 = millis();
  while (isMoving(id) && (millis() - t0 < maxTime)) delay(10);

  int pos = dxl.getPresentPosition(id);
  diff = final_goal - pos;
  if (abs(diff) > 5) {
    int trim = (diff > 0 ? 1 : -1);
    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, goal + trim);
    delay(15);
    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, goal);
  }

  delay(20);
  pos = dxl.getPresentPosition(id);
  diff = final_goal - pos;
  lOff(id);

  if (verboseOn) {
    for (int i = 0; i < depth; i++) Serial.print(" ");
    serial_printf("MOVE END id=%d start=%d goal=%d final=%d err=%d (ADAPTIVE d=%d)\n",
                  id, start, goal, pos, diff, depth);
  }

  if (abs(diff) > 6 && depth < 2) {
    int subGoal = goal + (diff > 0 ? 10 : -10);
    subGoal = constrain(subGoal, 0, 4095);
    cmdMoveAdaptive(id, subGoal, goal, depth + 1);
    cmdMoveAdaptive(id, goal, goal, depth + 1);
  }
}

// -------------------------------------------------------------------
//                   ADAPTIVE MOVE - arm1, arm2 and wrist
// -------------------------------------------------------------------
void cmdMoveAdaptiveSync(int id1, int goal1,
                         int id2, int goal2,
                         int idGrip, int grip_start,
                         uint8_t depth_in) {
  if (!dxl.ping(id1) || !dxl.ping(id2) || !dxl.ping(idGrip)) return;
  if (depth_in > 2) return;

  // --- Enable torque & LED feedback ---
  dxl.torqueOn(id1);
  dxl.torqueOn(id2);
  dxl.torqueOn(idGrip);
  lOn(id1);
  lOn(id2);
  lOn(idGrip);

  // --- Setup PWM limit and motion profiles ---
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id1, 1023);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id2, 1023);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, idGrip, 1023);

  int start1 = dxl.getPresentPosition(id1);
  int start2 = dxl.getPresentPosition(id2);

  int delta1 = goal1 - start1;
  int delta2 = goal2 - start2;
  int maxDelta = max(abs(delta1), abs(delta2));
  if (maxDelta < 3) return;

  // choose motion profile (same as cmdMoveAdaptive)
  int pv = COARSE_PV, pa = COARSE_PA, pg = COARSE_PG, dg = COARSE_DG, ig = FINE_IG;
  if (maxDelta <= FINE_WINDOW_TICKS) {
    pv = FINE_PV;
    pa = FINE_PA;
    pg = FINE_PG;
    dg = FINE_DG;
    ig = FINE_IG;
  }

  // apply identical profile to both arms
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id1, pv);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id1, pa);
  dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id1, pg);
  dxl.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, id1, dg);
  dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, id1, ig);

  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id2, pv);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id2, pa);
  dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id2, pg);
  dxl.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, id2, dg);
  dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, id2, ig);

  // --- Issue coordinated goals ---
  dxl.setGoalPosition(id1, goal1);
  dxl.setGoalPosition(id2, goal2);

  uint32_t maxTime = max(estimateTravelTimeMs(id1, delta1),
                         estimateTravelTimeMs(id2, delta2));
  uint32_t t0 = millis();

  // --- Motion loop: both arms move, gripper tracked live ---
  while (millis() - t0 < maxTime) {
    if (checkStall(id1) || checkStall(id2)) break;

    updateGripperKeepBaseRef(grip_start);  // maintain verticality

    bool moving1 = isMoving(id1);
    bool moving2 = isMoving(id2);
    if (!moving1 && !moving2) break;
    delay(10);
  }

  // --- Fine micro-jog correction ---
  int pos1 = dxl.getPresentPosition(id1);
  int pos2 = dxl.getPresentPosition(id2);
  int diff1 = goal1 - pos1;
  int diff2 = goal2 - pos2;
  uint32_t t_trim = millis();

  while (millis() - t_trim < MICROJOG_MAX_MS) {
    if (abs(diff1) <= FINISH_TOL_TICKS && abs(diff2) <= FINISH_TOL_TICKS) break;

    if (abs(diff1) > FINISH_TOL_TICKS)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id1,
                                pos1 + (diff1 > 0 ? 1 : -1));
    if (abs(diff2) > FINISH_TOL_TICKS)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id2,
                                pos2 + (diff2 > 0 ? 1 : -1));

    delay(MICROJOG_SAMPLE_DELAY);
    pos1 = dxl.getPresentPosition(id1);
    pos2 = dxl.getPresentPosition(id2);
    diff1 = goal1 - pos1;
    diff2 = goal2 - pos2;
  }

  delay(20);
  pos1 = dxl.getPresentPosition(id1);
  pos2 = dxl.getPresentPosition(id2);
  lOff(id1);
  lOff(id2);
  lOff(idGrip);

  if (verboseOn) {
    for (int i = 0; i < depth_in; i++) Serial.print(" ");
    serial_printf("SYNC MOVE END d=%d | A1:%d→%d (Δ=%d)  A2:%d→%d (Δ=%d)\n",
                  depth_in, start1, pos1, pos1 - start1, start2, pos2, pos2 - start2);
  }

  // --- Adaptive recursive finish (like cmdMoveAdaptive) ---
  int diffMax = max(abs(goal1 - pos1), abs(goal2 - pos2));
  if (diffMax > 6 && depth_in < 2) {
    int subGoal1 = goal1 + (goal1 - pos1 > 0 ? 10 : -10);
    int subGoal2 = goal2 + (goal2 - pos2 > 0 ? 10 : -10);
    subGoal1 = constrain(subGoal1, 0, 4095);
    subGoal2 = constrain(subGoal2, 0, 4095);

    cmdMoveAdaptiveSync(id1, subGoal1, id2, subGoal2, idGrip, grip_start, depth_in + 1);
    cmdMoveAdaptiveSync(id1, goal1, id2, goal2, idGrip, grip_start, depth_in + 1);
  }
}

// -------------------------------------------------------------------
//                   MOVEY / MOVEX (with gripper) using kinematics
// -------------------------------------------------------------------

void print_xy() {
  serial_printf("READXY X=%.2fmm Y=%.2fmm\n", kin.getXmm(), kin.getYmm());
  serial_printf("READXY A1=%.2f° A2=%.2f°\n", kin.getA1deg(), kin.getA2deg());
  serial_printf("READXY ticks A1=%4d  A2=%4d\n", kin.getA1ticks(), kin.getA2ticks());
  serial_printf("GRIPPER ticks=%4d  angle=%.2f°\n",
                kin.getGripperTicks(), kin.getGripperAng());
}

void cmdMoveY(double y_mm) {
  // --- validation ---
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2) || !dxl.ping(ID_WRIST)) {
    if (!dxl.ping(ID_ARM1)) Serial.println("ERR: servo 11 (arm1) not connected");
    if (!dxl.ping(ID_ARM2)) Serial.println("ERR: servo 12 (arm2) not connected");
    if (!dxl.ping(ID_WRIST)) Serial.println("ERR: servo 13 (gripper) not connected");
    return;
  }
  if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2 || !has_zero_gripper) {
    if (!has_arm_len) Serial.println("ERR: SETLEN missing");
    if (!has_zero_arm1) Serial.println("ERR: SETZERO 11 (arm1) missing");
    if (!has_zero_arm2) Serial.println("ERR: SETZERO 12 (arm2) missing");
    if (!has_zero_gripper) Serial.println("ERR: SETZERO 13 (gripper) missing");
    return;
  }

  // --- compute new arm angles and ticks from desired Y ---
  kin.setYmm(y_mm);
  int goal1 = kin.getA1ticks();
  int goal2 = kin.getA2ticks();

  if (verboseOn)
    serial_printf("MOVEY y=%.2fmm -> A1=%.2f° A2=%.2f° | ticks1=%d ticks2=%d\n",
                  y_mm, kin.getA1deg(), kin.getA2deg(), goal1, goal2);

  // --- gripper compensation setup ---
  int gripper_start = dxl.getPresentPosition(ID_WRIST);

  // --- coordinated move ---
  cmdMoveAdaptiveSync(ID_ARM1, goal1, ID_ARM2, goal2, ID_WRIST, gripper_start, 0);
}


void cmdMoveX(double x_mm) {
  // --- validation ---
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2) || !dxl.ping(ID_WRIST)) {
    if (!dxl.ping(ID_ARM1)) Serial.println("ERR: servo 11 (arm1) not connected");
    if (!dxl.ping(ID_ARM2)) Serial.println("ERR: servo 12 (arm1) not connected");
    if (!dxl.ping(ID_WRIST)) Serial.println("ERR: servo 13 (gripper) not connected");
    return;
  }
  if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2 || !has_zero_gripper) {
    if (!has_arm_len) Serial.println("ERR: SETLEN missing");
    if (!has_zero_arm1) Serial.println("ERR: SETZERO 11 (arm1) missing");
    if (!has_zero_arm2) Serial.println("ERR: SETZERO 12 (arm2) missing");
    if (!has_zero_gripper) Serial.println("ERR: SETZERO 13 (gripper) missing");
    return;
  }

  // --- compute new arm angles/ticks for lateral X displacement ---
  kin.setXmm(x_mm);
  int goal1 = kin.getA1ticks();
  int goal2 = kin.getA2ticks();

  if (verboseOn)
    serial_printf("MOVEX x=%.2fmm -> A1=%.2f° A2=%.2f° | ticks1=%d ticks2=%d\n",
                  x_mm, kin.getA1deg(), kin.getA2deg(), goal1, goal2);

  int gripper_start = dxl.getPresentPosition(ID_WRIST);

  // --- coordinated move ---
  cmdMoveAdaptiveSync(ID_ARM1, goal1, ID_ARM2, goal2, ID_WRIST, gripper_start, 0);
}

// -------------------------------------------------------------------
//                        READXY using kinematics
// -------------------------------------------------------------------

void cmdReadXY() {
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2) || !dxl.ping(ID_WRIST)) {
    if (!dxl.ping(ID_ARM1)) Serial.println("ERR: servo 11 (arm1) not connected");
    if (!dxl.ping(ID_ARM2)) Serial.println("ERR: servo 12 (arm2) not connected");
    if (!dxl.ping(ID_WRIST)) Serial.println("ERR: servo 13 (gripper) not connected");
    return;
  }

  int a1_ticks = dxl.getPresentPosition(ID_ARM1);
  int a2_ticks = dxl.getPresentPosition(ID_ARM2);

  kin.setA1ticks(a1_ticks);
  kin.setA2ticks(a2_ticks);

  print_xy();
}

// -------------------------------------------------------------------
// READ: diagnostics table (position, current, temp)
// -------------------------------------------------------------------
void print_status() {
  Serial.println("ID | Position | Current(mA) | Temp(C)");
  Serial.println("--------------------------------------");
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    uint8_t id = servo_ids[i];
    if (!dxl.ping(id)) {
      serial_printf("%2d |   ---     |    ---     |  ---\n", id);
      continue;
    }
    int pos = dxl.getPresentPosition(id);
    int curr = dxl.getPresentCurrent(id);
    int temp = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, id);
    serial_printf("%2d | %5d     | %6d     | %3d\n", id, pos, curr, temp);
  }
  Serial.println("--------------------------------------");
}

void print_pos(int id) {
  if (!dxl.ping(id)) {
    serial_printf("READ %2d ERR", id);
    return;
  }
  int pos = dxl.getPresentPosition(id);
  serial_printf("READ %2d %5d", id, pos);
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
    cmdMoveAdaptive(id, STOW_TICK, STOW_TICK, 0);
    cmdMoveAdaptive(id, STOW_TICK, STOW_TICK, 0);
    int baseline = dxl.getPresentPosition(id);
    for (int i = 0; i < RAND_INCS_N; ++i) {
      int goalUp = baseline + RAND_INCS[i];
      int goalDn = baseline - RAND_INCS[i];
      cmdMoveAdaptive(id, goalUp, goalUp, 0);
      cmdMoveAdaptive(id, goalDn, goalDn, 0);
      int final = dxl.getPresentPosition(id);
      tst::addErr(stats, RAND_INCS[i], abs(final - goalDn));
    }
    // large excursion
    cmdMoveAdaptive(id, STOW_TICK + 1000, STOW_TICK + 1000, 0);
    cmdMoveAdaptive(id, STOW_TICK, STOW_TICK, 0);
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
  cmdMoveX(rel_ticks);
  int end1 = dxl.getPresentPosition(ID_ARM1);
  int end2 = dxl.getPresentPosition(ID_ARM2);
  serial_printf("TESTMOVEX Δticks=%d | Arm1:%d→%d  Arm2:%d→%d\n",
                rel_ticks, start1, end1, start2, end2);
  Serial.println("=================\n");
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
  cmdMoveAdaptive(ID_ARM1, ticksStart1, ticksStart1, 0);
  cmdMoveAdaptive(ID_ARM2, ticksStart2, ticksStart2, 0);
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

    cmdMoveAdaptive(ID_ARM1, goal1, goal1, 0);
    cmdMoveAdaptive(ID_ARM2, goal2, goal2, 0);
    updateGripperKeepBaseRef(dxl.getPresentPosition(ID_WRIST));

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

  Serial.println("=== OpenRB Arm Motion Controller v10 (Adaptive + Verbose + GripperComp) ===");
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    uint8_t id = servo_ids[i];
    if (dxl.ping(id)) {
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_POSITION);
      dxl.torqueOn(id);
      serial_printf("Servo %d OK\n", id);
    }
  }

  Serial.println("\nDynamixel xl430 controller v9");
  Serial.println("Supported Commands (VERBOSE ON):");
  Serial.println("  VERBOSEON / VERBOSEOFF         - toggle console verbosity");
  Serial.println("  SETLIMIT <id> <min> <max>      - set soft position limits");
  Serial.println("  SETLEN  <mm>                   - set arm length");
  Serial.println("  SETZERO <id> <ticks> [±1]      - set zero tick and dir");
  Serial.println("  MOVE  <id> <absgoal|±rel>      - adaptive move one servo");
  Serial.println("  MOVEY <float mm>               - vertical (mirror arm1/arm2)");
  Serial.println("  MOVEX <float mm>               - lateral (keep y)");
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
    serial_printf("Arm length set to %.1f mm\n", arm_length_mm);
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
      cmdMoveAdaptive((uint8_t)id, goal, goal, 0);
      print_pos(id);
    } else {
      Serial.println("Usage: MOVE <id> <absgoal|±rel>");
    }
  }

  // -------------- MOVE Y AXIS --------------
  else if (U.startsWith("MOVEY")) {
    double val = line.substring(6).toFloat();
    cmdMoveY(val);
  }

  // -------------- MOVE X AXIS --------------
  else if (U.startsWith("MOVEX")) {
    double val = line.substring(6).toFloat();
    cmdMoveX(val);
  }

  // -------------- READ XY (kinematic position) --------------
  else if (U.startsWith("READXY")) {
    cmdReadXY();
  }

  // -------------- READ STATUS (all servos) --------------
  else if (U.startsWith("READ")) {
    if (verboseOn)
      print_status();
    else {
      print_pos(ID_ARM1);
      print_pos(ID_ARM2);
      print_pos(ID_WRIST);
      print_pos(ID_GRIP1);
      print_pos(ID_GRIP2);
      print_pos(ID_BASE);
    }
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
  else if (U.startsWith("TESTMOVEY")) {
    float y1 = 0, y2 = 0;
    if (sscanf(line.c_str(), "TESTMOVEY %f %f", &y1, &y2) == 2)
      cmdTestMoveY(y1, y2);
    else if (verboseOn)
      Serial.println("Usage: TESTMOVEY <yStart_mm> <yEnd_mm>");
  }

  // -------------- TEST MOVE X --------------
  else if (U.startsWith("TESTMOVEX")) {
    int v = line.substring(9).toInt();
    cmdTestMoveX(v);
  }

  // -------------- TEST MOVE (single servo cycles) --------------
  else if (U.startsWith("TESTMOVE")) {
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
