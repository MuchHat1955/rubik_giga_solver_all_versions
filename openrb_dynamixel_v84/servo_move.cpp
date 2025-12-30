
#include "servos.h"
#include "servo_move.h"
#include <map>
#include <algorithm>
#include <cmath>
#include "Dynamixel2Arduino.h"
#include "vertical_kinematics.h"
#include "ori.h"
#include "utils.h"
#include "log.h"

// ----------------------------------------------------------------------
// Externals from elsewhere in the project
// (These must be defined in other translation units.)
// ----------------------------------------------------------------------
extern Dynamixel2Arduino dxl;
extern VerticalKinematics kin;
extern double min_ymm;

double speed = 1.0;
double max_speed = 1.0;

double tol_g_per = 2.0;
double tol_w_deg = 1.0;
double tol_xmm = 0.5;
double tol_ymm = 0.5;

enum WristPos {
  WRIST_VERT,
  WRIST_HORIZ_LEFT,
  WRIST_HORIZ_RIGHT
};

// ----------------------------------------------------------------------
// NudgeController implementation
// ----------------------------------------------------------------------

NudgeController::NudgeController(uint8_t id_)
  : id(id_) {}

void NudgeController::recordData(int prevGoalTicks, int currPos, int nudge, MovePhase phase) {
  Record r;
  r.t_ms = millis();
  r.prevGoalTicks = prevGoalTicks;
  r.currPosTicks = currPos;
  r.errTicks = prevGoalTicks - currPos;
  r.nudgeApplied = nudge;
  r.phase = phase;

  if (records.size() >= maxRecords)
    records.erase(records.begin());

  records.push_back(r);
}

int NudgeController::computeNudge(int currErr, int dir, MovePhase phase, int samePosCount) {
  // Ignore tiny errors
  if (abs(currErr) <= 4) return 0;

  // In early phases, don't pull back if ahead of target
  if (currErr * dir < 0 && phase != MovePhase::FINAL) return 0;

  // Base estimate from simple proportional rule
  int nudge = baseEstimate(currErr, phase, samePosCount);

  // In final phase, add reinforcement if stuck
  if (phase == MovePhase::FINAL && samePosCount > 0) {
    nudge += (int)(samePosCount * 0.5 * (currErr > 0 ? 1 : -1));
  }

  nudge = (int)constrain(nudge, -20.0, 20.0);
  if (nudge > 0 && nudge < 6) nudge = 6;
  if (nudge < 0 && nudge > -6) nudge = -6;
  return nudge;
}

void NudgeController::printLog() {
  //DEBUG_INFO(MOD_SERVO_MOVE, "---- Nudge log for servo %d (count) ----",
  //    id, (int)records.size());

  for (auto& r : records) {
    const char* phaseStr =
      (r.phase == MovePhase::ACCEL) ? "ACC" : (r.phase == MovePhase::COAST) ? "COAST"
                                            : (r.phase == MovePhase::DECEL) ? "DEC"
                                                                            : "FINAL";

    //DEBUG_INFO(MOD_SERVO_MOVE, "[%lu ms] %s goal curr err nudge",
    //       r.t_ms, phaseStr, r.prevGoalTicks,
    //      r.currPosTicks, r.errTicks, r.nudgeApplied);
  }
}

int NudgeController::baseEstimate(int errTicks, MovePhase phase, int samePosCount) {
  double k = phaseGain(phase);
  double nudge = k * -errTicks;

  if (phase == MovePhase::FINAL && samePosCount > 0) {
    nudge += samePosCount * 2 * (errTicks > 0 ? -1 : 1);
  }

  nudge = constrain(nudge, -35.0, 35.0);
  return (int)nudge;
}

double NudgeController::phaseGain(MovePhase p) {
  switch (p) {
    case MovePhase::ACCEL: return 0.10;
    case MovePhase::COAST: return 0.12;
    case MovePhase::DECEL: return 0.16;
    case MovePhase::FINAL: return 0.25;
    default: return 0.10;
  }
}

// ----------------------------------------------------------------------
// Global NudgeController DB (per servo-id)
// ----------------------------------------------------------------------

static std::map<uint8_t, NudgeController> nudgeDB;

static NudgeController& getNudgeControllerForId(uint8_t id) {
  auto it = nudgeDB.find(id);
  if (it == nudgeDB.end()) {
    it = nudgeDB.emplace(id, NudgeController(id)).first;
  }
  return it->second;
}

// ----------------------------------------------------------------------
// AxisGroupController implementation
// ----------------------------------------------------------------------

AxisGroupController::AxisGroupController(Dynamixel2Arduino* dxl_ptr,
                                         VerticalKinematics* kin_ptr)
  : dxlPtr(dxl_ptr),
    kinPtr(kin_ptr),
    mode(AxisRunMode::UNDEFINED),
    configured(false),
    id_servo(0),
    goal_deg(0.0),
    goal_mm_x(0.0),
    goal_mm_y(0.0),
    goal_percent(0.0),
    grip_lastProgress(0.0) {
  start_ticks.assign(3, 0);
  goal_ticks.assign(3, 0);
  curr_ticks.assign(3, 0);
  id_list.assign(3, 0);
  dir_list.assign(3, 0.0);
  nudge_flags.assign(3, false);
}

void AxisGroupController::setMode(AxisRunMode m) {
  mode = m;

  goal_deg = 0.0;
  goal_mm_x = 0.0;
  goal_mm_y = 0.0;
  goal_percent = 0.0;
  grip_lastProgress = 0.0;
  configured = false;

  // Reset caches
  start_ticks.assign(3, 0);
  goal_ticks.assign(3, 0);
  curr_ticks.assign(3, 0);
  id_list.assign(3, 0);
  dir_list.assign(3, 0.0);
  nudge_flags.assign(3, false);
}

void AxisGroupController::setServoId(uint8_t id) {
  id_servo = id;
}
void AxisGroupController::setGoalDeg(double deg) {
  goal_deg = deg;
}
void AxisGroupController::setXGoalMm(double x_mm) {
  goal_mm_x = x_mm;
}
void AxisGroupController::setYGoalMm(double y_mm) {
  goal_mm_y = y_mm;
}
void AxisGroupController::setGoalPercent(double per) {
  goal_percent = per;
}

bool AxisGroupController::init() {
  switch (mode) {
    case AxisRunMode::SINGLE_SERVO: return initSingle();
    case AxisRunMode::XY_VERTICAL: return initXY(true);
    case AxisRunMode::XY_HORIZONTAL: return initXY(false);
    case AxisRunMode::GRIPPER: return initGripper();
    default: return false;
  }
}

int AxisGroupController::axesCount() const {
  switch (mode) {
    case AxisRunMode::SINGLE_SERVO: return 1;
    case AxisRunMode::GRIPPER: return 2;
    case AxisRunMode::XY_VERTICAL:
    case AxisRunMode::XY_HORIZONTAL: return 3;
    default: return 0;
  }
}

uint8_t AxisGroupController::getId(uint8_t index) const {
  if (index >= id_list.size()) return 0;
  return id_list[index];
}

int AxisGroupController::getGoalTicks(uint8_t index) const {
  if (index >= goal_ticks.size()) return 0;
  return goal_ticks[index];
}

const char* AxisGroupController::getMoveName() const {
  switch (mode) {
    case AxisRunMode::SINGLE_SERVO: return id2name(getId(0));
    case AxisRunMode::GRIPPER: return "gripper";
    case AxisRunMode::XY_VERTICAL: return "xy vert";
    case AxisRunMode::XY_HORIZONTAL: return "xy horiz";
    default: return "na";
  }
}

bool AxisGroupController::getNudgeFlag(uint8_t index) const {
  if (index >= nudge_flags.size()) return false;
  return nudge_flags[index];
}

NudgeController* AxisGroupController::getNudgeController(uint8_t index) {
  if (index >= id_list.size()) return nullptr;
  uint8_t id = id_list[index];
  if (id == 0) return nullptr;
  return &getNudgeControllerForId(id);
}

void AxisGroupController::start() {
  int n = axesCount();
  for (int i = 0; i < n; i++) {
    uint8_t id = getId(i);
    if (id > 0 && dxlPtr->ping(id)) {
      dxlPtr->ledOn(id);
    }
  }
  //DEBUG_INFO(MOD_SERVO_MOVE, "axes start | leds on");
}

void AxisGroupController::end() {
  int n = axesCount();
  for (int i = 0; i < n; i++) {
    uint8_t id = getId(i);
    if (id > 0 && dxlPtr->ping(id)) {
      dxlPtr->ledOff(id);
    }
  }
  //DEBUG_INFO(MOD_SERVO_MOVE, "axes end | leds off");
}

void AxisGroupController::readPresentTicks(int* posList) {
  int n = axesCount();
  for (int i = 0; i < n; i++) {
    uint8_t id = getId(i);
    if (id > 0) {
      int pos = dxlPtr->getPresentPosition(id);
      posList[i] = pos;
      curr_ticks[i] = pos;
    } else {
      posList[i] = 0;
      curr_ticks[i] = 0;
    }
  }
}

// -------------------------- init helpers --------------------------

bool AxisGroupController::initSingle() {
  if (!dxlPtr->ping(id_servo)) {
    //DEBUG_INFO(MOD_SERVO_MOVE, "[INIT SINGLE] ⚠ Servo %d not responding", id_servo);
    return false;
  }

  id_list[0] = id_servo;
  id_list[1] = 0;
  id_list[2] = 0;

  int start_ticks_servo = dxlPtr->getPresentPosition(id_list[0]);
  int goal_ticks_servo = deg2ticks(id_list[0], goal_deg);

  start_ticks[0] = start_ticks_servo;
  start_ticks[1] = -1;
  start_ticks[2] = -1;

  goal_ticks[0] = goal_ticks_servo;
  goal_ticks[1] = -1;
  goal_ticks[2] = -1;

  nudge_flags[0] = false;
  nudge_flags[1] = false;
  nudge_flags[2] = false;

  dir_list[0] = (goal_ticks_servo - start_ticks_servo >= 0) ? 1.0 : -1.0;
  dir_list[1] = 0.0;
  dir_list[2] = 0.0;

  configured = true;

  //DEBUG_INFO(MOD_SERVO_MOVE, "[INIT SINGLE] id start goal_deg=%.2f goal_ticks",
  //     id_list[0], start_ticks[0], goal_deg, goal_ticks[0]);
  return true;
}

bool AxisGroupController::initXY(bool keepX) {

  // ----------------------------------------------------------
  // Servo presence validation
  // ----------------------------------------------------------
  bool ping1 = dxlPtr->ping(ID_ARM1);
  bool ping2 = dxlPtr->ping(ID_ARM2);
  bool pingW = dxlPtr->ping(ID_WRIST);

  if (!ping1 || !ping2 || !pingW) {
    //DEBUG_INFO(MOD_SERVO_MOVE, "[INIT XY] ⚠ Missing servo(s): arm1 arm2 wrist",
    //    ping1, ping2, pingW);
    return false;
  }

  id_list[0] = ID_ARM1;
  id_list[1] = ID_ARM2;
  id_list[2] = ID_WRIST;

  start_ticks[0] = dxlPtr->getPresentPosition(ID_ARM1);
  start_ticks[1] = dxlPtr->getPresentPosition(ID_ARM2);
  start_ticks[2] = dxlPtr->getPresentPosition(ID_WRIST);

  // ----------------------------------------------------------
  // Compute current XY based on A1/A2
  // ----------------------------------------------------------
  double a1_center_deg = ticks2deg(ID_ARM1, start_ticks[0]);
  double a2_center_deg = ticks2deg(ID_ARM2, start_ticks[1]);

  if (!kinPtr->solve_x_y_from_a1_a2(a1_center_deg, a2_center_deg))
    return false;

  double x_now = kinPtr->getXmm();
  double y_now = kinPtr->getYmm();

  // ----------------------------------------------------------
  // Wrist orientation classification (VERT, LEFT, RIGHT)
  // ----------------------------------------------------------

  double g_vert = kinPtr->getWdeg_for_vertical();
  double g_horiz_left = kinPtr->getWdeg_for_horizontal_left();
  double g_horiz_right = kinPtr->getWdeg_for_horizontal_right();
  double g_present = kinPtr->getWdeg();

  // persistent state for hysteresis
  static WristPos wrist_last_state = WRIST_VERT;

  auto norm180 = [&](double d) {
    while (d < -180) d += 360.0;
    while (d >= 180) d -= 360.0;
    return d;
  };

  // normalize
  g_vert = norm180(g_vert);
  g_horiz_left = norm180(g_horiz_left);
  g_horiz_right = norm180(g_horiz_right);
  g_present = norm180(g_present);

  auto angDist180 = [&](double a, double b) {
    return fabs(norm180(a - b));
  };

  double dV = angDist180(g_present, g_vert);
  double dHL = angDist180(g_present, g_horiz_left);
  double dHR = angDist180(g_present, g_horiz_right);

  // closest raw
  WristPos best_state = WRIST_VERT;
  double best = dV;

  if (dHL < best) {
    best = dHL;
    best_state = WRIST_HORIZ_LEFT;
  }
  if (dHR < best) {
    best = dHR;
    best_state = WRIST_HORIZ_RIGHT;
  }

  // hysteresis
  const double HYS = 3.0;
  double dCurrent;

  switch (wrist_last_state) {
    case WRIST_VERT: dCurrent = dV; break;
    case WRIST_HORIZ_LEFT: dCurrent = dHL; break;
    case WRIST_HORIZ_RIGHT: dCurrent = dHR; break;
  }

  WristPos wrist_pos;
  if (best < dCurrent - HYS)
    wrist_pos = best_state;
  else
    wrist_pos = wrist_last_state;

  wrist_last_state = wrist_pos;

  // ----------------------------------------------------------
  // Determine goal XY
  // ----------------------------------------------------------
  if (keepX) goal_mm_x = x_now;
  else goal_mm_y = y_now;

  const char* wrist_name =
    (wrist_pos == WRIST_VERT) ? "vert" : (wrist_pos == WRIST_HORIZ_LEFT) ? "left"
                                                                         : "right";

  //DEBUG_INFO(MOD_SERVO_MOVE,
  //        "[INIT XY] currentXY=(%.2f, %.2f)mm -> goalXY=(%.2f, %.2f)mm, "
  //       "keepX, wrist",
  //       x_now, y_now, goal_mm_x, goal_mm_y, keepX, wrist_name);

  // ----------------------------------------------------------
  // Solve for new A1/A2 based on target XY
  // ----------------------------------------------------------
  kinPtr->solve_a1_a2_from_x_y(goal_mm_x, goal_mm_y);

  double a1 = kinPtr->getA1deg();
  double a2 = kinPtr->getA2deg();

  // pick G based on wrist state
  double g;
  switch (wrist_pos) {
    case WRIST_VERT:
      g = kinPtr->getWdeg_for_vertical();
      break;
    case WRIST_HORIZ_LEFT:
      g = kinPtr->getWdeg_for_horizontal_left();
      break;
    case WRIST_HORIZ_RIGHT:
      g = kinPtr->getWdeg_for_horizontal_right();
      break;
  }

  // ----------------------------------------------------------
  // Convert to ticks & direction
  // ----------------------------------------------------------
  goal_ticks[0] = deg2ticks(ID_ARM1, a1);
  goal_ticks[1] = deg2ticks(ID_ARM2, a2);
  goal_ticks[2] = deg2ticks(ID_WRIST, g);

  nudge_flags[0] = true;
  nudge_flags[1] = true;
  nudge_flags[2] = false;

  dir_list[0] = (goal_ticks[0] - start_ticks[0] >= 0) ? 1.0 : -1.0;
  dir_list[1] = (goal_ticks[1] - start_ticks[1] >= 0) ? 1.0 : -1.0;
  dir_list[2] = (goal_ticks[2] - start_ticks[2] >= 0) ? 1.0 : -1.0;

  configured = true;

  //DEBUG_INFO(MOD_SERVO_MOVE, "[INIT XY] a1=%.2f° a2=%.2f° g=%.2f°", a1, a2, g);
  //DEBUG_INFO(MOD_SERVO_MOVE, "[INIT XY] arm1 start goal Δ",
  //      start_ticks[0], goal_ticks[0],
  //    goal_ticks[0] - start_ticks[0]);
  //DEBUG_INFO(MOD_SERVO_MOVE, "[INIT XY] arm2 start goal Δ",
  //    start_ticks[1], goal_ticks[1],
  //    goal_ticks[1] - start_ticks[1]);
  //DEBUG_INFO(MOD_SERVO_MOVE, "[INIT XY] wrist start goal Δ",
  //    start_ticks[2], goal_ticks[2],
  //   goal_ticks[2] - start_ticks[2]);

  return true;
}

bool AxisGroupController::initGripper() {
  if (!dxlPtr->ping(ID_GRIP1) || !dxlPtr->ping(ID_GRIP2)) {
    //DEBUG_INFO(MOD_SERVO_MOVE, "[INIT GRIP] ⚠ Gripper ping failed ping g1 ping g2",
    //          dxlPtr->ping(ID_GRIP1), dxlPtr->ping(ID_GRIP2));
    return false;
  }

  int start1 = dxlPtr->getPresentPosition(ID_GRIP1);
  int start2 = dxlPtr->getPresentPosition(ID_GRIP2);

  int goal1 = per2ticks(ID_GRIP1, goal_percent);
  int goal2 = per2ticks(ID_GRIP2, goal_percent);

  int travel1 = abs(goal1 - start1);
  int travel2 = abs(goal2 - start2);

  // master = bigger travel
  if (travel1 >= travel2) {
    id_list[0] = ID_GRIP1;
    id_list[1] = ID_GRIP2;
    id_list[2] = 0;

    start_ticks[0] = start1;
    start_ticks[1] = start2;
    start_ticks[2] = -1;

    goal_ticks[0] = goal1;
    goal_ticks[1] = goal2;
    goal_ticks[2] = -1;
  } else {
    id_list[0] = ID_GRIP2;
    id_list[1] = ID_GRIP1;
    id_list[2] = 0;

    start_ticks[0] = start2;
    start_ticks[1] = start1;
    start_ticks[2] = -1;

    goal_ticks[0] = goal2;
    goal_ticks[1] = goal1;
    goal_ticks[2] = -1;
  }

  nudge_flags[0] = false;
  nudge_flags[1] = false;
  nudge_flags[2] = false;

  dir_list[0] = (goal_ticks[0] - start_ticks[0] >= 0) ? 1.0 : -1.0;
  dir_list[1] = (goal_ticks[1] - start_ticks[1] >= 0) ? 1.0 : -1.0;
  dir_list[2] = 0.0;

  configured = true;
  grip_lastProgress = 0.0;

  //DEBUG_INFO(MOD_SERVO_MOVE, "[INIT GRIP] goal%%=%.1f start1 start2 goal1 goal2",
  //   goal_percent, start1, start2, goal1, goal2);
  //DEBUG_INFO(MOD_SERVO_MOVE, "[INIT GRIP] travel1 travel2", travel1, travel2);
  return true;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ----------------------------------------------------------------------
// Global AxisGroupController instance
// ----------------------------------------------------------------------

AxisGroupController axes(&dxl, &kin);

// ----------------------------------------------------------------------
// PID + sync helpers (v2)
// ----------------------------------------------------------------------

// XL430 practical ranges
static const int P_MIN = 300;
static const int P_MAX = 2000;
static const int I_MIN = 0;
static const int I_MAX = 1000;
static const int D_MIN = 50;
static const int D_MAX = 600;

// Velocity & Accel ranges
static const int VEL_MIN = 10;   // don't crawl
static const int VEL_MAX = 100;  // ~23 rpm
static const int ACCEL_MIN = 40;
static const int ACCEL_MAX = 300;

static double clamp01(double x) {
  if (x < 0.0) return 0.0;
  if (x > 1.0) return 1.0;
  return x;
}

static void setPid(uint8_t id, double nP, double nI, double nD) {
  if (id == 0) return;

  nP = clamp01(nP);
  nI = clamp01(nI);
  nD = clamp01(nD);

  int P = P_MIN + (int)(nP * (P_MAX - P_MIN));
  int I = I_MIN + (int)(nI * (I_MAX - I_MIN));
  int D = D_MIN + (int)(nD * (D_MAX - D_MIN));

  //DEBUG_INFO(MOD_SERVO_MOVE, "PID id P I D", id, P, I, D);

  dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, P);
  dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, id, I);
  dxl.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, id, D);
}

static void syncServoMotion(uint8_t id1, uint8_t id2, uint8_t id3,
                            int dist1, int dist2, int dist3) {
  if (dist1 < 0) dist1 = 0;
  if (dist2 < 0) dist2 = 0;
  if (dist3 < 0) dist3 = 0;

  int maxDist = std::max(dist1, std::max(dist2, dist3));

  if (maxDist <= 0) {
    // No motion: gentle holding PIDs
    if (id1) setPid(id1, 0.1, 0.0, 0.8);
    if (id2) setPid(id2, 0.1, 0.0, 0.8);
    if (id3) setPid(id3, 0.1, 0.0, 0.8);
    return;
  }

  auto ratio = [&](int d) -> double {
    if (d <= 0) return 0.0;
    return clamp01((double)d / (double)maxDist);
  };

  double r1 = ratio(dist1);
  double r2 = ratio(dist2);
  double r3 = ratio(dist3);

  auto mapP = [&](double r) {
    return 0.25 + 0.75 * r;
  };  // 0.25–1.0
  auto mapI = [&](double r) {
    return 0.05 + 0.60 * r;
  };  // 0.05–0.65
  auto mapD = [&](double r) {
    return 0.80 - 0.60 * r;
  };  // 0.8–0.2

  if (id1) setPid(id1, mapP(r1), mapI(r1), mapD(r1));
  if (id2) setPid(id2, mapP(r2), mapI(r2), mapD(r2));
  if (id3) setPid(id3, mapP(r3), mapI(r3), mapD(r3));

  auto velFromRatio = [&](double r) -> int {
    if (r <= 0) return 0;
    double v = VEL_MIN + r * (VEL_MAX - VEL_MIN);
    return (int)v;
  };

  auto accelFromVel = [&](int v) -> int {
    if (v <= 0) return 0;
    int a = (int)(v * 2.5);
    if (a < ACCEL_MIN) a = ACCEL_MIN;
    if (a > ACCEL_MAX) a = ACCEL_MAX;
    return a;
  };

  // ---- Velocity scaled by speed ----
  double speed_to_use = speed;
  if (speed > max_speed) speed_to_use = max_speed;
  double s = clamp01(speed_to_use);
  int v1 = (int)(velFromRatio(r1) * s);
  int v2 = (int)(velFromRatio(r2) * s);
  int v3 = (int)(velFromRatio(r3) * s);

  // ---- Safe minimum ----
  v1 = max(5, v1);
  v2 = max(5, v2);
  v3 = max(5, v3);

  int a1 = accelFromVel(v1);
  int a2 = accelFromVel(v2);
  int a3 = accelFromVel(v3);

  if (id1) {
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id1, v1);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id1, a1);
  }
  if (id2) {
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id2, v2);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id2, a2);
  }
  if (id3) {
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id3, v3);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id3, a3);
  }

  //DEBUG_INFO(MOD_SERVO_MOVE, "SyncMotion: [%d:%d/%d]  [%d:%d/%d]  [%d:%d/%d]",
  //    id1, v1, a1, id2, v2, a2, id3, v3, a3);
}

static bool refineEndPositions(uint8_t id1, uint8_t id2, uint8_t id3,
                               int goal1, int goal2, int goal3) {
  const int REFINE_ERR = 4;
  const int REFINE_THRESH = 40;
  const uint32_t TIMEOUT = 500;

  int last_goal1 = -1;
  int last_goal2 = -1;
  int last_goal3 = -1;

  uint32_t t0 = millis();

  while (millis() - t0 < TIMEOUT) {
    int p1 = 0, p2 = 0, p3 = 0;

    if (id1) p1 = dxl.getPresentPosition(id1);
    if (id2) p2 = dxl.getPresentPosition(id2);
    if (id3) p3 = dxl.getPresentPosition(id3);

    bool done1 = (!id1 || goal1 == -1) || (abs(goal1 - p1) < REFINE_ERR);
    bool done2 = (!id2 || goal2 == -1) || (abs(goal2 - p2) < REFINE_ERR);
    bool done3 = (!id3 || goal3 == -1) || (abs(goal3 - p3) < REFINE_ERR);

    if (done1 && done2 && done3) return true;

    if (id1 && goal1 != -1 && abs(goal1 - p1) > REFINE_THRESH) {
      if (last_goal1 != goal1)
        if (!safeSetGoalPosition(id1, goal1)) return false;
      last_goal1 = goal1;
    }

    if (id2 && goal2 != -1 && abs(goal2 - p2) > REFINE_THRESH) {
      if (last_goal2 != goal2)
        if (!safeSetGoalPosition(id2, goal2)) return false;
      last_goal2 = goal2;
    }

    if (id3 && goal3 != -1 && abs(goal3 - p3) > REFINE_THRESH) {
      if (last_goal3 != goal3)
        if (!safeSetGoalPosition(id3, goal3)) return false;
      last_goal3 = goal3;
    }
  }
  return true;
}

// ----------------------------------------------------------------------
// move_smooth() wrapper -> v2
// ----------------------------------------------------------------------

bool move_smooth() {
  bool ok = move_smooth_v2();
  // print_kinematics_state("move_end");
  return ok;
}

/*
safe_delay(1000, { ID_BASE, ID_ARM1 });
safe_delay(800,  { ID_GRIP1, ID_GRIP2 });
safe_delay(500,  { });   // check all
safe_delay(300,  { ID_WRIST });
*/
// Define known IDs in one place
bool is_known_servo_id(int id) {
  return (id == ID_BASE || id == ID_ARM1 || id == ID_ARM2 || id == ID_GRIP1 || id == ID_GRIP2 || id == ID_WRIST);
}

bool safe_delay(unsigned long delay_millis, std::initializer_list<int> ids) {
  unsigned long start_millis = millis();

  while ((millis() - start_millis) < delay_millis) {

    if (ids.size() == 0) {
      // Check ALL servos
      int all_ids[] = { ID_BASE, ID_ARM1, ID_ARM2, ID_GRIP1, ID_GRIP2, ID_WRIST };
      for (int id : all_ids)
        if (!servo_ok(id)) return false;
    } else {
      // Check ONLY known IDs from the list
      for (int id : ids) {
        if (!is_known_servo_id(id)) {
          LOG_ERR(MOD_SERVO_MOVE, "error", "unknown_servo_id");
          LOG_VAR("id", id);
          continue;
        }

        if (!servo_ok(id))
          return false;
      }
    }
    delay(5);
  }

  return true;
}

// ----------------------------------------------------------------------
// move_smooth_v2  (DX-built-in profile + sync + refine)
// ----------------------------------------------------------------------

bool move_smooth_v2() {
  const int axes_count = axes.axesCount();
  if (axes_count == 0) return false;

  axes.start();

  int startTicks[3] = { 0, 0, 0 };
  int goalTicks[3] = { 0, 0, 0 };
  uint8_t ids[3] = { 0, 0, 0 };

  axes.readPresentTicks(startTicks);

  for (int i = 0; i < axes_count; i++) {
    goalTicks[i] = axes.getGoalTicks(i);
    ids[i] = axes.getId(i);

    int mn = getMin_ticks(ids[i]);
    int mx = getMax_ticks(ids[i]);

    if (goalTicks[i] > mx - 5) goalTicks[i] = mx - 5;
    if (goalTicks[i] < mn + 5) goalTicks[i] = mn + 5;
  }

  //DEBUG_INFO(MOD_SERVO_MOVE,
  //      "start move | axes | present={%d,%d,%d} | goals={%d,%d,%d}",
  //     axes_count,
  //      startTicks[0], startTicks[1], startTicks[2],
  //       goalTicks[0], goalTicks[1], goalTicks[2]);

  int distA = (axes_count >= 1) ? abs(goalTicks[0] - startTicks[0]) : 0;
  int distB = (axes_count >= 2) ? abs(goalTicks[1] - startTicks[1]) : 0;
  int distC = (axes_count >= 3) ? abs(goalTicks[2] - startTicks[2]) : 0;

  if (distA <= 4 && distB <= 4 && distC <= 4) {
    //DEBUG_INFO(MOD_SERVO_MOVE,
    //      "end move | axes | present={%d,%d,%d} | goals={%d,%d,%d}",
    //      axes_count,
    //      startTicks[0], startTicks[1], startTicks[2],
    //     goalTicks[0], goalTicks[1], goalTicks[2]);
    axes.end();
    return true;
  }

  syncServoMotion(
    (axes_count >= 1) ? ids[0] : 0,
    (axes_count >= 2) ? ids[1] : 0,
    (axes_count >= 3) ? ids[2] : 0,
    distA, distB, distC);

  if (axes_count >= 1)
    if (!safeSetGoalPosition(ids[0], goalTicks[0])) return false;
  if (axes_count >= 2)
    if (!safeSetGoalPosition(ids[1], goalTicks[1])) return false;
  if (axes_count >= 3)
    if (!safeSetGoalPosition(ids[2], goalTicks[2])) return false;

  if (!refineEndPositions(
        (axes_count >= 1) ? ids[0] : 0,
        (axes_count >= 2) ? ids[1] : 0,
        (axes_count >= 3) ? ids[2] : 0,
        (axes_count >= 1) ? goalTicks[0] : -1,
        (axes_count >= 2) ? goalTicks[1] : -1,
        (axes_count >= 3) ? goalTicks[2] : -1)) return false;

  // ----------------------------------------------------
  //   FINAL VERIFY: Make sure movement actually finished
  // ----------------------------------------------------

  // delay(50);

  // --------------------- INITIAL ERROR LOG ------------------------
  String err0 = (axes_count >= 1) ? String(abs(dxl.getPresentPosition(ids[0]) - goalTicks[0])) : "na";
  String err1 = (axes_count >= 2) ? String(abs(dxl.getPresentPosition(ids[1]) - goalTicks[1])) : "na";
  String err2 = (axes_count >= 3) ? String(abs(dxl.getPresentPosition(ids[2]) - goalTicks[2])) : "na";

  //DEBUG_INFO(MOD_SERVO_MOVE, "final verify | axes | err0 | err1 | err2",
  //      axes_count,
  //      err0.c_str(), err1.c_str(), err2.c_str());

  // ---------------------- SETTLE LOOP ------------------------------
  for (int i = 0; i < axes_count; i++) {

    uint8_t id = ids[i];
    if (id == 0) continue;  // safety

    int lastPos = dxl.getPresentPosition(id);

    // If already within tolerance, no need to settle
    if (abs(lastPos - goalTicks[i]) <= 4) continue;

    uint32_t t0 = millis();

    while (millis() - t0 < 150) {  // wait for stable reading

      int p = dxl.getPresentPosition(id);

      if (abs(p - lastPos) > 1) {
        // still moving → reset timer + update lastPos
        t0 = millis();
        lastPos = p;
      }
      if (!safe_delay(5, { id })) return false;
    }
  }

  // --------------------- RESEND IF STILL OFF -----------------------
  for (int t = 0; t < 3; t++) {

    for (int i = 0; i < axes_count; i++) {

      uint8_t id = ids[i];
      if (id == 0) continue;  // safety

      int finalPos = dxl.getPresentPosition(id);
      int diff = abs(finalPos - goalTicks[i]);

      if (diff > 4) {
        //DEBUG_INFO(MOD_SERVO_MOVE, "final check: servo %d still off by %d → resending goal",
        //        id, diff);
        if (!safeSetGoalPosition(id, goalTicks[i])) return false;
        if (!safe_delay(50, { id })) return false;
      }
    }
  }

  // ---------------------- FINAL ERROR READBACK ----------------------
  err0 = (axes_count >= 1) ? String(abs(dxl.getPresentPosition(ids[0]) - goalTicks[0])) : "na";
  err1 = (axes_count >= 2) ? String(abs(dxl.getPresentPosition(ids[1]) - goalTicks[1])) : "na";
  err2 = (axes_count >= 3) ? String(abs(dxl.getPresentPosition(ids[2]) - goalTicks[2])) : "na";

  //DEBUG_INFO(MOD_SERVO_MOVE, "end move | axes | err0 | err1 | err2",
  //      axes_count,
  //     err0.c_str(), err1.c_str(), err2.c_str());

  axes.end();

  return true;
}

// ----------------------------------------------------------------------
// Status helpers & command wrappers
// ----------------------------------------------------------------------
void read_print_kinematics_state(char* descr) {
  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  kin.solve_x_y_from_a1_a2(a1_deg, a2_deg);
  print_kinematics_state(descr);
}

void print_kinematics_state(char* descr) {
  LOG_INFO(MOD_SERVO_MOVE, "info", "kinematics_state");
  if (descr != nullptr) LOG_VAR("at", descr);

  double g1_per = ticks2per(ID_GRIP1, dxl.getPresentPosition(ID_GRIP1));
  double g2_per = ticks2per(ID_GRIP2, dxl.getPresentPosition(ID_GRIP2));
  double base_deg = ticks2deg(ID_BASE, dxl.getPresentPosition(ID_BASE));

  LOG_INFO(MOD_SERVO_MOVE, "info", "xy_arms");
  LOG_VAR("x_mm", kin.getXmm());
  LOG_VAR("y_mm", kin.getYmm());
  LOG_VAR("a1_deg", kin.getA1deg());
  LOG_VAR("a2_deg", kin.getA2deg());

  LOG_INFO(MOD_SERVO_MOVE, "info", "wrist");
  LOG_VAR("w_deg", kin.getWdeg());
  LOG_VAR("w_horiz_r_deg", kin.getWdeg_for_horizontal_right());
  LOG_VAR("w_horiz_l_deg", kin.getWdeg_for_horizontal_left());
  LOG_VAR("w_vert_deg", kin.getWdeg_for_vertical());

  LOG_INFO(MOD_SERVO_MOVE, "info", "grip_and_base");
  LOG_VAR("g1_per", g1_per);
  LOG_VAR("g2_per", g2_per);
  LOG_VAR("base_deg", base_deg);
}

bool cmdMoveServoDeg(uint8_t id, double goal_deg) {
  RUN_PING(id);

  axes.setMode(AxisGroupController::AxisRunMode::SINGLE_SERVO);
  axes.setServoId(id);
  axes.setGoalDeg(goal_deg);
  if (!axes.init()) return false;

  ////DEBUG_INFO(MOD_SERVO_MOVE,"START move_smooth for SINGLE_SERVO %d %.2f************************", id, goal_deg);
  bool r = move_smooth();
  ////DEBUG_INFO(MOD_SERVO_MOVE,"END move_smooth for SINGLE_SERVO %d %.2f************************", id, goal_deg);
  return r;
}

bool cmdMoveServoPer(int id, double goal_per) {
  RUN_PING(id);

  if (goal_per < -15.0 || goal_per > 115.0) {
    LOG_ERR(MOD_SERVO_MOVE, "error", "invalid_servo_percentage");

    LOG_VAR("goal_per", goal_per);
    LOG_VAR("expected_min_per", -15.0);
    LOG_VAR("expected_max_per", 115.0);
    return false;
  }

  double goal_deg = per2deg((uint8_t)id, goal_per);
  //DEBUG_INFO(MOD_SERVO_MOVE, "cmd_move_per: id per=%.2f deg=%.2f", id, goal_per, goal_deg);

  if (!cmdMoveServoDeg((uint8_t)id, goal_deg)) return false;
  print_servo_status((uint8_t)id);
  return true;
}

//------------------------------------------------------
// RAW 16-bit read (works with your library)
//------------------------------------------------------
int16_t readReg16(uint8_t id, uint16_t addr) {
  uint8_t data[2] = { 0, 0 };
  int32_t res = dxl.read(id, addr, 2, data, 2, 20);
  if (res > 0)
    return (int16_t)(data[0] | (data[1] << 8));
  return 0;
}

const int PWM_TOUCH = 185;  // TODO_ADJUST was 90
double MAX_PER_CLAMP_GRIP = 90.0;

bool isGripAtTouch(int pwm) {
  return (abs(pwm) > PWM_TOUCH);
}

bool cmdMoveGripperClamp() {
  RUN_CMD(cmdSquareBase(),"square base before clamp");

  if (getPos_per(ID_GRIP1) < 85.0 || getPos_per(ID_GRIP2) < 85.0) {
    if (!cmdMoveGripperPer(85.0)) return false;

    setPid(ID_GRIP1, 0.8, 0.10, 0.35);
    setPid(ID_GRIP2, 0.8, 0.10, 0.35);

    const uint16_t PWM_REG = 124;
    const double extraGrip2Fingers = 2.5;  //TODO_ADJUST was 1.5

    bool touched1 = false;
    bool touched2 = false;
    bool extraDone = false;

    double per1 = getPos_per(ID_GRIP1);
    double per2 = getPos_per(ID_GRIP2);

    const double STEP = 0.8;  // faster closing
    double extraGrip1Finger = 6.0;

    for (int step = 0; step < 80; step++) {
      if (per1 > (MAX_PER_CLAMP_GRIP + extraGrip1Finger) &&  //
          per2 > (MAX_PER_CLAMP_GRIP + extraGrip1Finger)) {
        LOG_ERR(MOD_SERVO_MOVE, "no clamp at max grip per1", per1);
        LOG_VAR("per2", per2);
        return false;
      }
      if (!touched1) dxl.setGoalPosition(ID_GRIP1, per2ticks(ID_GRIP1, per1));
      if (!touched2) dxl.setGoalPosition(ID_GRIP2, per2ticks(ID_GRIP2, per2));

      unsigned long until = millis() + 10;  // faster check
      while (millis() < until) {

        int16_t pwm1 = readReg16(ID_GRIP1, PWM_REG);
        int16_t pwm2 = readReg16(ID_GRIP2, PWM_REG);

        if (!touched1 && isGripAtTouch(pwm1)) touched1 = true;
        if (!touched2 && isGripAtTouch(pwm2)) touched2 = true;

        delay(2);  // faster update
      }
      delay(45);

      // freeze AFTER marking touch (faster syncing)
      if (touched1) {
        int hold = dxl.getPresentPosition(ID_GRIP1);
        dxl.setGoalPosition(ID_GRIP1, hold);
      }
      if (touched2) {
        int hold = dxl.getPresentPosition(ID_GRIP2);
        dxl.setGoalPosition(ID_GRIP2, hold);
      }

      if (!extraDone && touched1 && touched2) {
        per1 += extraGrip2Fingers;
        per2 += extraGrip2Fingers;

        if (per1 > 105.0) per1 = 105.0;
        if (per2 > 105.0) per2 = 105.0;

        dxl.setGoalPosition(ID_GRIP1, per2ticks(ID_GRIP1, per1));
        dxl.setGoalPosition(ID_GRIP2, per2ticks(ID_GRIP2, per2));

        extraDone = true;
        break;
      }

      if (!touched1) per1 += STEP;
      if (!touched2) per2 += STEP;

      if (per1 > 115.0) per1 = 115.0;
      if (per2 > 115.0) per2 = 115.0;
    }
    // Serial.println(touched1);
    // Serial.println(touched2);
    return (touched1 && touched2);
  }
  return false;
}

bool isGripperOpen(double min_open) {
  RUN_PING(ID_GRIP1);
  RUN_PING(ID_GRIP2);

  double g1_pos = getPos_per(ID_GRIP1);
  double g2_pos = getPos_per(ID_GRIP2);
  if (g1_pos < min_open + 3 && g2_pos < min_open + 3) return true;
  return false;
}

bool isYmmAbove(double min_y) {
  if (!dxl_ping_cached(ID_ARM1) || !dxl_ping_cached(ID_ARM2)) return false;
  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  kin.solve_x_y_from_a1_a2(a1_deg, a2_deg);

  if (kin.getYmm() > min_y) return true;
  return false;
}

bool cmdMoveGripperPer(double goal_per) {
  if (!dxl_ping_cached(ID_GRIP1) || !dxl_ping_cached(ID_GRIP2)) return false;

  set_torque_all_servos(true);

  // return if already there
  double now_g1_per = ticks2per(ID_GRIP1, dxl.getPresentPosition(ID_GRIP1));
  double now_g2_per = ticks2per(ID_GRIP2, dxl.getPresentPosition(ID_GRIP2));
  double err_g1_deg = fabs(now_g1_per - goal_per);
  double err_g2_deg = fabs(now_g2_per - goal_per);
  if (err_g1_deg <= tol_g_per && err_g2_deg <= tol_g_per) return true;


  axes.setMode(AxisGroupController::AxisRunMode::GRIPPER);
  axes.setGoalPercent(goal_per);
  if (!axes.init()) return false;

  //DEBUG_INFO(MOD_SERVO_MOVE, "START move_smooth for MODE_GRIPPER");
  bool ret = move_smooth();
  // read_print_kinematics_state();
  return ret;
}

bool cmdMoveWristDegVertical(double goal_deg) {
  if (!dxl_ping_cached(ID_ARM1) || !dxl_ping_cached(ID_ARM2) || !dxl_ping_cached(ID_WRIST)) return false;

  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  if (!kin.solve_x_y_from_a1_a2(a1_deg, a2_deg)) return false;

  double vert_deg = kin.getWdeg_for_vertical();

  /*
#define W_HORIZ_RIGHT -90 + VERT_CORRECTION  // -85
#define W_VERT 0 + VERT_CORRECTION           // 0
#define W_HORIZ_LEFT 90 + VERT_CORRECTION    //95
*/

  if (vert_deg < W_VERT - 5 || vert_deg > W_VERT + 5) {
    if (kin.getYmm() < min_ymm) {
      DEBUG_ERR(MOD_SERVO_MOVE, "y too low to rotate gripper");
      DEBUG_VAR("y_mm", kin.getYmm());
      DEBUG_VAR("min_y_mm", min_ymm);
      DEBUG_VAR("g_goal", vert_deg);
      DEBUG_VAR("g_vert_ref", W_VERT);

      return false;
    }
  }

  // return if already there
  double now_w_deg = ticks2deg(ID_WRIST, dxl.getPresentPosition(ID_WRIST));
  double err_w_deg = fabs(now_w_deg - vert_deg - goal_deg);
  if (err_w_deg <= tol_w_deg) return true;

  axes.setMode(AxisGroupController::AxisRunMode::SINGLE_SERVO);
  axes.setServoId(ID_WRIST);
  axes.setGoalDeg(vert_deg + goal_deg);
  if (!axes.init()) return false;

  //DEBUG_INFO(MOD_SERVO_MOVE, "START move_smooth for WRIST");
  return move_smooth();
}

bool cmdMoveYmm(double goal_ymm) {
  if (!dxl_ping_cached(ID_ARM1) || !dxl_ping_cached(ID_ARM2)) return false;

  double g_at_vert = kin.getWdeg_for_vertical();
  double g_relative_to_vert = kin.getWdeg() - g_at_vert;

  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  if (!kin.solve_x_y_from_a1_a2(a1_deg, a2_deg)) return false;

  if (goal_ymm < min_ymm) {
    DEBUG_ERR(MOD_SERVO_MOVE, "y too low due to vertical gripper");
    DEBUG_VAR("min_y_mm", min_ymm);
    DEBUG_VAR("g_vert_deg", kin.getWdeg_for_vertical());
    DEBUG_VAR("g_deg", kin.getWdeg());
    DEBUG_VAR("g_rel_vert_deg", g_relative_to_vert);

    return false;
  }

  // return if already there
  double now_ymm = kin.getYmm();
  double err_ymm = fabs(now_ymm - goal_ymm);
  if (err_ymm <= tol_ymm) return true;

  axes.setMode(AxisGroupController::AxisRunMode::XY_VERTICAL);
  axes.setXGoalMm(kin.getXmm());  // keep X
  axes.setYGoalMm(goal_ymm);
  if (!axes.init()) return false;

  //DEBUG_INFO(MOD_SERVO_MOVE, "START move_smooth for MODE_XY_VERTICAL");
  //DEBUG_INFO(MOD_SERVO_MOVE, "existing x=%.2f goal y=%.2f", kin.getXmm(), goal_ymm);
  double prev_speed = speed;
  if (speed > 0.35) speed = 0.35;  //TODO_ADJUST
  bool ret = move_smooth();
  speed = prev_speed;
  // read_print_kinematics_state();
  return ret;
}

bool cmdMoveXmm(double goal_xmm) {
  if (!dxl_ping_cached(ID_ARM1) || !dxl_ping_cached(ID_ARM2)) return false;

  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  if (!kin.solve_x_y_from_a1_a2(a1_deg, a2_deg)) {
    LOG_ERR(MOD_SERVO_MOVE, "kin solve failed for a1", a1_deg);
    LOG_VAR("a2", a2_deg);
    return false;
  }

  // return if already there
  double now_xmm = kin.getXmm();
  double err_xmm = fabs(now_xmm - goal_xmm);
  if (err_xmm <= tol_xmm) return true;

  axes.setMode(AxisGroupController::AxisRunMode::XY_HORIZONTAL);
  axes.setXGoalMm(goal_xmm);
  axes.setYGoalMm(kin.getYmm());  // keep Y

  if (!axes.init()) {
    LOG_ERR(MOD_SERVO_MOVE, "axes init", "fail");
    return false;
  }

  //DEBUG_INFO(MOD_SERVO_MO{VE, "START move_smooth for MODE_XY_HORIZONTAL");
  //DEBUG_INFO(MOD_SERVO_MOVE, "goal x=%.2f existing y=%.2f", x_mm, kin.getYmm());
  double prev_speed = speed;
  if (speed > 0.25) speed = 0.25;  //TODO_ADJUST
  bool ret = move_smooth();
  speed = prev_speed;
  // read_print_kinematics_state();
  return ret;
}

CubeOri ori(robot_move_callback);
