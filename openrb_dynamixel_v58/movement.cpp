#include "movement.h"

#include <map>
#include <algorithm>
#include <cmath>

#include "Dynamixel2Arduino.h"
#include "vertical_kinematics.h"
#include "ori.h"

// ----------------------------------------------------------------------
// Externals from elsewhere in the project
// (These must be defined in other translation units.)
// ----------------------------------------------------------------------
extern Dynamixel2Arduino dxl;
extern VerticalKinematics kin;
extern bool verboseOn;
extern double min_ymm;

double speed = 1.0;

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
  serial_printf_verbose("---- Nudge log for servo %d (count=%d) ----\n",
                        id, (int)records.size());

  for (auto& r : records) {
    const char* phaseStr =
      (r.phase == MovePhase::ACCEL) ? "ACC" : (r.phase == MovePhase::COAST) ? "COAST"
                                            : (r.phase == MovePhase::DECEL) ? "DEC"
                                                                            : "FINAL";

    serial_printf_verbose("[%lu ms] %s goal=%d curr=%d err=%d nudge=%d\n",
                          r.t_ms, phaseStr, r.prevGoalTicks,
                          r.currPosTicks, r.errTicks, r.nudgeApplied);
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
  serial_printf_verbose("axes start | leds on\n");
}

void AxisGroupController::end() {
  int n = axesCount();
  for (int i = 0; i < n; i++) {
    uint8_t id = getId(i);
    if (id > 0 && dxlPtr->ping(id)) {
      dxlPtr->ledOff(id);
    }
  }
  serial_printf_verbose("axes end | leds off\n");
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
    serial_printf_verbose("[INIT SINGLE] ⚠ Servo %d not responding\n", id_servo);
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

  serial_printf_verbose("[INIT SINGLE] id=%d start=%d goal_deg=%.2f goal_ticks=%d\n",
                        id_list[0], start_ticks[0], goal_deg, goal_ticks[0]);
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
    serial_printf_verbose("[INIT XY] ⚠ Missing servo(s): arm1=%d arm2=%d wrist=%d\n",
                          ping1, ping2, pingW);
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

  double g_vert = kinPtr->getGdeg_for_vertical();
  double g_horiz_left = kinPtr->getGdeg_for_horizontal_left();
  double g_horiz_right = kinPtr->getGdeg_for_horizontal_right();
  double g_present = kinPtr->getGdeg();

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

  serial_printf_verbose(
    "[INIT XY] currentXY=(%.2f, %.2f)mm -> goalXY=(%.2f, %.2f)mm, "
    "keepX=%d, wrist=%s\n",
    x_now, y_now, goal_mm_x, goal_mm_y, keepX, wrist_name);

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
      g = kinPtr->getGdeg_for_vertical();
      break;
    case WRIST_HORIZ_LEFT:
      g = kinPtr->getGdeg_for_horizontal_left();
      break;
    case WRIST_HORIZ_RIGHT:
      g = kinPtr->getGdeg_for_horizontal_right();
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

  serial_printf_verbose("[INIT XY] a1=%.2f° a2=%.2f° g=%.2f°\n", a1, a2, g);
  serial_printf_verbose("[INIT XY] arm1 start=%d goal=%d Δ=%d\n",
                        start_ticks[0], goal_ticks[0],
                        goal_ticks[0] - start_ticks[0]);
  serial_printf_verbose("[INIT XY] arm2 start=%d goal=%d Δ=%d\n",
                        start_ticks[1], goal_ticks[1],
                        goal_ticks[1] - start_ticks[1]);
  serial_printf_verbose("[INIT XY] wrist start=%d goal=%d Δ=%d\n",
                        start_ticks[2], goal_ticks[2],
                        goal_ticks[2] - start_ticks[2]);

  return true;
}

bool AxisGroupController::initGripper() {
  if (!dxlPtr->ping(ID_GRIP1) || !dxlPtr->ping(ID_GRIP2)) {
    serial_printf_verbose("[INIT GRIP] ⚠ Gripper ping failed ping g1=%d ping g2=%d\n",
                          dxlPtr->ping(ID_GRIP1), dxlPtr->ping(ID_GRIP2));
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

  serial_printf_verbose("[INIT GRIP] goal%%=%.1f start1=%d start2=%d goal1=%d goal2=%d\n",
                        goal_percent, start1, start2, goal1, goal2);
  serial_printf_verbose("[INIT GRIP] travel1=%d travel2=%d\n", travel1, travel2);
  return true;
}

// ----------------------------------------------------------------------
// Global AxisGroupController instance
// ----------------------------------------------------------------------

AxisGroupController axes(&dxl, &kin);

// ----------------------------------------------------------------------
// logProgress helper
// ----------------------------------------------------------------------

static void logProgress(const char* moveName, int startTicks, int crrTicks, int endTicks) {
  static int lastStartLogged = -1;
  static int lastEndLogged = -1;
  static int lastCrrLogged = -1;

  if (lastStartLogged != startTicks || lastEndLogged != endTicks) {
    lastCrrLogged = -1;
  }

  lastStartLogged = startTicks;
  lastEndLogged = endTicks;

  // log every ~50 ticks relative to start
  if (lastCrrLogged >= 0 && abs(crrTicks - lastCrrLogged) < 50) return;
  lastCrrLogged = crrTicks;

  serial_printf_verbose("MOVING %s %d/%d\n", moveName, crrTicks, endTicks);
}

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

  serial_printf_verbose("PID id=%d P=%d I=%d D=%d\n", id, P, I, D);

  dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, P);
  dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, id, I);
  dxl.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, id, D);
}

/**/
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
  int v1 = velFromRatio(r1) * speed;
  int v2 = velFromRatio(r2) * speed;
  int v3 = velFromRatio(r3) * speed;

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

  serial_printf_verbose("SyncMotion: [%d:%d/%d]  [%d:%d/%d]  [%d:%d/%d]\n",
                        id1, v1, a1, id2, v2, a2, id3, v3, a3);
}

static void refineEndPositions(uint8_t id1, uint8_t id2, uint8_t id3,
                               int goal1, int goal2, int goal3) {
  const int REFINE_ERR = 4;
  const int REFINE_THRESH = 40;
  const uint32_t TIMEOUT = 500;

  uint32_t t0 = millis();

  while (millis() - t0 < TIMEOUT) {
    int p1 = 0, p2 = 0, p3 = 0;

    if (id1) p1 = dxl.getPresentPosition(id1);
    if (id2) p2 = dxl.getPresentPosition(id2);
    if (id3) p3 = dxl.getPresentPosition(id3);

    bool done1 = (!id1 || goal1 == -1) || (abs(goal1 - p1) < REFINE_ERR);
    bool done2 = (!id2 || goal2 == -1) || (abs(goal2 - p2) < REFINE_ERR);
    bool done3 = (!id3 || goal3 == -1) || (abs(goal3 - p3) < REFINE_ERR);

    if (done1 && done2 && done3) return;

    if (id1 && goal1 != -1 && abs(goal1 - p1) > REFINE_THRESH)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id1, goal1);

    if (id2 && goal2 != -1 && abs(goal2 - p2) > REFINE_THRESH)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id2, goal2);

    if (id3 && goal3 != -1 && abs(goal3 - p3) > REFINE_THRESH)
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id3, goal3);
  }
}

// ----------------------------------------------------------------------
// move_smooth() wrapper -> v2
// ----------------------------------------------------------------------

bool move_smooth() {
  return move_smooth_v2();
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

  while (millis() < start_millis + delay_millis) {

    if (ids.size() == 0) {
      // Check ALL servos
      int all_ids[] = { ID_BASE, ID_ARM1, ID_ARM2, ID_GRIP1, ID_GRIP2, ID_WRIST };
      for (int id : all_ids)
        if (!servo_ok(id, false)) return false;
    } else {
      // Check ONLY known IDs from the list
      for (int id : ids) {
        if (!is_known_servo_id(id))
          continue;  // skip unknown values silently

        if (!servo_ok(id, false))
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

  serial_printf_verbose(
    "start move | axes=%d | present={%d,%d,%d} | goals={%d,%d,%d}\n",
    axes_count,
    startTicks[0], startTicks[1], startTicks[2],
    goalTicks[0], goalTicks[1], goalTicks[2]);

  int distA = (axes_count >= 1) ? abs(goalTicks[0] - startTicks[0]) : 0;
  int distB = (axes_count >= 2) ? abs(goalTicks[1] - startTicks[1]) : 0;
  int distC = (axes_count >= 3) ? abs(goalTicks[2] - startTicks[2]) : 0;

  if (distA <= 4 && distB <= 4 && distC <= 4) {
    serial_printf_verbose(
      "end move | axes=%d | present={%d,%d,%d} | goals={%d,%d,%d}\n",
      axes_count,
      startTicks[0], startTicks[1], startTicks[2],
      goalTicks[0], goalTicks[1], goalTicks[2]);
    axes.end();
    return true;
  }

  syncServoMotion(
    (axes_count >= 1) ? ids[0] : 0,
    (axes_count >= 2) ? ids[1] : 0,
    (axes_count >= 3) ? ids[2] : 0,
    distA, distB, distC);

  if (axes_count >= 1)
    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ids[0], goalTicks[0]);
  if (axes_count >= 2)
    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ids[1], goalTicks[1]);
  if (axes_count >= 3)
    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ids[2], goalTicks[2]);

  refineEndPositions(
    (axes_count >= 1) ? ids[0] : 0,
    (axes_count >= 2) ? ids[1] : 0,
    (axes_count >= 3) ? ids[2] : 0,
    (axes_count >= 1) ? goalTicks[0] : -1,
    (axes_count >= 2) ? goalTicks[1] : -1,
    (axes_count >= 3) ? goalTicks[2] : -1);

  // ----------------------------------------------------
  //   FINAL VERIFY: Make sure movement actually finished
  // ----------------------------------------------------

  // delay(50);

  // --------------------- INITIAL ERROR LOG ------------------------
  String err0 = (axes_count >= 1) ? String(abs(dxl.getPresentPosition(ids[0]) - goalTicks[0])) : "na";
  String err1 = (axes_count >= 2) ? String(abs(dxl.getPresentPosition(ids[1]) - goalTicks[1])) : "na";
  String err2 = (axes_count >= 3) ? String(abs(dxl.getPresentPosition(ids[2]) - goalTicks[2])) : "na";

  serial_printf_verbose("final verify | axes=%d | err0=%s | err1=%s | err2=%s\n",
                        axes_count,
                        err0.c_str(), err1.c_str(), err2.c_str());

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
        serial_printf_verbose("final check: servo %d still off by %d → resending goal\n",
                              id, diff);
        dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION,
                                  id, goalTicks[i]);
        if (!safe_delay(50, { id })) return false;
      }
    }
  }

  // ---------------------- FINAL ERROR READBACK ----------------------
  err0 = (axes_count >= 1) ? String(abs(dxl.getPresentPosition(ids[0]) - goalTicks[0])) : "na";
  err1 = (axes_count >= 2) ? String(abs(dxl.getPresentPosition(ids[1]) - goalTicks[1])) : "na";
  err2 = (axes_count >= 3) ? String(abs(dxl.getPresentPosition(ids[2]) - goalTicks[2])) : "na";

  serial_printf_verbose("end move | axes=%d | err0=%s | err1=%s | err2=%s\n",
                        axes_count,
                        err0.c_str(), err1.c_str(), err2.c_str());

  axes.end();

  return true;
}

// ----------------------------------------------------------------------
// Status helpers & command wrappers
// ----------------------------------------------------------------------

void read_print_xy_status() {
  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  kin.solve_x_y_from_a1_a2(a1_deg, a2_deg);
  print_xy_status();
}

void print_all_status() {
  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  double g1_per = ticks2per(ID_GRIP1, dxl.getPresentPosition(ID_GRIP1));
  double g2_per = ticks2per(ID_GRIP2, dxl.getPresentPosition(ID_GRIP2));
  double base_deg = ticks2deg(ID_BASE, dxl.getPresentPosition(ID_BASE));

  kin.solve_x_y_from_a1_a2(a1_deg, a2_deg);

  serial_printf("SERVOS info=servos_status x_mm=%.2f y_mm=%.2f a1_deg=%.2f a2_deg=%.2f g_vert_deg=%.2f g_deg=%.2f g1_per=%.2f g2_per=%.2f base_deg=%.2f\n",
    kin.getXmm(), kin.getYmm(), kin.getA1deg(), kin.getA2deg(),
    kin.getGdeg_for_vertical(), kin.getGdeg(),
    g1_per, g2_per, base_deg);
}

bool cmdMoveServoDeg(uint8_t id, double goal_deg) {
  if (!dxl.ping(id)) return false;

  axes.setMode(AxisGroupController::AxisRunMode::SINGLE_SERVO);
  axes.setServoId(id);
  axes.setGoalDeg(goal_deg);
  if (!axes.init()) return false;

  serial_printf_verbose("START move_smooth for SINGLE_SERVO %d %.2f************************\n", id, goal_deg);
  bool r = move_smooth();
  serial_printf_verbose("END move_smooth for SINGLE_SERVO %d %.2f************************\n", id, goal_deg);
  return r;
}

bool cmdMoveServoPer(int id, double goal_per) {
  if (!dxl.ping(id)) return false;

  if (goal_per < -15.0 || goal_per > 115.0) {
    serial_printf("ERR invalid servo percentage: %.2f expected range (-15.0 per to 115.0 per)\n", goal_per);
    return false;
  }

  double goal_deg = per2deg((uint8_t)id, goal_per);
  serial_printf_verbose("cmd_move_per: id=%d per=%.2f deg=%.2f\n", id, goal_per, goal_deg);

  if (!cmdMoveServoDeg((uint8_t)id, goal_deg)) return false;
  print_servo_status((uint8_t)id);
  return true;
}

//------------------------------------------------------
// RAW 16-bit read (works with your library)
//------------------------------------------------------
static int16_t readReg16(uint8_t id, uint16_t addr) {
  uint8_t data[2] = { 0, 0 };
  int32_t res = dxl.read(id, addr, 2, data, 2, 20);
  if (res > 0)
    return (int16_t)(data[0] | (data[1] << 8));
  return 0;
}

bool cmdMoveGripperClamp() {
  if (getPos_per(ID_GRIP1) < 85.0 || getPos_per(ID_GRIP2) < 85.0) {
    if (!cmdMoveGripperPer(85.0)) return false;

    setPid(ID_GRIP1, 0.8, 0.10, 0.35);
    setPid(ID_GRIP2, 0.8, 0.10, 0.35);

    const uint16_t PWM_REG = 124;
    const int PWM_TOUCH = 90;
    const double EXTRA = 3.5;  //TODO was 1.5

    bool touched1 = false;
    bool touched2 = false;
    bool extraDone = false;

    double per1 = getPos_per(ID_GRIP1);
    double per2 = getPos_per(ID_GRIP2);

    const double STEP = 0.8;  // faster closing

    for (int step = 0; step < 80; step++) {
      if (!touched1) dxl.setGoalPosition(ID_GRIP1, per2ticks(ID_GRIP1, per1));
      if (!touched2) dxl.setGoalPosition(ID_GRIP2, per2ticks(ID_GRIP2, per2));

      unsigned long until = millis() + 10;  // faster check
      while (millis() < until) {

        int16_t pwm1 = readReg16(ID_GRIP1, PWM_REG);
        int16_t pwm2 = readReg16(ID_GRIP2, PWM_REG);

        if (!touched1 && abs(pwm1) >= PWM_TOUCH) touched1 = true;
        if (!touched2 && abs(pwm2) >= PWM_TOUCH) touched2 = true;

        delay(2);  // faster update
      }
      if (!safe_delay(50, { ID_GRIP1, ID_GRIP2 })) return false;

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
        per1 += EXTRA;
        per2 += EXTRA;

        if (per1 > 105.0) per1 = 105.0;
        if (per2 > 105.0) per2 = 105.0;

        dxl.setGoalPosition(ID_GRIP1, per2ticks(ID_GRIP1, per1));
        dxl.setGoalPosition(ID_GRIP2, per2ticks(ID_GRIP2, per2));

        extraDone = true;
        break;
      }

      if (!touched1) per1 += STEP;
      if (!touched2) per2 += STEP;

      if (per1 > 105.0) per1 = 105.0;
      if (per2 > 105.0) per2 = 105.0;
    }

    return (touched1 && touched2);
  }
}

bool isGripperOpen(double min_open) {
  if (!dxl.ping(ID_GRIP1) || !dxl.ping(ID_GRIP2)) return false;
  double g1_pos = getPos_per(ID_GRIP1);
  double g2_pos = getPos_per(ID_GRIP1);
  if (g1_pos < min_open + 3 && g2_pos < min_open + 3) return true;
  return false;
}

bool cmdMoveGripperPer(double goal_per) {
  if (!dxl.ping(ID_GRIP1) || !dxl.ping(ID_GRIP2)) return false;

  axes.setMode(AxisGroupController::AxisRunMode::GRIPPER);
  axes.setGoalPercent(goal_per);
  if (!axes.init()) return false;

  serial_printf_verbose("START move_smooth for MODE_GRIPPER\n");
  bool ret = move_smooth();
  read_print_xy_status();
  return ret;
}

bool cmdMoveWristDegVertical(double goal_deg) {
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2) || !dxl.ping(ID_WRIST)) return false;

  if (goal_deg > 5 || goal_deg < -185) {
    if (kin.getYmm() < min_ymm) {
      serial_printf_verbose("ERR y=%.2f too low to rotate gripper <%.2f\n",
                            kin.getYmm(), min_ymm);
      return false;
    }
  }

  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  if (!kin.solve_x_y_from_a1_a2(a1_deg, a2_deg)) return false;

  double vert_deg = kin.getGdeg_for_vertical();
  print_xy_status();
  serial_printf_verbose("g deg vertical=%.2f, g deg move to=%.2f\n",
                        vert_deg, vert_deg + goal_deg);

  axes.setMode(AxisGroupController::AxisRunMode::SINGLE_SERVO);
  axes.setServoId(ID_WRIST);
  axes.setGoalDeg(vert_deg + goal_deg);
  if (!axes.init()) return false;

  serial_printf_verbose("START move_smooth for WRIST\n");
  return move_smooth();
}

bool cmdMoveYmm(double goal_ymm) {
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2)) return false;

  double g_at_vert = kin.getGdeg_for_vertical();
  double g_relative_to_vert = kin.getGdeg() - g_at_vert;


  if (goal_ymm < min_ymm) {
    serial_printf_verbose("ERR y too low (<%.2f) because of vertical gripper g_at_vert=%.2f g_deg=%.2f g_rel_vert=%.2f deg\n",
                          min_ymm, kin.getGdeg_for_vertical(), kin.getGdeg(), g_relative_to_vert);
    return false;
  }

  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  if (!kin.solve_x_y_from_a1_a2(a1_deg, a2_deg)) return false;

  axes.setMode(AxisGroupController::AxisRunMode::XY_VERTICAL);
  axes.setXGoalMm(kin.getXmm());  // keep X
  axes.setYGoalMm(goal_ymm);
  if (!axes.init()) return false;

  serial_printf_verbose("START move_smooth for MODE_XY_VERTICAL\n");
  serial_printf_verbose("existing x=%.2f goal y=%.2f\n", kin.getXmm(), goal_ymm);
  double prev_speed = speed;
  if (speed > 0.35) speed = 0.35;  //TODOadjust
  bool ret = move_smooth();
  speed = prev_speed;
  read_print_xy_status();
  return ret;
}

bool cmdMoveXmm(double x_mm) {
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2)) return false;

  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  if (!kin.solve_x_y_from_a1_a2(a1_deg, a2_deg)) return false;

  axes.setMode(AxisGroupController::AxisRunMode::XY_HORIZONTAL);
  axes.setXGoalMm(x_mm);
  axes.setYGoalMm(kin.getYmm());  // keep Y

  if (!axes.init()) return false;

  serial_printf_verbose("START move_smooth for MODE_XY_HORIZONTAL\n");
  serial_printf_verbose("goal x=%.2f existing y=%.2f\n", x_mm, kin.getYmm());
  double prev_speed = speed;
  if (speed > 0.25) speed = 0.25;  //TODOadjust
  bool ret = move_smooth();
  speed = prev_speed;
  read_print_xy_status();
  return ret;
}

CubeOri ori(robot_move_callback);
