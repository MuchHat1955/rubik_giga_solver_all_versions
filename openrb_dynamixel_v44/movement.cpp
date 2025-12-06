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

void AxisGroupController::writeTicks(const int* posList) {
  int n = axesCount();
  for (int i = 0; i < n; i++) {
    uint8_t id = getId(i);
    if (id > 0 && posList[i] >= 0) {
      if (!safeSetGoalPosition(id, posList[i])) break;
    }
  }
}

// Compute synchronized tick for slave axis using kinematics (XY) or
// percentage catch-up (gripper).
int AxisGroupController::getSyncGoal(uint8_t slaveIndex, int masterTicks) {
  if (slaveIndex == 0) return masterTicks;  // master itself

  switch (mode) {
    case AxisRunMode::XY_VERTICAL:
    case AxisRunMode::XY_HORIZONTAL:
      {
        // master is arm1 (index 0)
        double a1_deg = ticks2deg(getId(0), masterTicks);

        if (mode == AxisRunMode::XY_VERTICAL) {
          if (!kinPtr->solve_a2_y_from_a1_x(a1_deg, goal_mm_x))
            return curr_ticks[slaveIndex];
        } else {
          if (!kinPtr->solve_a2_x_from_a1_y(a1_deg, goal_mm_y))
            return curr_ticks[slaveIndex];
        }

        if (slaveIndex == 1) return kinPtr->getA2ticks();
        if (slaveIndex == 2) return kinPtr->getGticks_closest_aligned();
        return curr_ticks[slaveIndex];
      }

    case AxisRunMode::GRIPPER:
      {
        // master index 0, slave index 1
        if (slaveIndex != 1) return curr_ticks[slaveIndex];
        int masterTravelLeft = abs(goal_ticks[0] - curr_ticks[0]);
        int slaveTravelTotal = abs(goal_ticks[1] - start_ticks[1]);
        int mBehind = slaveTravelTotal - masterTravelLeft;

        if (mBehind < 0) {
          // slave waits
          return start_ticks[1];
        }

        int slaveGoal = goal_ticks[1] - masterTravelLeft * (int)dir_list[1];
        return slaveGoal;
      }

    default:
      return masterTicks;
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
  // IDs like ID_ARM1, ID_ARM2, ID_WRIST must be defined elsewhere
  if (!dxlPtr->ping(ID_ARM1) || !dxlPtr->ping(ID_ARM2) || !dxlPtr->ping(ID_WRIST)) {
    serial_printf_verbose("[INIT XY] ⚠ Missing servo(s): ping arm1=%d ping arm2=%d ping wrist=%d\n",
                          dxlPtr->ping(ID_ARM1),
                          dxlPtr->ping(ID_ARM2),
                          dxlPtr->ping(ID_WRIST));
    return false;
  }

  id_list[0] = ID_ARM1;
  id_list[1] = ID_ARM2;
  id_list[2] = ID_WRIST;

  start_ticks[0] = dxlPtr->getPresentPosition(ID_ARM1);
  start_ticks[1] = dxlPtr->getPresentPosition(ID_ARM2);
  start_ticks[2] = dxlPtr->getPresentPosition(ID_WRIST);

  double a1_center_deg = ticks2deg(ID_ARM1, start_ticks[0]);
  double a2_center_deg = ticks2deg(ID_ARM2, start_ticks[1]);
  if (!kinPtr->solve_x_y_from_a1_a2(a1_center_deg, a2_center_deg))
    return false;

  double x_now = kinPtr->getXmm();
  double y_now = kinPtr->getYmm();

  double g_vert = kinPtr->getGdeg_for_vertical();
  double g_horiz = kinPtr->getGdeg_for_horizontal();
  double g_present = kinPtr->getGdeg();

  auto norm360 = [](double d) {
    while (d < 0) d += 360.0;
    while (d >= 360) d -= 360.0;
    return d;
  };

  g_vert = norm360(g_vert);
  g_horiz = norm360(g_horiz);
  g_present = norm360(g_present);

  bool is_vert = (std::fabs(g_present - g_vert) < std::fabs(g_present - g_horiz));

  if (keepX) goal_mm_x = x_now;
  else goal_mm_y = y_now;

  serial_printf_verbose("[INIT XY] current XY=(%.2f, %.2f)mm -> goal XY=(%.2f, %.2f)mm, keepX=%d, is_vert=%s\n",
                        x_now, y_now, goal_mm_x, goal_mm_y, keepX,
                        is_vert ? "vert" : "horiz");

  kinPtr->solve_a1_a2_from_x_y(goal_mm_x, goal_mm_y);

  double a1 = kinPtr->getA1deg();
  double a2 = kinPtr->getA2deg();
  double g = is_vert ? kinPtr->getGdeg_for_vertical()
                     : kinPtr->getGdeg_for_horizontal();

  goal_ticks[0] = deg2ticks(ID_ARM1, a1);
  goal_ticks[1] = deg2ticks(ID_ARM2, a2);
  goal_ticks[2] = deg2ticks(ID_WRIST, g);

  nudge_flags[0] = true;   // allow nudging for arm1
  nudge_flags[1] = true;   // allow nudging for arm2
  nudge_flags[2] = false;  // no nudge for wrist

  dir_list[0] = (goal_ticks[0] - start_ticks[0] >= 0) ? 1.0 : -1.0;
  dir_list[1] = (goal_ticks[1] - start_ticks[1] >= 0) ? 1.0 : -1.0;
  dir_list[2] = (goal_ticks[2] - start_ticks[2] >= 0) ? 1.0 : -1.0;

  configured = true;

  serial_printf_verbose("[INIT XY] keep%s | a1=%.2f° a2=%.2f° g=%.2f°\n",
                        keepX ? "X" : "Y", a1, a2, g);
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

  serial_printf("MOVING %s %d/%d\n", moveName, crrTicks, endTicks);
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

// ----------------------------------------------------------------------
// move_smooth_v1  (original step-based + nudge logic)
// ----------------------------------------------------------------------

bool move_smooth_v1() {
  const int axes_count = axes.axesCount();
  if (axes_count == 0) return false;

  axes.start();

  MovePhase currentPhase = MovePhase::ACCEL;

  std::vector<NudgeController*> nudgers;
  nudgers.reserve(axes_count);
  for (int i = 0; i < axes_count; i++) {
    nudgers.push_back(axes.getNudgeController(i));
  }

  int samePosCount[3] = { 0, 0, 0 };
  int startTicks[3] = { 0, 0, 0 };
  int currTicks[3] = { 0, 0, 0 };
  int prevTicks[3] = { 0, 0, 0 };
  int errTicks[3] = { 0, 0, 0 };
  int dirTicks[3] = { 1, 1, 1 };
  int prevGoalTicks[3] = { 0, 0, 0 };
  int nextGoalTicks[3] = { 0, 0, 0 };
  int goalTicks[3] = { 0, 0, 0 };
  int correctionTicks[3] = { 0, 0, 0 };
  bool axesDone[3] = { false, false, false };

  axes.readPresentTicks(currTicks);
  serial_printf_verbose("start move | axes read current ticks={%d,%d,%d}\n",
                        currTicks[0], currTicks[1], currTicks[2]);
  serial_printf_verbose("start move | axes goal ticks={%d,%d,%d}\n",
                        axes.getGoalTicks(0), axes.getGoalTicks(1), axes.getGoalTicks(2));

  for (int i = 0; i < axes_count; i++) {
    samePosCount[i] = 0;
    startTicks[i] = currTicks[i];
    prevTicks[i] = currTicks[i];
    prevGoalTicks[i] = currTicks[i];
    errTicks[i] = 0;
    nextGoalTicks[i] = currTicks[i];
    goalTicks[i] = axes.getGoalTicks(i);
    axesDone[i] = false;
    correctionTicks[i] = 0;
    dirTicks[i] = (goalTicks[i] >= startTicks[i]) ? 1 : -1;

    uint8_t id = axes.getId(i);
    if (goalTicks[i] > getMax_ticks(id) - 50)
      goalTicks[i] = getMax_ticks(id) - 50;
    if (goalTicks[i] < getMin_ticks(id) + 50)
      goalTicks[i] = getMin_ticks(id) + 50;
  }

  int masterStart = startTicks[0];
  int masterGoal = goalTicks[0];
  int dist = masterGoal - masterStart;
  int dirMaster = (dist >= 0) ? 1 : -1;
  int tol_ticks = 4;

  int total_steps = abs(dist);
  if (total_steps < tol_ticks) {
    serial_printf_verbose("end move | already in position dist=%d\n", dist);
    axes.end();
    return true;
  }

  bool inFinalPosition = false;

  // These constants must exist somewhere or you can inline them here
  const int stepInterval_ms = 25;
  const int accelSteps = 10;
  const int decelSteps = 10;
  const int minStep_ticks = 1;
  const int maxStep_ticks = 8;

  int accelSpan = accelSteps * (minStep_ticks + maxStep_ticks) / 2;
  int decelSpan = decelSteps * (minStep_ticks + maxStep_ticks) / 2;
  int coastSpan = total_steps - (accelSpan + decelSpan);
  if (coastSpan < 0) coastSpan = 0;
  int coastSteps = (maxStep_ticks > 0) ? (coastSpan / maxStep_ticks) : 0;

  serial_printf_verbose("start move | start=%d goal=%d total_steps=%d accelSteps=%d coastSteps=%d decelSteps=%d\n\n",
                        masterStart, masterGoal, masterGoal - masterStart,
                        accelSteps, coastSteps, decelSteps);

  bool all_good = false;
  bool skip2final = false;
  int curr_step = 0;

  auto step_axes = [&](const char* phaseName, int iter, int step_ticks) {
    axes.readPresentTicks(currTicks);

    for (int ax = 0; ax < axes.axesCount(); ax++) {
      if (axesDone[ax]) {
        samePosCount[ax] = 0;
        prevGoalTicks[ax] = goalTicks[ax];
        errTicks[ax] = 0;
        nextGoalTicks[ax] = goalTicks[ax];
        continue;
      }

      errTicks[ax] = prevGoalTicks[ax] - currTicks[ax];
      if (prevGoalTicks[ax] < 0)
        errTicks[ax] = 0;

      if (abs(currTicks[ax] - goalTicks[ax]) < tol_ticks)
        axesDone[ax] = true;

      if (axesDone[ax]) {
        nextGoalTicks[ax] = goalTicks[ax];
        continue;
      }

      if (abs(prevTicks[ax] - currTicks[ax]) < 2)
        samePosCount[ax]++;
      else
        samePosCount[ax] = 0;

      samePosCount[ax] = std::min(samePosCount[ax], 6);
      prevTicks[ax] = currTicks[ax];

      correctionTicks[ax] = 0;
      if (axes.getNudgeFlag(ax) && nudgers[ax]) {
        correctionTicks[ax] = nudgers[ax]->computeNudge(errTicks[ax],
                                                        dirTicks[ax],
                                                        currentPhase,
                                                        samePosCount[ax]);
      }

      logProgress(axes.getMoveName(), startTicks[0], currTicks[0], goalTicks[0]);
      curr_step += step_ticks;

      bool use_final_goal = false;

      if (ax == 0) {
        if (currentPhase == MovePhase::FINAL) {
          if (step_ticks == 0) {
            use_final_goal = true;
            nextGoalTicks[0] = goalTicks[0] + correctionTicks[0];
          } else {
            nextGoalTicks[0] = currTicks[0] + correctionTicks[0];
          }
        } else {
          nextGoalTicks[0] += dirMaster * step_ticks;

          int err_final = abs(currTicks[0] - goalTicks[0]);
          if ((dirMaster >= 0 && nextGoalTicks[0] > (goalTicks[0] + tol_ticks)) || (dirMaster < 0 && nextGoalTicks[0] < (goalTicks[0] - tol_ticks)) || err_final < tol_ticks) {
            nextGoalTicks[0] = goalTicks[0];
            skip2final = true;
          }
        }
      } else {
        nextGoalTicks[ax] = axes.getSyncGoal(ax, nextGoalTicks[0]);
        nextGoalTicks[ax] += correctionTicks[ax];
      }

      serial_printf_verbose("[%s] axis=%d prev_goal=%d curr=%d err=%d next=%d corr=%d same=%d step=%d x=%.2f y=%.2f skip=%d use_final=%d\n",
                            phaseName, ax, prevGoalTicks[ax], currTicks[ax],
                            errTicks[ax], nextGoalTicks[ax], correctionTicks[ax],
                            samePosCount[ax], step_ticks,
                            kin.getXmm(), kin.getYmm(), skip2final, use_final_goal);
      prevGoalTicks[ax] = nextGoalTicks[ax];
    }

    all_good = true;
    for (int i = 0; i < axes.axesCount(); i++) {
      if (!axesDone[i]) {
        all_good = false;
        break;
      }
    }

    if (!all_good) {
      int goalsWithSkipDone[3] = { -1, -1, -1 };
      for (int i = 0; i < axes.axesCount(); i++) {
        goalsWithSkipDone[i] = axesDone[i] ? -1 : nextGoalTicks[i];
      }
      axes.writeTicks(goalsWithSkipDone);
      delay(stepInterval_ms);
    }
  };

  // Accel
  currentPhase = MovePhase::ACCEL;
  for (int i = 0; i < accelSteps; i++) {
    if (all_good || skip2final) break;
    int step_ticks = map(i, 0, accelSteps - 1, minStep_ticks, maxStep_ticks);
    step_axes("ACCEL", i, step_ticks);
  }

  // Coast
  currentPhase = MovePhase::COAST;
  for (int i = 0; i < coastSteps; i++) {
    if (all_good || skip2final) break;
    step_axes("COAST", i, maxStep_ticks);
  }

  // Decel
  currentPhase = MovePhase::DECEL;
  for (int i = 0; i < decelSteps; i++) {
    if (all_good || skip2final) break;
    int step_ticks = map(decelSteps - 1 - i, 0, decelSteps - 1,
                         minStep_ticks, maxStep_ticks);
    step_axes("DECEL", i, step_ticks);
  }

  delay(2 * stepInterval_ms);

  // Final
  currentPhase = MovePhase::FINAL;
  const int maxNudges = 8;
  const int nudgeExtraDelay = 65;

  for (int nu = 0; nu < maxNudges; nu++) {
    if (all_good) break;
    int step_ticks = (nu < ((maxNudges - 1) / 2)) ? 1 : 0;
    step_axes("FINAL", nu, step_ticks);
    delay(nudgeExtraDelay);
  }

  serial_printf_verbose("end move | master err=%d\n", errTicks[0]);
  axes.end();
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

      delay(5);
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
        delay(50);
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

  serial_printf(
    "x_mm=%.2f y_mm=%.2f a1_deg=%.2f a2_deg=%.2f g_vert_deg=%.2f g_deg=%.2f g1_per=%.2f g2_per=%.2f base_deg=%.2f\n",
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

  serial_printf_verbose("START move_smooth for SINGLE_SERVO\n");
  return move_smooth();
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
    const double EXTRA = 1.5;

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

bool robot_move_callback(const String& mv) {
  serial_printf_verbose("HW executes: %s\n", mv.c_str());
  return true;
}

CubeOri ori(robot_move_callback);
