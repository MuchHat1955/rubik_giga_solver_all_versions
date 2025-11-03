#include <vector>
#include <map>
#include <Arduino.h>
#include <algorithm>
#include "movement.h"
#include "Dynamixel2Arduino.h"
#include "vertical_kinematics.h"

// Forward decls & globals from main
class AxisGroupController;  // forward
extern bool verboseOn;
extern Dynamixel2Arduino dxl;
extern VerticalKinematics kin;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~START AXIS CONTROLLER~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ====================================================================================
// AxisGroupController
// ------------------------------------------------------------------------------------
//  Hardware / kinematics abstraction for the motion kernel.
//  Supports four logical modes:
//    1. Single servo (degree goal)
//    2. XY vertical   (keep X fixed)
//    3. XY horizontal (keep Y fixed)
//    4. Gripper open/close (percent goal, smart master/slave catch-up)
// ====================================================================================

enum AxisRunMode : uint8_t {
  MODE_UNDEFINED = 0,
  MODE_SINGLE_SERVO = 1,
  MODE_XY_VERTICAL = 2,
  MODE_XY_HORIZONTAL = 3,
  MODE_GRIPPER = 4
};

class AxisGroupController {
public:
  AxisGroupController(Dynamixel2Arduino* dxl_ptr,
                      VerticalKinematics* kin_ptr)
    : dxl(dxl_ptr), kin(kin_ptr) {}

  // ------------------------------------------------------------
  //               Mode & configuration setup
  // ------------------------------------------------------------
  void setMode(AxisRunMode m) {
    mode = m;
    // Reset IDs/goals
    goal_deg = goal_mm_x = goal_mm_y = goal_percent = 0.0;
    start_ticks.clear();
    goal_ticks.clear();
    curr_ticks.clear();
    nudge_list.clear();
    nudge_flags.clear();
    id_list.clear();
    configured = false;
    grip_lastProgress = 0.0;
  }

  // For MODE_SINGLE_SERVO
  void setServoId(uint8_t id) {
    id_servo = id;
  }
  void setGoalMasterDeg(double deg) {
    goal_deg = deg;
  }
  void setXGoalMm(double x_mm) {
    goal_mm_x = x_mm;
  }
  void setYGoalMm(double y_mm) {
    goal_mm_y = y_mm;
  }
  void setServoGoalPercent(double per) {
    goal_percent = per;
  }

  // ------------------------------------------------------------
  //                      Initialization
  // ------------------------------------------------------------
  bool init() {
    switch (mode) {
      case MODE_SINGLE_SERVO: return initSingle();
      case MODE_XY_VERTICAL: return initXY(true);     // keep X
      case MODE_XY_HORIZONTAL: return initXY(false);  // keep Y
      case MODE_GRIPPER: return initGripper();
      default: return false;
    }
  }

  // ------------------------------------------------------------
  //                  Access for motion kernel
  // ------------------------------------------------------------
  int getAxisCount() const {
    switch (mode) {
      case MODE_SINGLE_SERVO: return 1;
      case MODE_GRIPPER: return 2;
      case MODE_XY_VERTICAL:
      case MODE_XY_HORIZONTAL: return 3;
      default: return 0;
    }
  }

  int getId(uint8_t index) const {
    if (index >= id_list.size()) return 0;
    return id_list[index];
  }
  int getGoalTicks(uint8_t index) const {
    if (index >= goal_ticks.size()) return 0;
    return goal_ticks[index];
  }

  bool getNudgeFlag(uint8_t index) const {
    if (index >= nudge_flags.size()) return false;
    return nudge_flags[index];
  }

  NudgeController* getNudgeController(uint8_t index) {
    if (index >= nudge_list.size()) return nullptr;
    return &nudge_list[index];
  }

  void start() {
    // optional LED on or torque relax
    for (uint8_t i = 0; i < getAxisCount(); i++) {
      uint8_t id = getId(i);
      if (dxl->ping(id)) dxl->ledOn(id);
    }
  }

  void end() {
    // optional LED off or torque relax
    for (uint8_t i = 0; i < getAxisCount(); i++) {
      uint8_t id = getId(i);
      if (dxl->ping(id)) dxl->ledOff(id);
    }
  }

  void readPositions(int* posList) {
    for (uint8_t i = 0; i < getAxisCount(); i++) {
      uint8_t id = getId(i);
      if (id > 0) posList[i] = dxl->getPresentPosition(id);
    }
  }
  void applyPos(int* posList) {
    for (uint8_t i = 0; i < getAxisCount(); i++) {
      uint8_t id = getId(i);
      if (id > 0 && posList[i] >= 0) dxl->writeControlTableItem(ControlTableItem::GOAL_POSITION, id, posList[i]);
    }
  }
  // Compute synchronized tick for slave axis using kinematics (XY) or % catch-up (gripper)
  int getSyncGoal(uint8_t slaveIndex, int masterTicks) const {
    if (slaveIndex == 0) return masterTicks;  // master

    switch (mode) {
      case MODE_XY_VERTICAL:
      case MODE_XY_HORIZONTAL:
        {
          // master is arm1 (index 0)
          VerticalKinematics* k = kin;

          if (mode == MODE_XY_VERTICAL) {
            double _a1_center_deg = ticks2deg(ID_ARM1, masterTicks);
            if (!const_cast<VerticalKinematics*>(k)->solve_a2_y_from_a1_x(_a1_center_deg, goal_mm_x)) return 0;
          }
          if (mode == MODE_XY_HORIZONTAL) {
            double _a1_center_deg = ticks2deg(ID_ARM1, masterTicks);
            if (!const_cast<VerticalKinematics*>(k)->solve_a2_x_from_a1_y(_a1_center_deg, goal_mm_y)) return 0;
          }
          if (slaveIndex == 1) return const_cast<VerticalKinematics*>(k)->getA2ticks();
          if (slaveIndex == 2) return const_cast<VerticalKinematics*>(k)->getGticks_closest_aligned();  // keep existing position
          return 0;
        }

      case MODE_GRIPPER:
        {
          // Smart catch-up: slave waits until master's % travel >= slave's, then syncs
          const int mStart = start_ticks[0];
          const int mGoal = goal_ticks[0];
          const int mTravel = abs(mGoal - mStart);
          if (mTravel == 0) return start_ticks[slaveIndex];

          // Master progress in [0..1]
          double mProg = (double)abs(masterTicks - mStart) / (double)mTravel;
          mProg = constrain(mProg, 0.0, 1.0);

          const int sStart = start_ticks[slaveIndex];
          const int sGoal = goal_ticks[slaveIndex];
          const int sTravel = abs(sGoal - sStart);

          // Monotonic progress
          if (mProg < grip_lastProgress) mProg = grip_lastProgress;

          int newGoal;
          if (mProg * (double)mTravel < (double)sTravel) {
            newGoal = sStart;  // hold until comparable % covered
          } else {
            newGoal = (int)lround(sStart + (sGoal - sStart) * mProg);
          }

          const_cast<AxisGroupController*>(this)->grip_lastProgress = mProg;
          return newGoal;
        }

      default:
        return 0;
    }
  }

private:
  // ------------------------------------------------------------
  //            Internal initialization helpers
  // ------------------------------------------------------------
  bool initSingle() {
    if (!dxl->ping(id_master)) {
      if (verboseOn)
        serial_printf("[INIT SINGLE] ⚠ Servo %d not responding\n", id_master);
      return false;
    }

    start_ticks_master = dxl->getPresentPosition(0);
    goal_ticks_master = deg2ticks(id_master, goal_deg);
    goal_ticks = { goal_ticks_master };
    start_ticks = { start_ticks_master };
    nudge_list.resize(1);
    nudge_flags = { false };  // no mid-nudging for single
    configured = true;

    if (verboseOn) {
      serial_printf("[INIT SINGLE] id=%d start=%d goal_deg=%.2f goal_ticks=%d\n",
                    id_master, start_ticks_master, goal_deg, goal_ticks_master);
      serial_printf("[INIT SINGLE] Δticks=%d (%.2f deg)\n",
                    goal_ticks_master - start_ticks_master, goal_deg);
    }
    return true;
  }

  bool initXY(bool keepX) {
    if (!dxl->ping(ID_ARM1) || !dxl->ping(ID_ARM2) || !dxl->ping(ID_WRIST)) {
      if (verboseOn)
        serial_printf("[INIT XY] ⚠ Missing servo(s): ping arm1=%d ping arm2=%d ping grip=%d\n",
                      dxl->ping(ID_ARM1), dxl->ping(ID_ARM2), dxl->ping(ID_WRIST));
      return false;
    }

    // just in case
    id_list[0] = ID_ARM1;
    id_list[1] = ID_ARM2;
    id_list[2] = ID_WRIST;

    start_ticks[0] = dxl->getPresentPosition(ID_ARM1);
    start_ticks[1] = dxl->getPresentPosition(ID_ARM2);
    start_ticks[2] = dxl->getPresentPosition(ID_WRIST);

    double _a1_center_deg = ticks2deg(ID_ARM1, dxl->getPresentPosition(ID_ARM1));
    double _a2_center_deg = ticks2deg(ID_ARM2, dxl->getPresentPosition(ID_ARM2));
    if (!kin->solve_x_y_from_a1_a2(_a1_center_deg, _a2_center_deg)) return false;

    double x_now = kin->getXmm();
    double y_now = kin->getYmm();

    if (verboseOn)
      serial_printf("[INIT XY] Current XY=(%.2f, %.2f)mm  keepX=%d\n",
                    x_now, y_now, keepX);

    if (keepX) kin->solve_a1_a2_from_x_y(x_now, goal_mm_y);
    else kin->solve_a1_a2_from_x_y(goal_mm_x, y_now);

    double a1 = kin->getA1deg();
    double a2 = kin->getA2deg();
    double g = kin->getGdeg_closest_aligned();


    goal_ticks[0] = deg2ticks(ID_ARM1, a1);
    goal_ticks[1] = deg2ticks(ID_ARM2, a2);
    goal_ticks[2] = deg2ticks(ID_WRIST, g);

    nudge_flags = { true, true, false };  // mid-nudge for arm1 and arm2, not gripper
    nudge_list.resize(3);
    configured = true;

    if (verboseOn) {
      serial_printf("[INIT XY] keep%s | a1=%.2f° a2=%.2f° g=%.2f°\n",
                    keepX ? "X" : "Y", a1, a2, g);
      serial_printf("[INIT XY] arm1 start=%d goal=%d  Δ=%d\n",
                    start_ticks[0], goal_ticks[0],
                    goal_ticks[0] - start_ticks[0]);
      serial_printf("[INIT XY] arm2 start=%d goal=%d  Δ=%d\n",
                    start_ticks[1], goal_ticks[1],
                    goal_ticks[1] - start_ticks[1]);
      serial_printf("[INIT XY] grip start=%d goal=%d  Δ=%d\n",
                    start_ticks[2], goal_ticks[2],
                    goal_ticks[2] - start_ticks[2]);
    }
    return true;
  }

  bool initGripper() {
    bool ok1 = dxl->ping(ID_GRIP1);
    bool ok2 = dxl->ping(ID_GRIP2);
    if (!ok1 || !ok2) {
      if (verboseOn)
        serial_printf("[INIT GRIP] ⚠ Gripper ping failed ping g1=%d ping g2=%d\n", ok1, ok2);
      return false;
    }

    // Read starts
    int start1 = dxl->getPresentPosition(ID_GRIP1);
    int start2 = dxl->getPresentPosition(ID_GRIP2);

    // Compute tick goals from % for each jaw
    int goal1 = per2ticks(ID_GRIP1, goal_percent);
    int goal2 = per2ticks(ID_GRIP2, goal_percent);

    int travel1 = abs(goal1 - start1);
    int travel2 = abs(goal2 - start2);

    // Pick master = larger travel; slave waits until % matches
    // just in case ignore the setId
    if (travel1 >= travel2) {
      id_list = { ID_GRIP1, ID_GRIP2, 0 };
      start_ticks = { start1, start2, -1 };
      goal_list = { goal1, goal2, -1 };
    } else {
      id_list = { ID_GRIP2, ID_GRIP1, 0 };
      start_ticks = { start2, start1, -1 };
      goal_list = { goal2, goal1, -1 };
    }

    nudge_flags = { false, false };  // typically no mid-nudge for gripper
    nudge_list.resize(2);
    configured = true;
    grip_lastProgress = 0.0;

    if (verboseOn) {
      serial_printf("[INIT GRIP] goal%%=%.1f start1=%d start2=%d goal1=%d goal2=%d\n",
                    goal_percent, start1, start2, goal1, goal2);
      serial_printf("[INIT GRIP] masterID=%d slaveID=%d travel1=%d travel2=%d\n",
                    id_master, id_slave, travel1, travel2);
    }
    return true;
  }

private:
  Dynamixel2Arduino* dxl;
  VerticalKinematics* kin;
  AxisRunMode mode = MODE_UNDEFINED;
  bool configured = false;

  // Servo IDs
  uint8_t id_servo = 0;

  // Goals (varies by mode)
  double goal_deg = 0.0;
  double goal_mm_x = 0.0;
  double goal_mm_y = 0.0;
  double goal_percent = 0.0;

  // Cached ticks
  std::vector<int> start_ticks;
  std::vector<int> goal_ticks;
  std::vector<int> curr_ticks;
  std::vector<int> id_list;

  // Nudging
  std::vector<NudgeController> nudge_list;
  std::vector<bool> nudge_flags;

  // Gripper progress memory (for monotonic sync)
  double grip_lastProgress = 0.0;
};

// Instantiate the global controller AFTER class definition
AxisGroupController axes(&dxl, &kin);

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~START NUDGE CONTROLLER~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ====================================================================================
//          NudgeController Implementation
// ------------------------------------------------------------------------------------

void NudgeController::recordData(int prevGoal, int currPos, int nudge, MovePhase phase) {
  Record r;
  r.t_ms = millis();
  r.prevGoal = prevGoal;
  r.currPos = currPos;
  r.err = prevGoal - currPos;
  r.nudgeApplied = nudge;
  r.phase = phase;

  if (records.size() >= maxRecords)
    records.erase(records.begin());
  records.push_back(r);
}

int NudgeController::computeNudge(int currErr, MovePhase phase, int samePosCount) {

  if (abs(currErr) <= 4) return 0;  //TODO
  // If no history, fall back to simple proportional estimate

  // if (records.empty()) // TODO use just the base
  return baseEstimate(currErr, phase, samePosCount);

  double sumErr = 0, sumNudge = 0, weightSum = 0;
  for (int i = records.size() - 1; i >= 0; i--) {
    if (records[i].phase != phase) continue;
    double w = 1.0 + 0.02 * (i + 1);  // more recent samples weighted higher
    sumErr += w * records[i].err;
    sumNudge += w * records[i].nudgeApplied;
    weightSum += w;
  }

  double avgErr = (weightSum > 0) ? (sumErr / weightSum) : currErr;
  double avgNudge = (weightSum > 0) ? (sumNudge / weightSum) : 0.0;

  double kPhase = phaseGain(phase);
  double pred = kPhase * avgErr + 0.3 * avgNudge;

  // In final phase, add same-position reinforcement if stuck
  if (phase == MovePhase::FINAL && samePosCount > 0)
    pred += samePosCount * 0.5 * (currErr > 0 ? 1 : -1);

  int nudge = (int)constrain(pred, -20.0, 20.0);
  return nudge;
}

void NudgeController::printLog() {
  serial_printf("---- Nudge log for servo %d (count=%d) ----\n",
                id, (int)records.size());
  for (auto& r : records) {
    const char* phaseStr =
      (r.phase == MovePhase::ACCEL) ? "ACC" : (r.phase == MovePhase::COAST) ? "COAST"
                                            : (r.phase == MovePhase::DECEL) ? "DEC"
                                                                            : "FINAL";
    serial_printf("[%lu ms] %s goal=%d curr=%d err=%d nudge=%d\n",
                  r.t_ms, phaseStr, r.prevGoal, r.currPos, r.err, r.nudgeApplied);
  }
}

int NudgeController::baseEstimate(int err, MovePhase phase, int samePosCount) {
  // TODO if err is negate -> pos nudge
  double k = phaseGain(phase);
  double nudge = k * -err;
  if (phase == MovePhase::FINAL && samePosCount > 0)
    nudge += samePosCount * 2 * (err > 0 ? -1 : 1);  //TODO adjust
  nudge = constrain(nudge, -35.0, 35.0);
  serial_printf("   --nudge for err=%d is %d", err, nudge);
  return nudge;
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

// static controller map per servo
std::map<uint8_t, NudgeController> nudgeDB;
inline NudgeController& getNudgeController(uint8_t id) {
  if (nudgeDB.find(id) == nudgeDB.end())
    nudgeDB[id] = NudgeController(id);
  return nudgeDB[id];
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~START SMOOTH MOVE USING THE AXIS CONTROLLER~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ============================================================
// Default motion-profile parameters for move_smooth()
// ============================================================

// Time between incremental servo goal updates (ms)
#define SMOOTH_STEP_INTERVAL_MS 35
// Number of acceleration steps before reaching full speed
#define SMOOTH_ACCEL_STEPS 20
// Number of deceleration steps before stopping
#define SMOOTH_DECEL_STEPS 20
// Minimum tick change per update (slowest motion increment)
#define SMOOTH_MIN_STEP_TICKS 1
// Maximum tick change per update (fastest motion increment)
#define SMOOTH_MAX_STEP_TICKS 25
// Tolerance in ticks for considering goal reached
#define SMOOTH_TOL_TICKS 4

// ====================================================================================
//                 move_smooth()
//  - trajectory and timing kernel (geometry-agnostic)
//  - asks global AxisGroupController for all tick-space I/O
// ------------------------------------------------------------------------------------

bool move_smooth(
  int stepInterval_ms = SMOOTH_STEP_INTERVAL_MS,
  int accelSteps = SMOOTH_ACCEL_STEPS,
  int decelSteps = SMOOTH_DECEL_STEPS,
  int minStep_ticks = SMOOTH_MIN_STEP_TICKS,
  int maxStep_ticks = SMOOTH_MAX_STEP_TICKS,
  int tol_ticks = SMOOTH_TOL_TICKS) {
  const int n = axes.getAxisCount();
  if (n == 0) return false;

  axes.start();  // prepares hardware (reads starts, etc.)

  MovePhase currentPhase = MovePhase::ACCEL;

  // TODO adjust torque

  // per-axis nudgers
  std::vector<NudgeController*> nudgers;
  nudgers.reserve(n);
  for (int i = 0; i < n; i++)
    nudgers.push_back(axes.getNudgeController(i));

  int samePosCount[3];
  int startPos[3];
  int crrPos[3];
  int prevPos[3];
  int err[3];
  int prevGoal[3];
  int nextGoal[3];
  int finalGoal[3];
  bool axesDone[3];

  for (int i = 0; i < n; i++) {
    samePosCount[i] = 0;
    startPos[i] = axes.getCurrTicks(i);
    prevPos[i] = axes.getCurrTicks(i);
    prevGoal[i] = -1;
    crrPos[i] = -1;
    err[i] = 0;
    nextGoal[i] = -1;
    finalGoal[i] = axes.getGoalTicks(i);
    axesDone[i] = false;
  }

  // Gather master trajectory info
  int masterStart = startPos[0];
  int masterGoal = finalGoal[0];
  int dist = masterGoal - masterStart;
  int dir = (dist >= 0) ? 1 : -1;
  int totalDiff = abs(dist);
  if (totalDiff < 3) return true;

  bool inFinalPosition = false;

  int accelSpan = accelSteps * (minStep_ticks + maxStep_ticks) / 2;
  int decelSpan = decelSteps * (minStep_ticks + maxStep_ticks) / 2;
  int coastSpan = totalDiff - (accelSpan + decelSpan);
  if (coastSpan < 0) coastSpan = 0;
  int coastSteps = coastSpan / maxStep_ticks;

  if (verboseOn)
    serial_printf("MOVE start=%d goal=%d totalDiff=%d accel=%d coast=%d decel=%d\n",
                  masterStart, masterGoal, totalDiff, accelSteps, coastSteps, decelSteps);

  bool all_good = false;

  // Helper: one phase iteration
  auto step_axes = [&](const char* phaseName, int iter, int step_ticks) {
    //////////////////////////////
    // feedback & nudging
    axes.readPositions(crrPos);

    //////////////////////////////
    // feedback & nudging
    for (int i = 0; i < axesCount; i++) {
      if (i < 0) continue;  // axes noot used
      if (axesDone[i]) {
        samePosCount[i] = 0;
        prevGoal[i] = -1;
        err[i] = 0;
        nextGoal[i] = -1;
        continue;
      }

      // compute the error vs last goal
      int err[i] = prevGoal[i] - crrPos[i];
      if (prevGoal[i] < 0) err = 0;  // first iteration
      int delta = abs(crrPos[i] - prevPos[i]);
      if (abs(curr - finalGoal[i]) < tol_ticks) axesDone[i] = true;

      if (axesDone[i]) {
        newGoal[i] = -1;
        continue;
      }

      // compute now the nudge
      if (delta < 2) samePosCount[i]++;
      else samePosCount[i] = 0;
      samePosCount[i] = std::min(samePosCount[i], 6);
      prevPos[i] = curr;
      int correction = 0;
      if (axes.getNudgeFlag(i)) correction = nudgers[i]->computeNudge(err, currentPhase, samePosCount[i]);

      // compute the new goal
      if (i == 0) {
        if (step_ticks == 0) nextGoal[0] = finalGoal[0];
        nextGoal[0] += dir * step_ticks;
      } else {
        if (step_ticks == 0) nextGoal[i] = finalGoal[i];
        nextGoal[i] = axes.getSyncGoal(i, posMaster);
      }
      nextGoal[i] += correction;

      // do not apply to master in case it reached end goal, but can apply to slaves
      serial_printf("[%s] READ axis=%d curr=%d prev goal=%d err=%d new goal=%d same=%d\n",
                    phaseName, i, crrPos[i], prevGoal[i], err[i], newGoal[i], samePosCount[i]);
      prevGoal[i] = newGoal;
    }

    all_done = true;
    for (int i = 0; i < axesCount; i++) {
      if (!axesDone[i]) all_done = false;
    }
    if (!all_done) {
      axes.applyPos(newGoal);
      delay(stepInterval_ms);
    }
  };

  // 1. Acceleration phase
  currentPhase = MovePhase::ACCEL;
  for (int i = 0; i < accelSteps; i++) {
    if (all_good) break;
    int step_ticks = map(i, 0, accelSteps - 1, minStep_ticks, maxStep_ticks);
    step_axes("ACCEL", i, step_ticks);
  }

  // 2. Coast phase
  currentPhase = MovePhase::COAST;
  for (int i = 0; i < coastSteps; i++)
    if (all_good) break;
  step_axes("COAST", i, maxStep_ticks);

  // 3. Deceleration phase
  currentPhase = MovePhase::DECEL;
  for (int i = decelSteps - 1; i >= 0; i--) {
    if (all_good) break;
    int step_ticks = map(i, 0, decelSteps - 1, minStep_ticks, maxStep_ticks);
    step_axes("DECEL", i, step_ticks);
  }

  // 4. Final nudging & settle
  currentPhase = MovePhase::FINAL;
  const int maxNudges = 6;
  const int nudgeExtraDelay = 65;

  for (int count = 0; count < maxNudges; count++) {
    if (all_good) break;
    step_axes("FINAL", i, 0);  // step is zero
    delay(nudgeExtraDelay);    // extra delay
  }
  axes.end();
  return true;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~START WRAPPERS FOR COMMANDS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

bool cmdMoveServoDeg(uint8_t id, double goal_deg) {
  if (!dxl.ping(id)) return false;

  axes.setMode(MODE_SINGLE_SERVO);
  axes.setServoId(id);
  axes.setGoalMasterDeg(goal_deg);
  if (!axes.init()) return false;

  if (verboseOn) serial_printf("START move_smooth for SINGLE_SERVO\n");
  return move_smooth();
}

bool cmdMoveGripperPer(double goal_per) {
  if (!dxl.ping(ID_GRIP1) || !dxl.ping(ID_GRIP2)) return false;

  axes.setMode(MODE_GRIPPER);
  axes.setGripperIds(ID_GRIP1, ID_GRIP2);
  axes.setGoalMasterPercent(goal_per);
  if (!axes.init()) return false;

  if (verboseOn) serial_printf("START move_smooth for MODE_GRIPPER\n");
  return move_smooth();
}

bool cmdMoveYmm(double goal_ymm) {
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2)) return false;

  double _a1_center_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double _a2_center_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  if (!kin.solve_x_y_from_a1_a2(_a1_center_deg, _a2_center_deg)) return false;

  axes.setMode(MODE_XY_VERTICAL);
  axes.setServoIds(ID_ARM1, ID_ARM2, ID_WRIST);
  axes.setYGoalMm(kin.getXmm());  // use existing x
  axes.setYGoalMm(goal_ymm);
  if (!axes.init()) return false;

  if (verboseOn) serial_printf("START move_smooth for MODE_XY_VERTICAL\n");
  return move_smooth();
}

bool cmdMoveXmm(double x_mm) {
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2)) return false;

  double _a1_center_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double _a2_center_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  if (!kin.solve_x_y_from_a1_a2(_a1_center_deg, _a2_center_deg)) return false;

  axes.setMode(MODE_XY_HORIZONTAL);
  axes.setServoIds(ID_ARM1, ID_ARM2, ID_WRIST);
  axes.setXGoalMm(x_mm);
  axes.setYGoalMm(kin.getYmm());  // use existing y
  if (!axes.init()) return false;

  if (verboseOn) serial_printf("START move_smooth for MODE_XY_HORIZONTAL\n");
  return move_smooth();
}
