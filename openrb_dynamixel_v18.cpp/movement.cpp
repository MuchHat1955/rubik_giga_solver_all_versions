#include <vector>
#include <map>
#include <Arduino.h>
#include <algorithm>
#include "movement.h"

// Line below uses a global from main file
extern bool verboseOn;

// ============================================================
//              Unified Smooth Move Module v4
// ============================================================
//  - samePos-based nudging + per-servo NudgeController
//  - persistent prediction history (per servo ID)
//  - logs phase data: accel / coast / decel / final
//  - tick-based synchronization + kin G alignment
// ============================================================

extern Dynamixel2Arduino dxl;
extern bool verboseOn;
extern VerticalKinematics kin;

// ------------------------------------------------------------
// Default profile configuration
// ------------------------------------------------------------
#define SMOOTH_STEP_INTERVAL_MS 15
#define SMOOTH_ACCEL_STEPS      20
#define SMOOTH_DECEL_STEPS      20
#define SMOOTH_MIN_STEP_TICKS   1
#define SMOOTH_MAX_STEP_TICKS   25
#define SMOOTH_TOL_TICKS        4

// ------------------------------------------------------------
// Conversion helpers
// ------------------------------------------------------------
inline int mm2ticks(double mm)   { return (int)round(mm / MM_PER_TICK); }
inline int deg2ticks(double deg) { return (int)round(deg / DEG_PER_TICK); }
inline int per2ticks(double per) { return (int)round(per * (4095.0 / 100.0)); }

// ============================================================
//                 Nudge Controller Class
// ============================================================

enum class MovePhase : uint8_t { ACCEL, COAST, DECEL, FINAL };

class NudgeController {
public:
  struct Record {
    unsigned long t_ms;
    int prevGoal;
    int currPos;
    int err;
    int nudgeApplied;
    MovePhase phase;
  };

  NudgeController(uint8_t servoId = 0) : id(servoId) {}

  void recordData(int prevGoal, int currPos, int nudge, MovePhase phase) {
    Record r;
    r.t_ms = millis();
    r.prevGoal = prevGoal;
    r.currPos = currPos;
    r.err = prevGoal - currPos;
    r.nudgeApplied = nudge;
    r.phase = phase;
    if (records.size() >= maxRecords) records.erase(records.begin());
    records.push_back(r);
  }

  int computeNudge(int currErr, MovePhase phase, int samePosCount) {
    if (records.empty()) return baseEstimate(currErr, phase, samePosCount);

    double sumErr = 0, sumNudge = 0, weightSum = 0;
    for (int i = records.size() - 1; i >= 0; i--) {
      if (records[i].phase != phase) continue;
      double w = 1.0 + 0.02 * (i + 1);
      sumErr += w * records[i].err;
      sumNudge += w * records[i].nudgeApplied;
      weightSum += w;
    }

    double avgErr = (weightSum > 0) ? sumErr / weightSum : currErr;
    double avgNudge = (weightSum > 0) ? sumNudge / weightSum : 0;
    double kPhase = phaseGain(phase);
    double pred = kPhase * avgErr + 0.3 * avgNudge;

    if (phase == MovePhase::FINAL && samePosCount > 0)
      pred += samePosCount * 0.5 * (currErr > 0 ? 1 : -1);

    return (int)constrain(pred, -20.0, 20.0);
  }

  void printLog() {
    serial_printf(
"---- Nudge log for servo %d (count=%d) ----\n", id, (int)records.size());
    for (auto &r : records) {
      const char *phaseStr =
          (r.phase == MovePhase::ACCEL) ? "ACC" :
          (r.phase == MovePhase::COAST) ? "COAST" :
          (r.phase == MovePhase::DECEL) ? "DEC" : "FINAL";
      serial_printf(
"[%lu ms] %s goal=%d curr=%d err=%d nudge=%d\n",
                    r.t_ms, phaseStr, r.prevGoal, r.currPos, r.err, r.nudgeApplied);
    }
  }

private:
  uint8_t id;
  std::vector<Record> records;
  size_t maxRecords = 100;

  int baseEstimate(int err, MovePhase phase, int samePosCount) {
    double k = phaseGain(phase);
    double nudge = k * err;
    if (phase == MovePhase::FINAL && samePosCount > 0)
      nudge += samePosCount * 0.5 * (err > 0 ? 1 : -1);
    return (int)constrain(nudge, -15.0, 15.0);
  }

  double phaseGain(MovePhase p) {
    switch (p) {
      case MovePhase::ACCEL: return 0.10;
      case MovePhase::COAST: return 0.12;
      case MovePhase::DECEL: return 0.16;
      case MovePhase::FINAL: return 0.25;
      default: return 0.1;
    }
  }
};

// static controller map per servo
static std::map<uint8_t, NudgeController> nudgeDB;
inline NudgeController& getNudgeController(uint8_t id) {
  if (nudgeDB.find(id) == nudgeDB.end())
    nudgeDB[id] = NudgeController(id);
  return nudgeDB[id];
}

// ============================================================
//                Unified tick-based motion kernel
// ============================================================

bool move_smooth_ticks(
    const std::vector<uint8_t> &ids,
    const std::vector<int> &start_ticks,
    const std::vector<int> &goal_ticks,
    const std::vector<bool> &mid_nudge_flags,
    bool keepXconstant,
    bool keepYconstant,
    int stepInterval_ms = SMOOTH_STEP_INTERVAL_MS,
    int accelSteps = SMOOTH_ACCEL_STEPS,
    int decelSteps = SMOOTH_DECEL_STEPS,
    int minStep_ticks = SMOOTH_MIN_STEP_TICKS,
    int maxStep_ticks = SMOOTH_MAX_STEP_TICKS,
    int tol_ticks = SMOOTH_TOL_TICKS)
{
  const int n = ids.size();
  if (n == 0) return false;

  // per-servo controllers
  std::vector<NudgeController*> nudgers(n);
  for (int i = 0; i < n; i++)
    nudgers[i] = &getNudgeController(ids[i]);

  std::vector<int> dir(n), totalDiff(n), pos(n);
  int maxDist = 0;
  for (int i = 0; i < n; i++) {
    int dist = goal_ticks[i] - start_ticks[i];
    dir[i] = (dist >= 0) ? 1 : -1;
    totalDiff[i] = abs(dist);
    pos[i] = start_ticks[i];
    maxDist = std::max(maxDist, totalDiff[i]);
  }
  if (maxDist < 3) return true;

  torqueOnGroup(ids);
  adjustPwmGroup(ids, 880);
  ledOnGroup(ids);

  int accelSpan = accelSteps * (minStep_ticks + maxStep_ticks) / 2;
  int decelSpan = decelSteps * (minStep_ticks + maxStep_ticks) / 2;
  int coastSpan = maxDist - (accelSpan + decelSpan);
  if (coastSpan < 0) coastSpan = 0;
  int coastSteps = coastSpan / maxStep_ticks;

  std::vector<float> ratio(n);
  for (int i = 0; i < n; i++)
    ratio[i] = (maxDist > 0) ? ((float)totalDiff[i] / maxDist) : 0.0f;

  std::vector<int> samePosCount(n, 0);
  std::vector<int> prevPos(n, 0);
  for (int i = 0; i < n; i++)
    prevPos[i] = dxl.getPresentPosition(ids[i]);

  bool hasGrip = (std::find(ids.begin(), ids.end(), ID_GRIP) != ids.end());
  MovePhase currentPhase = MovePhase::ACCEL;

  auto step_axes = [&](const char *phase, int iter, int step_ticks) {
    // planned step
    for (int k = 0; k < n; k++) {
      int scaled = (int)round(ratio[k] * step_ticks) * dir[k];
      pos[k] += scaled;
      dxl.setGoalPosition(ids[k], pos[k]);
    }

    // update G alignment if present
    if (hasGrip) {
      kin.readPresentPositions();
      double gAligned = kin.getGdeg_aligned();
      dxl.setGoalPosition(ID_GRIP, deg2ticks(gAligned));
    }

    delay(stepInterval_ms);

    for (int k = 0; k < n; k++) {
      int curr = dxl.getPresentPosition(ids[k]);
      int err = goal_ticks[k] - curr;
      int delta = abs(curr - prevPos[k]);
      if (delta < 2) samePosCount[k]++;
      else samePosCount[k] = 0;
      samePosCount[k] = std::min(samePosCount[k], 6);
      prevPos[k] = curr;

      // predictive mid-nudge
      if (mid_nudge_flags[k]) {
        int correction = nudgers[k]->computeNudge(err, currentPhase, samePosCount[k]);
        dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION,
                                  ids[k], curr + correction);
        nudgers[k]->recordData(pos[k], curr, correction, currentPhase);
      }
    }
  };

  // ---------------- Accel
  currentPhase = MovePhase::ACCEL;
  for (int i = 0; i < accelSteps; i++) {
    int step_ticks = map(i, 0, accelSteps - 1, minStep_ticks, maxStep_ticks);
    step_axes("accel", i, step_ticks);
  }

  // ---------------- Coast
  currentPhase = MovePhase::COAST;
  for (int i = 0; i < coastSteps; i++)
    step_axes("coast", i, maxStep_ticks);

  // ---------------- Decel
  currentPhase = MovePhase::DECEL;
  for (int i = decelSteps - 1; i >= 0; i--) {
    int step_ticks = map(i, 0, decelSteps - 1, minStep_ticks, maxStep_ticks);
    step_axes("decel", i, step_ticks);
  }

  // ---------------- Final nudging
  currentPhase = MovePhase::FINAL;
  const int maxNudges = 6;
  const int nudgeDelay = 85;

  for (int count = 0; count < maxNudges; count++) {
    bool all_good = true;
    for (int k = 0; k < n; k++) {
      int curr = dxl.getPresentPosition(ids[k]);
      int err = goal_ticks[k] - curr;
      if (abs(err) > tol_ticks) {
        all_good = false;
        int nudge = nudgers[k]->computeNudge(err, currentPhase, samePosCount[k]);
        dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ids[k], curr + nudge);
        nudgers[k]->recordData(pos[k], curr, nudge, currentPhase);
      }
    }

    if (hasGrip) {
      kin.readPresentPositions();
      double gAligned = kin.getGdeg_aligned();
      dxl.setGoalPosition(ID_GRIP, deg2ticks(gAligned));
    }

    if (all_good) break;
    delay(nudgeDelay);
  }

  adjustPwmGroup(ids, 400);
  ledOffGroup(ids);

  for (int i = 0; i < n; i++)
    nudgers[i]->printLog();

  return true;
}

// ============================================================
//                Wrappers (API-level commands)
// ============================================================

bool cmdMoveSmoothDeg(uint8_t id, double goal_deg)
{
  if (!dxl.ping(id)) return false;
  int start = dxl.getPresentPosition(id);
  int goal = deg2ticks(goal_deg);
  return move_smooth_ticks({id}, {start}, {goal}, {false}, false, false);
}

bool cmdMoveSmoothGripper(double goal_per)
{
  if (!dxl.ping(ID_GRIP)) return false;
  int start = dxl.getPresentPosition(ID_GRIP);
  int goal = per2ticks(goal_per);
  return move_smooth_ticks({ID_GRIP}, {start}, {goal}, {false}, false, false);
}

bool cmdMoveSmoothXY(double goal_xmm, double goal_ymm)
{
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2)) return false;
  kin.readPresentPositions();
  kin.setXYmm(goal_xmm, goal_ymm);
  double goalA1_deg = kin.getA1deg();
  double goalA2_deg = kin.getA2deg();

  int startA1 = dxl.getPresentPosition(ID_ARM1);
  int startA2 = dxl.getPresentPosition(ID_ARM2);
  int goalA1 = deg2ticks(goalA1_deg);
  int goalA2 = deg2ticks(goalA2_deg);

  return move_smooth_ticks({ID_ARM1, ID_ARM2},
                           {startA1, startA2},
                           {goalA1, goalA2},
                           {true, true}, true, false);
}

bool cmdMoveSmoothYWithGrip(double goal_xmm, double goal_ymm, double goal_gdeg)
{
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2) || !dxl.ping(ID_GRIP)) return false;
  kin.readPresentPositions();
  kin.setXYmm(goal_xmm, goal_ymm);
  double goalA1_deg = kin.getA1deg();
  double goalA2_deg = kin.getA2deg();

  int startA1 = dxl.getPresentPosition(ID_ARM1);
  int startA2 = dxl.getPresentPosition(ID_ARM2);
  int startG  = dxl.getPresentPosition(ID_GRIP);

  int goalA1 = deg2ticks(goalA1_deg);
  int goalA2 = deg2ticks(goalA2_deg);
  int goalG  = deg2ticks(goal_gdeg);

  return move_smooth_ticks({ID_ARM1, ID_ARM2, ID_GRIP},
                           {startA1, startA2, startG},
                           {goalA1, goalA2, goalG},
                           {true, true, false}, false, true);
}

bool cmdMoveYmm(double y_mm)
{
  if (y_mm < 40 || y_mm > 110) return false;
  kin.readPresentPositions();
  double x_mm_curr = kin.getXmm();
  double g_goal_deg = kin.getGdeg_aligned();
  return cmdMoveSmoothYWithGrip(x_mm_curr, y_mm, g_goal_deg);
}

bool cmdMoveXmm(double x_mm)
{
  if (x_mm > 55) return false;
  kin.readPresentPositions();
  double y_mm_curr = kin.getYmm();
  double g_goal_deg = kin.getGdeg_aligned();
  kin.setXYmm(x_mm, y_mm_curr);
  return cmdMoveSmoothYWithGrip(x_mm, y_mm_curr, g_goal_deg);
}

