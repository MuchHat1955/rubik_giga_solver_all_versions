#pragma once
#include <Arduino.h>
#include <vector>
#include <map>
#include <algorithm>
#include "servos.h"
#include "vertical_kinematics.h"
#include "utils.h"

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
#define SMOOTH_STEP_INTERVAL_MS 25  // was 15
#define SMOOTH_ACCEL_STEPS 15       // was 20
#define SMOOTH_DECEL_STEPS 15       // was 20
#define SMOOTH_MIN_STEP_TICKS 1
#define SMOOTH_MAX_STEP_TICKS 15  // was 25
#define SMOOTH_TOL_TICKS 4

// ============================================================
// Default motion-profile parameters for move_smooth()
// ============================================================

// Time between incremental servo goal updates (ms)
#define SMOOTH_STEP_INTERVAL_MS 35
// Number of acceleration steps before reaching full speed
#define SMOOTH_ACCEL_STEPS 10 // was 20
// Number of deceleration steps before stopping
#define SMOOTH_DECEL_STEPS 10 // was 20
// Minimum tick change per update (slowest motion increment)
#define SMOOTH_MIN_STEP_TICKS 1 
// Maximum tick change per update (fastest motion increment)
#define SMOOTH_MAX_STEP_TICKS 15 // was 25
// Tolerance in ticks for considering goal reached
#define SMOOTH_TOL_TICKS 4

// ------------------------------------------------------------
// Conversion helpers
// ------------------------------------------------------------
inline int mm2ticks(double mm) {
  return (int)round(mm / MM_PER_TICK);
}
inline int deg2ticks(double deg) {
  return (int)round(deg / DEG_PER_TICK);
}
inline int per2ticks(double per) {
  return (int)round(per * (4095.0 / 100.0));
}

// ============================================================
//                 Nudge Controller Class
// ============================================================

enum class MovePhase : uint8_t { ACCEL,
                                 COAST,
                                 DECEL,
                                 FINAL };

class NudgeController {
public:
  struct Record {
    unsigned long t_ms;
    int prevGoalTicks;
    int currPosTicks;
    int errTicks;
    int nudgeApplied;
    MovePhase phase;
  };

  explicit NudgeController(uint8_t servoId = 0)
    : id(servoId) {}

  void recordData(int prevGoal, int currPos, int nudge, MovePhase phase);
  int computeNudge(int currErr, MovePhase phase, int samePosCount);
  void printLog();

private:
  uint8_t id;
  std::vector<Record> records;
  size_t maxRecords = 100;

  int baseEstimate(int err, MovePhase phase, int samePosCount);
  double phaseGain(MovePhase p);
};

// persistent controller map (declared in movement.cpp)
extern std::map<uint8_t, NudgeController> nudgeDB;
NudgeController& getNudgeController(uint8_t id);

// ============================================================
//                Unified tick-based motion kernel
// ============================================================

bool move_smooth_ticks(
  const std::vector<uint8_t>& ids,
  const std::vector<int>& start_ticks,
  const std::vector<int>& goal_ticks,
  const std::vector<bool>& mid_nudge_flags,
  bool keepXconstant,
  bool keepYconstant,
  int stepInterval_ms = SMOOTH_STEP_INTERVAL_MS,
  int accelSteps = SMOOTH_ACCEL_STEPS,
  int decelSteps = SMOOTH_DECEL_STEPS,
  int minStep_ticks = SMOOTH_MIN_STEP_TICKS,
  int maxStep_ticks = SMOOTH_MAX_STEP_TICKS,
  int tol_ticks = SMOOTH_TOL_TICKS);

// ============================================================
//                High-level wrappers / API commands
// ============================================================

// convenience motion commands in mm
bool cmdMoveServoDeg(uint8_t id, double goal_deg);
bool cmdMoveYmm(double y_mm);
bool cmdMoveXmm(double x_mm);
bool cmdMoveGripperPer(double goal_per);
