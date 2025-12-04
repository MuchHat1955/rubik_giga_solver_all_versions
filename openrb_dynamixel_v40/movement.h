#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include <vector>

class Dynamixel2Arduino;
class VerticalKinematics;

// Motion phases for nudging
enum class MovePhase : uint8_t {
  ACCEL,
  COAST,
  DECEL,
  FINAL
};

// ----------------------------------------------------------------------
// NudgeController - per-servo nudging logic
// ----------------------------------------------------------------------
class NudgeController {
public:
  explicit NudgeController(uint8_t id = 0);

  void recordData(int prevGoalTicks, int currPos, int nudge, MovePhase phase);
  int  computeNudge(int currErr, int dir, MovePhase phase, int samePosCount);
  void printLog();

private:
  struct Record {
    uint32_t   t_ms;
    int        prevGoalTicks;
    int        currPosTicks;
    int        errTicks;
    int        nudgeApplied;
    MovePhase  phase;
  };

  int    baseEstimate(int errTicks, MovePhase phase, int samePosCount);
  double phaseGain(MovePhase p);

  uint8_t               id;
  std::vector<Record>   records;
  static const size_t   maxRecords = 64;
};

// ----------------------------------------------------------------------
// AxisGroupController - abstraction for 1–3 axis coordinated motion
// ----------------------------------------------------------------------
class AxisGroupController {
public:
  enum class AxisRunMode : uint8_t {
    UNDEFINED = 0,
    SINGLE_SERVO,
    XY_VERTICAL,
    XY_HORIZONTAL,
    GRIPPER
  };

  AxisGroupController(Dynamixel2Arduino* dxl_ptr,
                      VerticalKinematics* kin_ptr);

  // Mode & configuration
  void setMode(AxisRunMode m);
  void setServoId(uint8_t id);    // for SINGLE_SERVO
  void setGoalDeg(double deg);    // for SINGLE_SERVO
  void setXGoalMm(double x_mm);   // for XY modes
  void setYGoalMm(double y_mm);   // for XY modes
  void setGoalPercent(double per);// for GRIPPER

  bool init();                    // initialize for the chosen mode

  // Access for motion kernel
  int      axesCount() const;
  uint8_t  getId(uint8_t index) const;
  int      getGoalTicks(uint8_t index) const;
  const char* getMoveName() const;

  bool             getNudgeFlag(uint8_t index) const;
  NudgeController* getNudgeController(uint8_t index);

  void start();                  // LEDs on etc.
  void end();                    // LEDs off etc.

  void readPresentTicks(int* posList);
  void writeTicks(const int* posList);

  // Slave axis goal, given master ticks
  int getSyncGoal(uint8_t slaveIndex, int masterTicks);

private:
  bool initSingle();
  bool initXY(bool keepX);
  bool initGripper();

  Dynamixel2Arduino*  dxlPtr;
  VerticalKinematics* kinPtr;
  AxisRunMode         mode;
  bool                configured;

  // Servo IDs
  uint8_t id_servo;

  // Goals (varies by mode)
  double goal_deg;
  double goal_mm_x;
  double goal_mm_y;
  double goal_percent;

  // Cached ticks / IDs
  std::vector<int>      start_ticks;
  std::vector<int>      goal_ticks;
  std::vector<int>      curr_ticks;
  std::vector<uint8_t>  id_list;
  std::vector<double>   dir_list;
  std::vector<bool>     nudge_flags;

  // Gripper progress memory (for monotonic sync)
  double grip_lastProgress;
};

// Global instance used by the motion functions
extern AxisGroupController axes;

// High-level motion APIs
bool move_smooth();   // wrapper -> v2
bool move_smooth_v1();
bool move_smooth_v2();

// Command wrappers
bool cmdMoveServoDeg(uint8_t id, double goal_deg);
bool cmdMoveServoPer(int id, double goal_per);
bool cmdMoveGripperPer(double goal_per);
bool cmdMoveWristDegVertical(double goal_deg);
bool cmdMoveYmm(double goal_ymm);
bool cmdMoveXmm(double x_mm);
bool cmdMoveGripperClamp();

void read_print_xy_status();
void print_all_status();

#endif  // MOVEMENT_H
