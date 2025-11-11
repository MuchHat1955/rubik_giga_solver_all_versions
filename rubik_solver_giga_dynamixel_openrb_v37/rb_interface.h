#pragma once
#include "utils.h"
#include "logging.h"
#include <Arduino.h>
#include <vector>

// Servo ID definitions (match RB firmware)
#define ID_ARM1 11
#define ID_ARM2 12
#define ID_WRIST 13
#define ID_GRIPPER1 14
#define ID_GRIPPER2 15
#define ID_BASE 16

#define MAX_SERVOS 20  // adjust for your robot

// -----------------------------------------------------------------------------
// ServoInfo
// -----------------------------------------------------------------------------
class ServoInfo {
public:
  uint8_t id = 0;
  uint8_t op_mode = 0;
  uint8_t drive_mode = 0;
  bool time_based = false;
  int profile_vel = 0;
  int profile_accel = 0;
  int pos_min = 0;
  int pos_max = 0;
  double span_deg = 0;
  int pos_present = 0;
  double rpm = 0;
  double ticks_per_sec = 0;

  void clear() {
    *this = ServoInfo();
  }

  void log() const {
    LOG_PRINTF("INFO id {%d} op_mode {%d} drive_mode {%d} time_based {%s} profile_vel {%d} rpm {%.3f} tps {%.1f} profile_accel {%d} pos_min {%d} pos_max {%d} span_deg {%.1f} pos_present {%d}",
               id, op_mode, drive_mode, time_based ? "time" : "velocity",
               profile_vel, rpm, ticks_per_sec, profile_accel,
               pos_min, pos_max, span_deg, pos_present);
  }
};

// -----------------------------------------------------------------------------
// RBStatus - current robot state from RB board
// -----------------------------------------------------------------------------
struct RBStatus {
  double x_mm = 0.0;
  double y_mm = 0.0;
  double a1_deg = 0.0;
  double a2_deg = 0.0;
  double g_vert_deg = 0.0;
  double g1_per = 0.0;
  double g2_per = 0.0;
  double base_deg = 0.0;
  int completed = 0;
};

// -----------------------------------------------------------------------------
// RBInterface - GIGA-side serial communication interface to RB controller
// -----------------------------------------------------------------------------
class RBInterface {
public:
  RBInterface();

  // ---- communication setup --------------------------------------------------
  bool begin(unsigned long baud = 115200, uint32_t timeout_ms = 3000);

  // ---- command sending ------------------------------------------------------
  bool runCommand(const char* name, const float* args = nullptr, int argCount = 0);

  // ---- state / info queries -------------------------------------------------
  bool updateInfo();
  bool xyInfoMm(double* x, double* y);
  bool grippersInfoPer(double* g);
  bool gripper1InfoPer(double* g);
  bool gripper2InfoPer(double* g);
  bool baseInfoDeg(double* b);
  bool wristVertInfoDeg(double* v);

  // ---- movement helpers ----------------------------------------------------
  bool moveYmm(double y);
  bool moveXmm(double x);
  bool moveBaseDeg(double deg);
  bool moveWristVertDeg(double deg);
  bool moveGrippersPer(double per);
  bool moveGripper1Per(double per);
  bool moveGripper2Per(double per);

  // ---- servo info management -----------------------------------------------
  bool requestServoInfo(uint8_t id);
  bool requestAllServoInfo();
  String getAllServoInfoLines() const;

    // ---- last known status ---------------------------------------------------
  RBStatus getLastStatus() const;

private:
  RBStatus last;
  bool verboseOn = false;
  ServoInfo servo_infos[MAX_SERVOS];

  // ---- internal helpers ----------------------------------------------------
  bool waitForCompletion(const char* commandName);
  bool readUntilEnd(const char* keyword);
  void parseStatusLine(const String& line);
  bool verifyExpected(const char* cmd_name, double val, int servo_id, double tol);
};

// Optional global instance
extern RBInterface rb;
