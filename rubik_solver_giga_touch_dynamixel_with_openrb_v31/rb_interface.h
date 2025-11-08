#pragma once
#include "utils.h"

// Servo ID definitions (match RB firmware)
#define ID_ARM1 11
#define ID_ARM2 12
#define ID_WRIST 13
#define ID_GRIPPER1 14
#define ID_GRIPPER2 15
#define ID_BASE 16

// -----------------------------------------------------------------------------
// RBStatus
//   Represents the current robot state as reported by the RB board.
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
// RBInterface
//   GIGA-side serial communication interface to the RB controller.
// -----------------------------------------------------------------------------
class RBInterface {
public:
  explicit RBInterface();

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

  // ---- error retrieval -----------------------------------------------------
  const char* getLastErrorLine();
  String getAllErrorLines() const;

  // ---- last known status ---------------------------------------------------
  RBStatus getLastStatus() const;

private:
  RBStatus last;
  std::vector<String> errorLines;
  bool verboseOn = false;

  // ---- internal helpers ----------------------------------------------------
  bool waitForCompletion(const char* commandName);
  bool readUntilEnd(const char* keyword);
  void parseStatusLine(const String& line);
  void verifyExpected(const char* commandName);
  void clearErrorBuffer();
  bool requestServoInfo(uint8_t id);
};