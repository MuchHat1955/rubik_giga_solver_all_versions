#pragma once
#include <Arduino.h>
#include <map>
#include <vector>

extern const int SERVO_TICKS_MIN;
extern const int SERVO_TICKS_MAX;

class ServoManager {
public:
  struct ServoInfo {
    uint8_t id{ 0 };
    String key;
    int posTicks{ 0 };
    int goalTicks{ 0 };
    int current_mA{ 0 };
    int temperature_C{ 0 };
    bool pingOK{ false };
    bool hasError{ false };
    bool isStall{ false };
    bool isNearStall{ false };
    bool isAtMin{ false };
    bool isAtMax{ false };
    bool isThermal{ false };
    bool inverted{ false };
  };

  // ----------------------------------------------------------
  //                  CONSTRUCTOR
  // ----------------------------------------------------------
  ServoManager();

  // ----------------------------------------------------------
  //             REGISTRATION / ACCESSORS
  // ----------------------------------------------------------
  void registerServo(const String &key, uint8_t id, bool inverted = false);
  const std::map<String, ServoInfo> &getServoStore() const;

  // ----------------------------------------------------------
  //                     UPDATE STATUS
  // ----------------------------------------------------------
  void updateServo(const String &key);
  void updateServos();

  // ----------------------------------------------------------
  //                        GETTERS
  // ----------------------------------------------------------
  int getServoTicks(const String &key);
  int getServoCurrent(const String &key);
  bool hasServoErrors(const String &key);
  bool pingServoOK(const String &key);
  String getErrorString(const String &key);

  // ----------------------------------------------------------
  //                  TYPE IDENTIFICATION
  // ----------------------------------------------------------
  bool isServo(const String &key) const;
  bool isServoPose(const String &key) const;
  bool isGroupPose(const String &key) const;

  // ----------------------------------------------------------
  //                  BTN / TYPE HELPERS
  // ----------------------------------------------------------
  bool isBtnForPose(const String &key) const;
  String getPoseFromBtn(const String &key) const;
  bool isBtnForServo(const String &key) const;
  String getServoFromBtn(const String &key) const;
  bool isBtnForGroupPose(const String &key) const;
  String getGroupPoseFromBtn(const String &key) const;

  // ----------------------------------------------------------
  //                   REFLECT UI HELPERS
  // ----------------------------------------------------------
  void reflectUI();
  void reflectUIForKey(const String &key);

  // ----------------------------------------------------------
  //                 GROUP POSE MANAGEMENT
  // ----------------------------------------------------------
  void addGroupPose(const String &groupKey, const std::vector<String> &poseKeys);
  std::vector<String> getGroupPose(const String &groupKey);
  bool isServoAtPose(const String &key, int targetTicks, int tol = 10);

  // ----------------------------------------------------------
  //                   MOTION CONTROL
  // ----------------------------------------------------------
  bool moveServoToTicks(const String &key, int ticks);
  bool moveServoToAngle(const String &key, float angleDeg);
  bool moveServoToPose(const String &poseKey);           // main version
  bool moveServoToPose(const String &key, const String &poseKey); // optional overload
  bool moveServosToGroupPose(const String &groupKey, uint32_t duration_ms = 700);

  // ----------------------------------------------------------
  //                   DIAGNOSTICS
  // ----------------------------------------------------------
  String getServosDiagnosticString() const;

  // ----------------------------------------------------------
  //              STARTUP ERROR TRACKING
  // ----------------------------------------------------------
  void setStartupTestErrorString(const String &msg);
  String getStartupTestErrorString() const;

private:
  std::map<String, ServoInfo> servosStore;
  std::map<String, std::vector<String>> groupPoseStore;

  int clampTicksForServo(const String &key, int ticks);

  String startupErrorSummary;
};
