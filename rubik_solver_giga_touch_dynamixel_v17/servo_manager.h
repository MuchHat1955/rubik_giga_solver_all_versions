#pragma once
#include <Arduino.h>
#include <map>
#include <vector>

extern const int SERVO_TICKS_MIN;
extern const int SERVO_TICKS_MAX;

class ServoManager {
public:
  struct ServoInfo {
    uint8_t id{0};
    String key;
    int posTicks{0};
    int goalTicks{0};
    int current_mA{0};
    int temperature_C{0};
    bool pingOK{false};
    bool hasError{false};
    bool isStall{false};
    bool isNearStall{false};
    bool isAtMin{false};
    bool isAtMax{false};
    bool isThermal{false};
    bool inverted{false};
  };

  // --- constructor ---
  ServoManager();

  // --- registration / mapping ---
  void registerServo(const String &key, uint8_t id, bool inverted = false);
  const std::map<String, ServoInfo>& getServoMap() const;  // declaration only

  // --- update functions ---
  void updateServo(const String &key, bool refreshHW = true);
  void updateAll(bool refreshHW = true);

  // --- getters ---
  int getServoTicks(const String &key);
  int getServoCurrent(const String &key);
  bool hasServoErrors(const String &key);
  bool pingServoOK(const String &key);
  String getErrorString(const String &key);

  // --- type identification ---
  bool isServo(const String &key) const;
  bool isServoPose(const String &key) const;
  bool isGroupPose(const String &key) const;

  // --- group pose management ---
  void addGroupPose(const String &groupKey, const std::vector<String> &poseKeys);
  std::vector<String> getGroupPose(const String &groupKey);
  bool isServoAtPose(const String &key, int targetTicks, int tol = 10);

  // --- motion control ---
  bool moveServoToTicks(const String &key, int ticks);
  bool moveServoToAngle(const String &key, float angleDeg);
  bool moveServoToPose(const String &key, const String &poseKey);
  bool moveServoToPose(const String &poseKey);
  bool moveServosToGroupPose(const String &groupKey, uint32_t duration_ms = 700);

  // --- diagnostics ---
  String getFullDiagnosticString() const;

  // --- startup error tracking ---
  void setStartupTestErrorString(const String &msg);
  String getStartupTestErrorString() const;

private:
  std::map<String, ServoInfo> servos;
  std::map<String, std::vector<String>> groupPoseStore;

  int clampTicksForServo(const String &key, int ticks);
  void reflectUI(const String &key);

  String startupErrorSummary;
};
