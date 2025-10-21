// ----------------------------------------------------------
//                      servo_manager.cpp
// ----------------------------------------------------------
#include "servo_manager.h"
#include "logging.h"
#include "ui_touch.h"
#include <Dynamixel2Arduino.h>
#include <actuator.h>

extern Dynamixel2Arduino dxl;
extern int getParamValue(const char *key);
extern void updateButtonStateByKey(const String &servoId, bool issue, bool active);

// ----------------------------------------------------------
//                  CONSTANTS / GLOBALS
// ----------------------------------------------------------
static const int XL_TICKS_MIN = 0;
static const int XL_TICKS_MAX = 4095;
static const int POSE_TOL_TICKS = 10;
static const int NEAR_STALL_CURRENT_mA = 800;
static const int STALL_CURRENT_mA = 1200;
static const int THERMAL_LIMIT_C = 75;
static const int CHECK_INTERVAL_MS = 15;

extern Dynamixel2Arduino dxl;
extern int getParamValue(const char *key);
extern void updateButtonStateByKey(const String &servoId, bool issue, bool active);

// ----------------------------------------------------------
//               SERVO MANAGER IMPLEMENTATION
// ----------------------------------------------------------

ServoManager::ServoManager() {}

// ----------------------------------------------------------
//                 REGISTRATION / ACCESSORS
// ----------------------------------------------------------
void ServoManager::registerServo(const String &key, uint8_t id, bool inverted) {
  ServoInfo s;
  s.id = id;
  s.key = key;
  s.inverted = inverted;
  servos[key] = s;
  LOG_VAR2("register servo", key, "inverted", inverted ? "inverted" : "normal");
}

// ----------------------------------------------------------
//                     UPDATE STATUS
// ----------------------------------------------------------
void ServoManager::updateServo(const String &key, bool refreshHW) {
  auto it = servos.find(key);
  if (it == servos.end()) return;
  ServoInfo &s = it->second;

  if (refreshHW) {
    s.pingOK = dxl.ping(s.id);
    if (!s.pingOK) {
      s.hasError = true;
      reflectUI(key);
      return;
    }

    s.posTicks = dxl.getPresentPosition(s.id);
    s.current_mA = dxl.getPresentCurrent(s.id);

    // 146 = PRESENT_TEMPERATURE for XL430 (Robotis official control table)
    s.temperature_C = dxl.readControlTableItem(146, s.id);

    int mn = getParamValue((key + "_min").c_str());
    int mx = getParamValue((key + "_max").c_str());
    if (mn <= 0 && mx <= 0) {
      mn = XL_TICKS_MIN;
      mx = XL_TICKS_MAX;
    }

    s.isAtMin = (s.posTicks <= mn + POSE_TOL_TICKS);
    s.isAtMax = (s.posTicks >= mx - POSE_TOL_TICKS);
    s.isThermal = (s.temperature_C >= THERMAL_LIMIT_C);
    s.isStall = (s.current_mA >= STALL_CURRENT_mA);
    s.isNearStall = (!s.isStall && s.current_mA >= NEAR_STALL_CURRENT_mA);
    s.hasError = (!s.pingOK) || s.isThermal || s.isStall || s.isAtMin || s.isAtMax;
  }

  reflectUI(key);
}

void ServoManager::updateAll(bool refreshHW) {
  for (auto &kv : servos) updateServo(kv.first, refreshHW);
}

// ----------------------------------------------------------
//                        GETTERS
// ----------------------------------------------------------
int ServoManager::getServoPos(const String &key) {
  auto it = servos.find(key);
  return (it == servos.end()) ? 0 : it->second.posTicks;
}

int ServoManager::getServoCurrent(const String &key) {
  auto it = servos.find(key);
  return (it == servos.end()) ? 0 : it->second.current_mA;
}

bool ServoManager::hasServoErrors(const String &key) {
  auto it = servos.find(key);
  return (it == servos.end()) ? true : it->second.hasError;
}

bool ServoManager::pingServoOK(const String &key) {
  auto it = servos.find(key);
  return (it == servos.end()) ? false : dxl.ping(it->second.id);
}

String ServoManager::getErrorString(const String &key) {
  auto it = servos.find(key);
  if (it == servos.end()) return "unknown servo";
  const ServoInfo &s = it->second;
  if (!s.pingOK) return "id not found";
  if (s.isThermal) return "temp overload";
  if (s.isStall) return "stall";
  if (s.isNearStall) return "near stall";
  if (s.isAtMin) return "min limit";
  if (s.isAtMax) return "max limit";
  return "";
}

// ----------------------------------------------------------
//                  TYPE IDENTIFICATION
// ----------------------------------------------------------
bool ServoManager::isServo(const String &key) const {
  return servos.find(key) != servos.end();
}
bool ServoManager::isServoPose(const String &key) const {
  return getParamValue(key.c_str()) != 0;
}
bool ServoManager::isGroupPose(const String &key) const {
  return groupPoseStore.find(key) != groupPoseStore.end();
}

// ----------------------------------------------------------
//                    GROUP POSE STORE
// ----------------------------------------------------------
void ServoManager::addGroupPose(const String &groupKey, const std::vector<String> &poseKeys) {
  groupPoseStore[groupKey] = poseKeys;
}

std::vector<String> ServoManager::getGroupPose(const String &groupKey) {
  auto it = groupPoseStore.find(groupKey);
  return (it != groupPoseStore.end()) ? it->second : std::vector<String>();
}

bool ServoManager::isServoAtPose(const String &key, int targetTicks, int tol) {
  return abs(getServoPos(key) - targetTicks) <= tol;
}

// ----------------------------------------------------------
//                     MOTION CONTROL
// ----------------------------------------------------------
bool ServoManager::moveServoToTicks(const String &key, int ticks) {
  auto it = servos.find(key);
  if (it == servos.end()) return false;
  ServoInfo &s = it->second;

  updateServo(key, true);
  if (!s.pingOK || s.isThermal) {
    reflectUI(key);
    return false;
  }

  s.goalTicks = clampTicksForServo(key, ticks);
  dxl.setGoalPosition(s.id, s.goalTicks);
  updateServo(key, false);
  return true;
}

bool ServoManager::moveServoToAngle(const String &key, float angleDeg) {
  auto it = servos.find(key);
  if (it == servos.end()) return false;
  ServoInfo &s = it->second;

  if (s.inverted) angleDeg = -angleDeg;

  int ticks = (int)lround((angleDeg / 360.0f) * XL_TICKS_MAX);
  return moveServoToTicks(key, ticks);
}

bool ServoManager::moveServoToPose(const String &key, const String &poseKey) {
  int target = getParamValue(poseKey.c_str());
  if (target == 0) {
    LOG_VAR("moveServoToPose: invalid pose value 0", poseKey.c_str());
    return false;
  }
  return moveServoToTicks(key, target);
}

bool ServoManager::moveServoToPose(const String &poseKey) {
  int target = getParamValue(poseKey.c_str());
  if (target == 0) {
    LOG_VAR("moveServoToPose: invalid pose value 0", poseKey.c_str());
    return false;
  }

  int underscore = poseKey.indexOf('_');
  if (underscore <= 0) return false;
  String servoName = poseKey.substring(0, underscore) + "_program";
  if (!isServo(servoName)) return false;

  return moveServoToTicks(servoName, target);
}

bool ServoManager::moveServosToGroupPose(const String &groupKey, uint32_t duration_ms) {
  auto it = groupPoseStore.find(groupKey);
  if (it == groupPoseStore.end()) return false;

  const auto &poseList = it->second;
  if (poseList.empty()) return false;

  struct Plan {
    ServoInfo *s;
    int start;
    int goal;
  };
  std::vector<Plan> plan;

  for (auto &poseKey : poseList) {
    int underscore = poseKey.indexOf('_');
    if (underscore <= 0) continue;
    String servoName = poseKey.substring(0, underscore) + "_program";
    if (!isServo(servoName)) continue;

    ServoInfo &sv = servos[servoName];
    int target = getParamValue(poseKey.c_str());
    if (target == 0) continue;

    updateServo(servoName, true);
    if (!sv.pingOK || sv.isThermal) {
      reflectUI(servoName);
      continue;
    }

    Plan p{ &sv, sv.posTicks, clampTicksForServo(servoName, target) };
    sv.goalTicks = p.goal;
    plan.push_back(p);
  }

  if (plan.empty()) return false;

  uint32_t t0 = millis();
  while (true) {
    uint32_t t = millis() - t0;
    float u = (duration_ms == 0) ? 1.0f : (float)t / (float)duration_ms;
    if (u > 1.0f) u = 1.0f;
    float eased = 0.5f - 0.5f * cosf(u * PI);
    for (auto &p : plan) {
      int cmd = p.start + (int)lround((p.goal - p.start) * eased);
      dxl.setGoalPosition(p.s->id, cmd);
    }
    if (u >= 1.0f) break;
    delay(CHECK_INTERVAL_MS);
  }

  for (auto &p : plan) updateServo(p.s->key, true);
  return true;
}

// ----------------------------------------------------------
//                  DIAGNOSTICS / UTILITIES
// ----------------------------------------------------------
String ServoManager::getFullDiagnosticString() const {
  String out = "servo diagnostics\n-----------------\n";
  for (auto &kv : servos) {
    const ServoInfo &s = kv.second;
    out += s.key + " | id:" + String(s.id);
    out += " | pos:" + String(s.posTicks);
    out += " | cur:" + String(s.current_mA) + "mA";
    out += " | temp:" + String(s.temperature_C) + "C";
    out += s.inverted ? " | inverted" : "";
    if (!s.pingOK)
      out += " | id not found";
    else if (s.isThermal)
      out += " | temp overload";
    else if (s.isStall)
      out += " | stall";
    else if (s.isNearStall)
      out += " | near stall";
    else if (s.isAtMin)
      out += " | min limit";
    else if (s.isAtMax)
      out += " | max limit";
    out += "\n";
  }
  return out;
}

void ServoManager::setStartupTestErrorString(const String &msg) {
  startupErrorSummary = msg;
}
String ServoManager::getStartupTestErrorString() const {
  return startupErrorSummary.length() ? startupErrorSummary : "No startup test run yet";
}

// ----------------------------------------------------------
//               INTERNAL UTILITIES
// ----------------------------------------------------------
int ServoManager::clampTicksForServo(const String &key, int ticks) {
  int mn = getParamValue((key + "_min").c_str());
  int mx = getParamValue((key + "_max").c_str());
  if (mn <= 0 && mx <= 0) {
    mn = XL_TICKS_MIN;
    mx = XL_TICKS_MAX;
  }
  return constrain(ticks, mn, mx);
}

void ServoManager::reflectUI(const String &key) {
  auto it = servos.find(key);
  if (it == servos.end()) return;
  const ServoInfo &s = it->second;
  bool issue = (!s.pingOK) || s.isThermal || s.isStall || s.isAtMin || s.isAtMax;
  bool active = abs(s.posTicks - s.goalTicks) <= POSE_TOL_TICKS;
  updateButtonStateByKey(key, issue, active);
}

// ----------------------------------------------------------
//               GLOBAL INSTANCE + INIT
// ----------------------------------------------------------
ServoManager servoMgr;

void initPoseStore() {
  LOG_SECTION_START("initPoseStore");

  struct ServoEntry {
    const char *key;
    uint8_t id;
    bool inverted;
  };

  ServoEntry servoList[] = {
    { "arm1_program", 11, false },
    { "arm2_program", 12, false },
    { "wrist_program", 13, false },
    { "grip1_program", 14, false },
    { "grip2_program", 15, false },
    { "base_program", 16, false },
  };

  int failCount = 0;
  String startupReport;

  for (auto &s : servoList) {
    servoMgr.registerServo(s.key, s.id, s.inverted);
    bool ok = dxl.ping(s.id);
    if (!ok) {
      failCount++;
      startupReport += String("❌ ") + s.key + " (id " + s.id + ") not found\n";
      updateButtonStateByKey(s.key, true, false);
    } else {
      startupReport += String("✅ ") + s.key + " (id " + s.id + ") OK\n";
      updateButtonStateByKey(s.key, false, false);
    }
  }

  servoMgr.addGroupPose("arms_home", { "arm1_pose_0", "arm2_pose_0", "wrist_pose_0" });
  servoMgr.addGroupPose("arms_5mm", { "arm1_pose_5mm", "arm2_pose_5mm", "wrist_pose_5mm" });
  servoMgr.addGroupPose("arms_10mm", { "arm1_pose_10mm", "arm2_pose_10mm", "wrist_pose_10mm" });
  servoMgr.addGroupPose("arms_15mm", { "arm1_pose_15mm", "arm2_pose_15mm", "wrist_pose_15mm" });
  servoMgr.addGroupPose("arms_high", { "arm1_pose_high", "arm2_pose_high", "wrist_pose_high" });
  servoMgr.addGroupPose("grippers_open", { "grip1_pose_0", "grip2_pose_0" });
  servoMgr.addGroupPose("grippers_closed", { "grip1_pose_1", "grip2_pose_1" });

  if (failCount == 0) {
    servoMgr.setStartupTestErrorString("All servos OK");
    setFooter("Startup test: all servos OK");
  } else {
    String summary = String(failCount) + " servo(s) missing:\n" + startupReport;
    servoMgr.setStartupTestErrorString(summary);
    setFooter("Startup test: issue(s) found");
  }

  LOG_SECTION_END();
}
