// param_store.cpp
#include <Arduino.h>
#include <map>
#include <string>
#include <algorithm>
#include <KVStore.h>
#include <kvstore_global_api.h>
#include "logging.h"
#include "param_store.h"
#include "servos.h"
#include "servo_manager.h"

extern ServoManager servoMgr;

#ifndef MBED_SUCCESS
#define MBED_SUCCESS 0
#endif

int clampServoAbsoluteTicks(int val) {
  return std::max(SERVO_TICKS_MIN, std::min(SERVO_TICKS_MAX, val));
}
int clampServoParamTicks(const char* key, int val) {
  if (!servoMgr.isServo(key)) return val;  // must return int

  String kmin = String(key) + "_min";
  String kmax = String(key) + "_max";

  int mn = getParamValue(kmin.c_str());
  int mx = getParamValue(kmax.c_str());

  if (mn < SERVO_TICKS_MIN) mn = SERVO_TICKS_MIN;
  if (mx > SERVO_TICKS_MAX) mx = SERVO_TICKS_MAX;

  // Clamp within range
  if (val < mn) val = mn;
  else if (val > mx) val = mx;

  return val;
}

// ----------------------------------------------------------
//                    STUBS
// ----------------------------------------------------------

void runAction(const char* key) {
  LOG_SECTION_START_VAR("run", "action", key);

  if (servoMgr.isServoPose(key)) {
    LOG_VAR("move servo to pose", key);
    servoMgr.moveServoToPose(key);
  } else if (servoMgr.isGroupPose(key)) {
    LOG_VAR("move servo to group pose", key);
    servoMgr.moveServosToGroupPose(key);
  }

  LOG_SECTION_END();
}

// overload for convenience
void runAction(const std::string& k) {
  runAction(k.c_str());
}

// Checks if a key ends with "_min" or "_max" AND the prefix is a valid servo key
bool isServoMinMaxKey(const char* key) {
  if (!key || !*key) return false;

  size_t len = strlen(key);
  if (len <= 4) return false;  // too short to contain "_min"/"_max"

  // Check for suffix "_min" or "_max"
  const char* suffix = nullptr;
  if (len > 4 && strcmp(key + len - 4, "_min") == 0)
    suffix = "_min";
  else if (len > 4 && strcmp(key + len - 4, "_max") == 0)
    suffix = "_max";

  if (!suffix) return false;

  // Copy prefix (everything before suffix)
  char base[64];
  size_t prefixLen = len - strlen(suffix);
  if (prefixLen >= sizeof(base)) prefixLen = sizeof(base) - 1;
  strncpy(base, key, prefixLen);
  base[prefixLen] = '\0';

  // Return true only if the base key is a known servo
  return servoMgr.isServo(base);
}

// Extracts the servo base key (e.g. "arm1") from "arm1_min" or "arm1_max"
String servoKeyFromMinMax(const char* key) {
  if (!key || !*key) return "";

  size_t len = strlen(key);
  if (len <= 4) return "";

  const char* suffix = nullptr;
  if (len > 4 && strcmp(key + len - 4, "_min") == 0)
    suffix = "_min";
  else if (len > 4 && strcmp(key + len - 4, "_max") == 0)
    suffix = "_max";

  if (!suffix) return "";

  size_t prefixLen = len - strlen(suffix);
  return String(key).substring(0, prefixLen);
}

// Extracts the servo base key (e.g. "arm1") from a pose key like "arm1_open" or "arm1_pose_1"
String servoKeyFromPose(const char* key) {
  if (!key || !*key) return "";

  // Find the first underscore
  const char* underscore = strchr(key, '_');
  if (!underscore) return "";  // no underscore found → not a pose key

  // Compute prefix length (everything before first '_')
  size_t prefixLen = underscore - key;

  // Build and return the prefix
  return String(key).substring(0, prefixLen);
}

void incrementParam(const char* key, int delta) {

  int val = getParamValue(key);
  LOG_VAR_("increment param for key", key);
  LOG_VAR_CONT_("from val", val);
  LOG_VAR_CONT("with delta", delta);

  val += delta;
  val = clampServoParamTicks(key, val);
  if (servoMgr.isServo(key)) servoMgr.moveServoToTicks(key, val);
  else if (isServoMinMaxKey(key)) {
    String servoKey = servoKeyFromMinMax(key);
    servoMgr.moveServoToTicks(servoKey.c_str(), val);
  }
  setParamValue(key, val);
}

// overload for convenience
void incrementParam(const std::string& k, int delta) {
  incrementParam(k.c_str(), delta);
}

// ----------------------------------------------------------
//                    PARAM
// ----------------------------------------------------------

void updateDerived(const std::string& k, int v);

struct ParamRecord {
  char id[24];
  int value;
};

struct ParamStorage {
  ParamRecord records[64];
  int count;
};

struct Param {
  int value{ 0 }, minv{ 0 }, maxv{ 0 };
  bool fixed{ false };  // the min and max for the min and max are fixed not changeable in UI
  bool persist{ true };
};
static std::map<std::string, Param> store;

static void add(const char* k, int v, int mn, int mx, bool fixed_ = false, bool persist_ = true) {
  v = clampServoParamTicks(k, v);
  mn = clampServoAbsoluteTicks(mn);
  mx = clampServoAbsoluteTicks(mx);
  store[k] = { v, mn, mx, fixed_, persist_ };
}

// ------------------------------------------------------
// Save all parameters to non-volatile Preferences
// ------------------------------------------------------
static void saveParamsToFlash() {
  LOG_SECTION_START("saveParamsToFlash");

  for (auto& kv : store) {
    if (!kv.second.persist) continue;
    const char* key = kv.first.c_str();
    int value_to_save = kv.second.value;
    kv_set(key, &value_to_save, sizeof(value_to_save), 0);
  }
  LOG_SECTION_END();
}

// ------------------------------------------------------
// Load all parameters from non-volatile Preferences
// ------------------------------------------------------
static void loadParamsFromFlash() {
  LOG_SECTION_START("loadParamsFromFlash");

  for (auto& kv : store) {
    const char* key = kv.first.c_str();
    int loaded_value = 0;
    size_t actual_size = 0;
    int ret = kv_get(key, &loaded_value, sizeof(loaded_value), &actual_size);
    if (ret == MBED_SUCCESS) kv.second.value = loaded_value;
    else kv.second.value = 0;  // TODO the default
  }

  LOG_SECTION_END();
}

// ----------------------------------------------------------
//                SERVO POSE PARAMETERS
// ----------------------------------------------------------
const char* poseKeys[] = {
  // ------------------ Arm1 poses ------------------
  "arm1_0",
  "arm1_90", "arm1_minus90",
  "arm1_0mm", "arm1_5mm", "arm1_10mm", "arm1_15mm",

  // ------------------ Arm2 poses ------------------
  "arm2_0",
  "arm2_90", "arm2_minus90",
  "arm2_0mm", "arm2_5mm", "arm2_10mm", "arm2_15mm",

  // ------------------ Wrist poses ------------------
  "wrist_0", "wrist_90", "wrist_minus90",
  "wrist_0mm", "wrist_5mm", "wrist_10mm", "wrist_15mm",

  // ------------------ Gripper poses ------------------
  "grip1_open", "grip1_close",
  "grip2_open", "grip2_close",

  // ------------------ Base poses ------------------
  "base_0", "base_90", "base_minus90",
  nullptr
};

// ------------------------------------------------------
// Initialize parameter store
// ------------------------------------------------------
void initParamStore() {
  LOG_SECTION_START("initParamStore");

  // ----------------------------------------------------------
  //                SERVO CORE LIMITS
  // ----------------------------------------------------------
  for (auto n : { "arm1", "arm2", "wrist", "grip1", "grip2", "base" }) {
    // Base servo entry (not fixed, not persisted)
    add(n, SERVO_TICKS_MIN, 0, SERVO_TICKS_MAX, false, false);

    // Servo minimum limit
    std::string mn = std::string(n) + "_min";
    add(mn.c_str(), 0, SERVO_TICKS_MIN, SERVO_TICKS_MAX, false, true);

    // Servo maximum limit
    std::string mx = std::string(n) + "_max";
    add(mx.c_str(), SERVO_TICKS_MIN, SERVO_TICKS_MAX, SERVO_TICKS_MAX, true, false);
  }

  for (int i = 0; poseKeys[i] != nullptr; ++i) {
    add(poseKeys[i], 0, SERVO_TICKS_MIN, SERVO_TICKS_MAX);
  }

  // ----------------------------------------------------------
  //                  LOAD FROM FLASH
  // ----------------------------------------------------------
  loadParamsFromFlash();

  LOG_SECTION_END();
}

// ------------------------------------------------------
// Get parameter
// ------------------------------------------------------
int getParamValue(const char* k) {
  String key(k);  // ✅ use Arduino String, not std::string

  auto it = store.find(std::string(k));  // your map is still std::string-keyed
  int val = (it == store.end()) ? 0 : it->second.value;

  if (servoMgr.isServo(key))
    val = servoMgr.getServoTicks(key);

  LOG_VAR2("get param", k, "val", val);
  return val;
}

// overload for convenience
int getParamValue(const std::string& k) {
  return getParamValue(k.c_str());
}

// ------------------------------------------------------
// Set parameter (auto-saves and updates derived values)
// ------------------------------------------------------
void setParamValue(const char* k, int v) {
  // Convert to std::string for lookup
  std::string key(k);

  LOG_SECTION_START_VAR("setParamValue", "key", k);

  auto it = store.find(k);
  if (it == store.end()) {
    LOG_SECTION_END();
    return;
  }
  Param& p = it->second;
  if (!it->second.fixed) {
    v = std::max(p.minv, std::min(p.maxv, v));
    if (p.value != v) {
      p.value = v;
      LOG_VAR2("update derived key", k, "val", v);
      updateDerived(k, v);
      static unsigned long lastSave = 0;
      if (millis() - lastSave > 1000) {
        saveParamsToFlash();
        lastSave = millis();
      }
    }
  } else LOG_VAR2("not changing fixed param", k, "to value", v);
  LOG_SECTION_END();
}

// overload for convenience
void setParamValue(const std::string& k, int v) {
  setParamValue(k.c_str(), v);
}

// ------------------------------------------------------
// Update derived servo angle parameters
// ------------------------------------------------------
void updateDerived(const std::string& k, int v) {
  LOG_SECTION_START_VAR("updateDerived", "key", k.c_str());

  const int ticks_per_rev = 4096;
  const int ticks_per_90 = ticks_per_rev / 4;

  size_t pos = k.find('_');
  if (pos == std::string::npos) {
    LOG_SECTION_END();
    return;
  }
  std::string base = k.substr(0, pos);
  std::string suffix = k.substr(pos + 1);

  auto safe_set = [&](const std::string& key, int val) {
    auto it = store.find(key);
    if (it == store.end()) return;
    val = std::max(it->second.minv, std::min(it->second.maxv, val));
    if (it->second.value != val) it->second.value = val;
    LOG_VAR2("safe set param", key.c_str(), "val", val);
  };

  if (suffix == "0") {
    safe_set(base + "_90", v + ticks_per_90);
    safe_set(base + "_minus90", v - ticks_per_90);
  } else if (suffix == "90") {
    safe_set(base + "_0", v - ticks_per_90);
    safe_set(base + "_minus90", v - 2 * ticks_per_90);
  } else if (suffix == "minus90") {
    safe_set(base + "_0", v + ticks_per_90);
    safe_set(base + "_90", v + 2 * ticks_per_90);
  }

  for (auto sfx : { "0", "90", "minus90" }) {
    std::string key = base + "_" + sfx;
    auto it = store.find(key);
    if (it != store.end()) {
      it->second.value = std::max(0, std::min(SERVO_TICKS_MAX, it->second.value));
    }
  }

  LOG_SECTION_END();
}
