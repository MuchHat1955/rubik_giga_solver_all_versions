// param_store.cpp
#include <arduino.h>
#include <map>
#include <string>
#include <algorithm>
#include <kvstore.h>
#include <kvstore_global_api.h>

#include "logging.h"
#include "param_store.h"
#include "pose_store.h"
#include "ui_touch.h"
#include "rb_interface.h"

extern RBInterface rb;
extern PoseStore pose_store;

#ifndef MBED_SUCCESS
#define MBED_SUCCESS 0
#endif


// ----------------------------------------------------------
//  RUN ACTION //TODO MOVE IN THE RIGHT FILE
// ----------------------------------------------------------
/*
  bool is_pose(const char *name) const;
  bool is_button_for_pose(const char *btn_key) const;
  bool get_pose_params(const char *name, double *p1, double *p2, String *type = nullptr) const;
  bool set_pose_params(const char *name, double p1, double p2);
  bool run_pose(const char *name);
  bool run_pose_by_button(const char *btn_key);
  bool increment_pose_param(const char *name, int units, double &new_value_ref);
  bool is_at_pose(const char *name, double tol_mm = 0.5, double tol_deg = 1.0);
*/

void runAction(const char* key) {
  LOG_SECTION_START_VAR("runAction", "key", key);

  if (!key || !*key) {
    LOG_PRINTF("run action null/empty key ignored\n");
    return;
  }

  // Defensive: prevent recursive self-call crash
  if (strcmp(key, "runAction") == 0) {
    LOG_PRINTF("run action recursion guard triggered\n");
    return;
  }

  // --- Pose buttons (e.g., "arm1_0_btn", "v_pose_r1") ---
  if (pose_store.is_button_for_pose(key)) {
    bool ok = pose_store.run_pose_by_button(key);
    // reflectUIForKey(key); TODO
    LOG_PRINTF("pose move{%s} result{%s}", key, ok ? "OK" : "FAIL");
    LOG_SECTION_END();
    return;
  }

  /* TODO
  // --- Sequence buttons ---
  if (sequencesparam_store.is_button_for_sequence(key)) {
    bool ok = moveToSequence(key);
    // reflectUIForKey(String(key) + "_btn"); TODO
    LOG_VAR2("plain pose move", key, "", ok ? "OK" : "FAIL");
    LOG_SECTION_END();
    return;
  }
  */

  // --- Fallback for sequences or other custom actions ---
  LOG_PRINTF("unhandled action key {%s}\n", key);
  LOG_SECTION_END();
}

// overload for convenience
void runAction(const std::string& k) {
  runAction(k.c_str());
}

// ----------------------------------------------------------
//                    PARAM
// ----------------------------------------------------------

struct ParamRecord {
  char id[24];
  int value;
};

struct ParamStorage {
  ParamRecord records[64];
  int count;
};

struct Param {
  int value{ 0 };
  bool fixed{ false };  // the min and max for the min and max are fixed not changeable in UI
  bool persist{ true };
};
std::map<std::string, Param> param_store;

static void add(const char* k, int v, bool persist_ = true) {
  param_store[k] = { v, persist_ };
}

// ------------------------------------------------------
// Save all parameters to non-volatile Preferences
// ------------------------------------------------------
static void saveParamsToFlash() {
  LOG_SECTION_START("saveParamsToFlash");

  for (auto& kv : param_store) {
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

  for (auto& kv : param_store) {
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
// SERVO POSE PARAMETERS (TODO THIS NEEDS TO MATCH POSES STORE)
// ----------------------------------------------------------
const char* paramKeys[] = {
  // XY poses
  "xy_zero", "xy_2nd", "xy_3rd", "xy_c1", "xy_c2", "xy_c3", "xy_c4", "xy_c5", "xy_c6",  //

  // Combined grippers
  "grips_open", "grips_close",  //

  // Individual gripper 1
  "grip1_open", "grip1_close",  //

  // Individual gripper 2
  "grip2_open", "grip2_close",  //

  // Wrist poses
  "wrist_horiz", "wrist_vert",  //

  // Base rotation
  "base_0", "base_90", "base_90minus",  //

  // ------------------ Terminator ------------------
  nullptr
};

// ------------------------------------------------------
// Initialize parameter store
// ------------------------------------------------------
void initParamStore() {
  LOG_SECTION_START("initParamStore");

  // ----------------------------------------------------------
  //                USE THE LIST ABOVE
  // ----------------------------------------------------------
  for (int i = 0; paramKeys[i] != nullptr; ++i) {
    add(paramKeys[i], 0);
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

  auto it = param_store.find(std::string(k));  // your map is still std::string-keyed
  if (it == param_store.end()) return 0;
  int val = (it == param_store.end()) ? 0 : it->second.value;

  LOG_PRINTF("get param{%s} val{%d}\n", k, val);
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

  auto it = param_store.find(k);
  if (it == param_store.end()) {
    LOG_SECTION_END();
    return;
  }
  Param& p = it->second;
  if (p.value != v) {
    p.value = v;

    static unsigned long lastSave = 0;
    if (millis() - lastSave > 1000) {
      saveParamsToFlash();
      lastSave = millis();
      LOG_PRINTF("saving param key{%s} val{%d}\n", k, v);
    }
  }
  LOG_SECTION_END();
}

// overload for convenience
void setParamValue(const std::string& k, int v) {
  setParamValue(k.c_str(), v);
}

void incrementParam(const char* k, int v) {
  // TODO
}
