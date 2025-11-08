#include <Arduino.h>
#include <map>
#include <string>
#include <algorithm>
#include <KVStore.h>
#include <kvstore_global_api.h>

#include "logging.h"
#include "ui_touch.h"
#include "rb_interface.h"
#include "pose_store.h"
#include "sequence_store.h"
#include "param_store.h"

extern RBInterface rb;
extern PoseStore pose_store;
extern SequenceStore sequence_store;

#ifndef MBED_SUCCESS
#define MBED_SUCCESS 0
#endif

// =====================================================================
//                           RUN ACTION
// =====================================================================
void runAction(const char* key) {
  LOG_SECTION_START_VAR("runAction", "key", key);

  if (!key || !*key) {
    LOG_PRINTF("run action null/empty key ignored\n");
    LOG_SECTION_END();
    return;
  }

  if (strcmp(key, "runAction") == 0) {  // recursion guard
    LOG_PRINTF("run action recursion guard triggered\n");
    LOG_SECTION_END();
    return;
  }

  // --- Pose buttons ---
  if (pose_store.is_button_for_pose(key)) {
    bool ok = pose_store.run_pose_by_button(key);
    LOG_PRINTF("pose move {%s} result {%s}\n", key, ok ? "OK" : "FAIL");
    LOG_SECTION_END();
    return;
  }

  if (sequence_store.is_key_for_sequence(key)) {
    bool ok = sequence_store.run_sequence_by_key(key);
    LOG_PRINTF("sequence move {%s} result {%s}\n", key, ok ? "OK" : "FAIL");
    LOG_SECTION_END();
    return;
  }

  // --- Fallback for unhandled keys ---
  LOG_PRINTF("unhandled action key {%s}\n", key);
  LOG_SECTION_END();
}

// overload for std::string
void runAction(const std::string& k) {
  runAction(k.c_str());
}

// =====================================================================
//                          PARAM STORE CORE
// =====================================================================
struct Param {
  int value{ 0 };
  bool fixed{ false };
  bool persist{ true };
};

static std::map<std::string, Param> param_store;

static void add(const char* k, double v, bool persist_ = true) {
  param_store[k] = { v, false, persist_ };
}

// ---------------------------------------------------------------------
// Save parameters to non-volatile KVStore
// ---------------------------------------------------------------------
static void saveParamsToFlash() {
  LOG_SECTION_START("saveParamsToFlash");

  for (auto& kv : param_store) {
    if (!kv.second.persist) continue;
    const char* key = kv.first.c_str();
    double val = kv.second.value;
    kv_set(key, &val, sizeof(val), 0);
    LOG_PRINTF("saved {%s} = %.2f\n", key, val);
  }

  LOG_SECTION_END();
}

// ---------------------------------------------------------------------
// Load parameters from non-volatile KVStore
// ---------------------------------------------------------------------
static void loadParamsFromFlash() {
  LOG_SECTION_START("loadParamsFromFlash");

  for (auto& kv : param_store) {
    const char* key = kv.first.c_str();
    double loaded = 0;
    size_t actual = 0;
    int ret = kv_get(key, &loaded, sizeof(loaded), &actual);
    if (ret == MBED_SUCCESS) {
      kv.second.value = loaded;
      LOG_PRINTF("loaded {%s} = %.2f\n", key, loaded);
    }
  }

  LOG_SECTION_END();
}

// ---------------------------------------------------------------------
// Initialize parameter store
// ---------------------------------------------------------------------
void initParamStore() {
  LOG_SECTION_START("initParamStore");

  // TODO below needs to mach the pose store that has to match the UI

  const char* keys[] = {
    // XY poses
    "y_zero_param", "y_1st_param", "y_2nd_param", "y_3rd_param", "x_c2_param", "x_c3_param", "x_center_param", "x_left_param", "x_right_param",
    // Combined grippers
    "grippers_open_param", "grippers_close_param",
    // Individual grippers
    "gripper1_open_param", "gripper1_close_param", "gripper2_open_param", "gripper2_close_param",
    // Wrist
    "wrist_vert_param", "wrist_horiz_left_param", "wrist_horiz_left_param",
    // Base
    "base_front_param", "base_left_param", "base_right_param",
    // end
    nullptr
  };

  for (int i = 0; keys[i] != nullptr; ++i)
    add(keys[i], 0);

  loadParamsFromFlash();
  LOG_SECTION_END();
}

// ---------------------------------------------------------------------
// Get parameter
// ---------------------------------------------------------------------

double getParamValue(const char* k) {
  //LOG_SECTION_START_VAR("getParamValue", "key", k ? k : "(null)");

  if (!k || !*k) {
    //LOG_SECTION_END();
    return PARAM_VAL_NA;
  }

  auto it = param_store.find(std::string(k));
  if (it == param_store.end()) {
    LOG_PRINTF("[!] param {%s} not found in the store\n", k);
    //LOG_SECTION_END();
    return PARAM_VAL_NA;
  }

  int val = it->second.value;
  LOG_PRINTF("param {%s} result from the store {%.2f}\n", k, val);
  //LOG_SECTION_END();
  return val;
}

double getParamValue(std::string& k) {
  return getParamValue(k.c_str());
}

// ---------------------------------------------------------------------
// Set parameter
// ---------------------------------------------------------------------
void setParamValue(const char* k, double v) {
  LOG_SECTION_START_VAR("setParamValue", "key", k ? k : "(null)");

  if (!k || !*k) {
    LOG_SECTION_END();
    return;
  }

  auto it = param_store.find(std::string(k));
  if (it == param_store.end()) {
    LOG_PRINTF("param not found {%s}\n", k);
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
    }
    LOG_PRINTF("updated param {%s} = %d\n", k, v);
  } else {
    LOG_PRINTF("no change for {%s} (still %d)\n", k, v);
  }

  LOG_SECTION_END();
}

void setParamValue(std::string& k, double v) {
  setParamValue(k.c_str(), v);
}

// ---------------------------------------------------------------------
// Increment parameter
// Use the following from the pose store
//    bool PoseStore::increment_pose_param(const char* param_name, int units, double& new_value_ref);
//    void PoseStore::set_pose_val_from_param(const char* param_name, double val);
//    int PoseStore::is_param_for_pose(const char* btn_key) const;

// ---------------------------------------------------------------------
void incrementParam(const char* k, int delta) {
  LOG_SECTION_START_VAR("incrementParam", "key", k ? k : "(null)");

  if (!k || !*k) {
    LOG_SECTION_END();
    return;
  }
  if (!pose_store.is_param_for_pose(k)) {
    LOG_PRINTF("[!] increment param err no pose found{%s}\n", k);
    return;
  }
  double p1 = 0;
  if (!pose_store.get_pose_params(k, &p1)) {
    LOG_PRINTF("[!] increment param err cannot get pose params{%s}\n", k);
    return;
  }
  pose_store.increment_pose_param(k, delta, p1);
  pose_store.set_pose_params(k, p1);

  LOG_PRINTF("incremented {%s} by {%d -> %d}\n", k, delta, p1);
  LOG_SECTION_END();
}
