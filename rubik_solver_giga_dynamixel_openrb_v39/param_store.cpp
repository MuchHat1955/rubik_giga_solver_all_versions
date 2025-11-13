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
//                          PARAM STORE CORE
// =====================================================================
struct Param {
  double value{ 0 };
  bool persist{ true };
};

static std::map<std::string, Param> param_store;

static void add(const char* k, double v, bool persist_ = true) {
  param_store[k] = { v, persist_ };
}

// ---------------------------------------------------------------------
// Save parameters to non-volatile KVStore
// ---------------------------------------------------------------------
static void saveParamsToFlash() {
  LOG_SECTION_START_PARAM("saveParamsToFlash");

  for (auto& kv : param_store) {
    if (!kv.second.persist) continue;
    const char* key = kv.first.c_str();
    double val = kv.second.value;
    kv_set(key, &val, sizeof(val), 0);
    LOG_PRINTF_PARAM("saving to flash | key {%s} | val {%.2f}\n", key, val);
  }

  LOG_SECTION_END_PARAM();
}

// ---------------------------------------------------------------------
// Load parameters from non-volatile KVStore
// ---------------------------------------------------------------------
static void loadParamsFromFlash() {
  LOG_SECTION_START_PARAM("loadParamsFromFlash");

  for (auto& kv : param_store) {
    const char* key = kv.first.c_str();
    double loaded = 0;
    size_t actual = 0;
    int ret = kv_get(key, &loaded, sizeof(loaded), &actual);
    if (ret == MBED_SUCCESS) {
      kv.second.value = loaded;
      LOG_PRINTF_PARAM("loading from flash | key {%s} | val {%.2f}\n", key, loaded);
    }
  }
  updatePoseStoreFromParamStore();
  LOG_SECTION_END_PARAM();
}

// ---------------------------------------------------------------------
// Initialize parameter store
// ---------------------------------------------------------------------
void initParamStore() {
  LOG_SECTION_START_PARAM("initParamStore");

  // TODO below MUST mach the pose store that has to MATCH the UI
  const char* keys[] = {
    // XY poses
    "y zero", "y 1st", "y 2nd", "y 3rd", "y c2", "y c3", "x c2", "x c3", "x center", "x left", "x right",
    // Combined grippers
    "grippers open", "grippers close",
    // Individual grippers
    "gripper 1 open", "gripper 1 close", "gripper 2 open", "gripper 2 close",
    // Wrist
    "wrist vert", "wrist horiz right", "wrist horiz left",
    // Base
    "base front", "base left", "base right",
    // end
    nullptr
  };

  for (int i = 0; keys[i] != nullptr; ++i)
    add(keys[i], 0);

  loadParamsFromFlash();
  LOG_SECTION_END_PARAM();
}

// ---------------------------------------------------------------------
// Get parameter
// ---------------------------------------------------------------------

double getParamValue(const char* k) {
  // LOG_PRINTF_PARAM("get param value for | key {%s}\n", k ? k : "(null)");

  if (!k || !*k) {
    return PARAM_VAL_NA;
  }

  auto it = param_store.find(std::string(k));
  if (it == param_store.end()) {
    // LOG_PRINTF_PARAM("[!] param not found in the param store | key {%s} \n", k);
    return PARAM_VAL_NA;
  }

  double val = it->second.value;
  LOG_PRINTF_PARAM("param was found in the param store | key {%s} | val {%.2f}\n", k, val);
  return val;
}

double getParamValue(std::string& k) {
  return getParamValue(k.c_str());
}

// ---------------------------------------------------------------------
// Set parameter
// ---------------------------------------------------------------------
void setParamValue(const char* key, double v) {
  LOG_PRINTF_PARAM("setParamValue | key {%s} | val {%.2f}\n", key ? key : "(null)", v);

  if (!key || !*key) {
    return;
  }

  // Convert to String for safe manipulation
  String key_str = key;
  const char* key_final = key_str.c_str();

  auto it = param_store.find(std::string(key_final));
  if (it == param_store.end()) {
    LOG_PRINTF_PARAM("param not found {%s}\n", key_final);
    return;
  }

  Param& p = it->second;
  LOG_PRINTF_PARAM("param found in the param store with | param {%s} | val {%.2f}\n", key_final, p.value);
  if (p.value != v) {
    p.value = v;
    LOG_PRINTF_PARAM("set new value in the param store | param {%s} | val {%.2f}\n", key_final, p.value);
    static unsigned long lastSave = 0;
    if (millis() - lastSave > 300) {
      LOG_PRINTF_PARAM("saving to flash all params store because of | param {%s} | val {%.2f}\n", key_final, p.value);
      saveParamsToFlash();
      lastSave = millis();
    }
  } else {
    LOG_PRINTF_PARAM("unchanged no need to save param {%s} | val {%.2f}\n", key_final, v);
  }
}

void setParamValue(std::string& k, double v) {
  setParamValue(k.c_str(), v);
}

// ---------------------------------------------------------------------
// Increment parameter
// Use the following from the pose store
//    bool PoseStore::increment_in_pose_store(const char* param_name, int units, double& new_value_ref);
//    void PoseStore::set_pose_val_from_param(const char* param_name, double val);
//    int PoseStore::is_param_for_pose(const char* btn_key) const;

// ---------------------------------------------------------------------
void increment_pose_param_in_pose_and_param_stores(const char* k, int delta) {

  // it does not increment unless is a pose and in that case
  // it will have it to the pose store also

  LOG_PRINTF_PARAM("increment_pose_param_in_pose_and_param_stores key {%s}", k ? k : "(null)\n");

  if (!k || !*k) {
    return;
  }
  if (!pose_store.is_param_for_pose(k)) {
    LOG_PRINTF_PARAM("[!] increment param err no pose found {%s}\n", k);
    return;
  }
  double p1 = 0;
  if (!pose_store.get_pose_params(k, &p1)) {
    LOG_PRINTF_PARAM("[!] increment param err cannot get pose params {%s}\n", k);
    return;
  }
  pose_store.increment_in_pose_store(k, delta, p1);
  pose_store.save_pose_in_param_store(k, p1);

  LOG_PRINTF_PARAM("done incrementing {%s} | by {%d} | to {%.2f}\n", k, delta, p1);
}
