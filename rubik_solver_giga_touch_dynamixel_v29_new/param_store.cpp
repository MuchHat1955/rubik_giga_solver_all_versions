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

extern RBInterface rb;
extern PoseStore pose_store;

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

static void add(const char* k, int v, bool persist_ = true) {
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
    int val = kv.second.value;
    kv_set(key, &val, sizeof(val), 0);
    LOG_PRINTF("saved {%s} = %d\n", key, val);
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
    int loaded = 0;
    size_t actual = 0;
    int ret = kv_get(key, &loaded, sizeof(loaded), &actual);
    if (ret == MBED_SUCCESS) {
      kv.second.value = loaded;
      LOG_PRINTF("loaded {%s} = %d\n", key, loaded);
    }
  }

  LOG_SECTION_END();
}

// ---------------------------------------------------------------------
// Initialize parameter store
// ---------------------------------------------------------------------
void initParamStore() {
  LOG_SECTION_START("initParamStore");

  const char* keys[] = {
    // XY poses
    "xy_zero", "xy_2nd", "xy_3rd", "xy_c1", "xy_c2", "xy_c3", "xy_c4", "xy_c5", "xy_c6",
    // Combined grippers
    "grips_open", "grips_close",
    // Individual grippers
    "grip1_open", "grip1_close", "grip2_open", "grip2_close",
    // Wrist
    "wrist_horiz", "wrist_vert",
    // Base
    "base_0", "base_90", "base_90minus",
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
int getParamValue(const char* k) {
  LOG_SECTION_START_VAR("getParamValue", "key", k ? k : "(null)");

  if (!k || !*k) {
    LOG_SECTION_END();
    return 0;
  }

  auto it = param_store.find(std::string(k));
  if (it == param_store.end()) {
    LOG_PRINTF("not found -> 0\n");
    LOG_SECTION_END();
    return 0;
  }

  int val = it->second.value;
  LOG_PRINTF("get param {%s} = %d\n", k, val);
  LOG_SECTION_END();
  return val;
}

int getParamValue(std::string& k) {
  return getParamValue(k.c_str());
}

// ---------------------------------------------------------------------
// Set parameter
// ---------------------------------------------------------------------
void setParamValue(const char* k, int v) {
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

void setParamValue(std::string& k, int v) {
  setParamValue(k.c_str(), v);
}

// ---------------------------------------------------------------------
// Increment parameter
// ---------------------------------------------------------------------
void incrementParam(const char* k, int delta) {
  LOG_SECTION_START_VAR("incrementParam", "key", k ? k : "(null)");

  if (!k || !*k) {
    LOG_SECTION_END();
    return;
  }

  int current = getParamValue(k);
  int updated = current + delta;
  setParamValue(k, updated);

  LOG_PRINTF("incremented {%s} by {%d -> %d}\n", k, delta, updated);
  LOG_SECTION_END();
}
