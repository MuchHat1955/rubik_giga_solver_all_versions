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

#ifndef MBED_SUCCESS
#define MBED_SUCCESS 0
#endif

// ----------------------------------------------------------
//                    STUBS
// ----------------------------------------------------------

int getParamValue(const char *name) {
  return 0;
}
void setParamValue(const char *name, int val) {
}
const char *getStatusText(const char *name) {
  return "na";
}
const char *getStatusColor(const char *name) {
  return "green";
}

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
  bool fixed{ false };
};
static std::map<std::string, Param> store;

static void add(const char* k, int v, int mn, int mx, bool fx = false) {
  store[k] = { v, mn, mx, fx };
}

// ======================================================
// Save all parameters to non-volatile Preferences
// ======================================================
static void saveParamsToFlash() {
  LOG_SECTION_START("saveParamsToFlash");

  for (auto& kv : store) {
    const char* key = kv.first.c_str();
    int value_to_save = kv.second.value;
    kv_set(key, &value_to_save, sizeof(value_to_save), 0);
  }

  LOG_SECTION_END();
}

// ======================================================
// Load all parameters from non-volatile Preferences
// ======================================================
static void loadParamsFromFlash() {
  LOG_SECTION_START("loadParamsFromFlash");

  for (auto& kv : store) {
    const char* key = kv.first.c_str();
    int loaded_value = 0;
    size_t actual_size = 0;
    int ret = kv_get(key, &loaded_value, sizeof(loaded_value), &actual_size);
    if (ret == MBED_SUCCESS) kv.second.value = loaded_value;
    else kv.second.value = -1;
  }

  LOG_SECTION_END();
}

// ======================================================
// Initialize parameter store
// ======================================================
void initParamStore() {
  LOG_SECTION_START("initParamStore");

  for (auto n : { "arm1", "arm2", "wrist", "grip1", "grip2", "base" }) {
    add(n, 1024, 0, 3072);
    std::string mn = std::string(n) + "_min";
    add(mn.c_str(), 0, 0, 3072, true);
    std::string mx = std::string(n) + "_max";
    add(mx.c_str(), 3072, 0, 3072, true);
  }
  for (auto n : { "arm1_0", "arm1_mid", "arm1_high",
                  "arm1_90", "arm1_minus90",
                  "arm2_0", "arm2_mid", "arm2_high",
                  "arm2_90", "arm2_minus90",
                  "wrist_0", "wrist_90", "wrist_minus90",
                  "grip1_open", "grip1_close", "grip2_open", "grip2_close",
                  "base_0", "base_90", "base_minus90" })
    add(n, 1024, 0, 3072);

  loadParamsFromFlash();

  LOG_SECTION_END();
}

// ======================================================
// Get parameter
// ======================================================
int getParamValue(const std::string& k) {
  auto it = store.find(k);
  return it == store.end() ? 0 : it->second.value;
}

// ======================================================
// Set parameter (auto-saves and updates derived values)
// ======================================================
void setParamValue(const std::string& k, int v) {
  LOG_SECTION_START_VAR("setParamValue", "key", k.c_str());

  auto it = store.find(k);
  if (it == store.end()) {
    LOG_SECTION_END();
    return;
  }
  Param& p = it->second;
  v = std::max(p.minv, std::min(p.maxv, v));
  if (p.value != v) {
    p.value = v;
    updateDerived(k, v);
    static unsigned long lastSave = 0;
    if (millis() - lastSave > 1000) {
      saveParamsToFlash();
      lastSave = millis();
    }
  }

  LOG_SECTION_END();
}

// ======================================================
// Update derived servo angle parameters
// ======================================================
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
      it->second.value = std::max(0, std::min(4095, it->second.value));
    }
  }

  LOG_SECTION_END();
}
