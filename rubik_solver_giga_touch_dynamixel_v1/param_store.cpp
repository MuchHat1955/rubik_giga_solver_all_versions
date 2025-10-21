// param_store.cpp
#include "param_store.h"
#include <map>
#include <string>
#include <algorithm>

struct Param {
  int value{ 0 }, minv{ 0 }, maxv{ 0 };
  bool fixed{ false };
};
static std::map<std::string, Param> store;

static void add(const char* k, int v, int mn, int mx, bool fx = false) {
  store[k] = { v, mn, mx, fx };
}
static void updateDerived() {
  int base0 = getParamValue("base_0");
  int base90 = getParamValue("base_90");

  auto ang2 = [&](int a) -> int {
    long d = (long)base90 - (long)base0;
    long t = base0 + d * a / 90;
    if (t < 0) t = 0;
    if (t > 4095) t = 4095;
    return (int)t;
  };

  store["base_minus60"].value = ang2(-60);
  store["base_minus30"].value = ang2(-30);
  store["base_30"].value = ang2(30);
  store["base_60"].value = ang2(60);
}


void initParamStore() {
  for (auto n : { "arm1", "arm2", "wrist", "grip1", "grip2", "base" }) {
    add(n, 1024, 0, 3072);
    std::string mn = std::string(n) + "_min";
    add(mn.c_str(), 0, 0, 3072, true);
    std::string mx = std::string(n) + "_max";
    add(mx.c_str(), 3072, 0, 3072, true);
  }
  for (auto n : { "arm1_0", "arm1_1", "arm1_2", "arm1_read1",
                  "arm2_0", "arm2_1", "arm2_2", "arm2_read1",
                  "wrist_0", "wrist_90", "wrist_180", "wrist_minus90",
                  "base_0", "base_90", "base_180", "base_minus90",
                  "base_minus60", "base_minus30", "base_30", "base_60" })
    add(n, 1024, 0, 3072);

  add("grip_open", 1024, 0, 3072);
  add("grip_closed", 1024, 0, 3072);

  // tick calibration (optional future use)
  add("arm1_0_tick", 2048, 0, 4095);
  add("arm1_plus_tick", 2560, 0, 4095);
  add("arm1_plus_deg", 90, 10, 180, true);
  add("arm2_0_tick", 2048, 0, 4095);
  add("arm2_plus_tick", 2560, 0, 4095);
  add("arm2_plus_deg", 90, 10, 180, true);
  add("wrist_0_tick", 2048, 0, 4095);
  add("wrist_plus_tick", 2560, 0, 4095);
  add("wrist_plus_deg", 90, 10, 180, true);

  updateDerived();
}

int getParamValue(const std::string& k) {
  auto it = store.find(k);
  return it == store.end() ? 0 : it->second.value;
}
void setParamValue(const std::string& k, int v) {
  auto it = store.find(k);
  if (it == store.end()) return;
  Param& p = it->second;
  v = std::max(p.minv, std::min(p.maxv, v));
  if (p.value != v) {
    p.value = v;
    if (k == "base_0" || k == "base_90") updateDerived();
  }
}
