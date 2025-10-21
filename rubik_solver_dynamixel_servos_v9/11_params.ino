/********************************************************************
 * PARAMS ARE NUMBER STORED IN EEPROM LIKE SERVO LIMITS, SERVO TICKS FOR POSES ETC
 ********************************************************************/
class Param {
public:
  int value{ 0 }, minv{ 0 }, maxv{ 0 };
  bool fixed{ false };  // for *_min/_max style params
  int fixedMin{ 0 }, fixedMax{ 0 };

  Param() {}
  Param(int v, int mn, int mx)
    : value(v), minv(mn), maxv(mx) {}
  Param(int v, int mn, int mx, bool fx)
    : value(v), minv(mn), maxv(mx), fixed(fx), fixedMin(mn), fixedMax(mx) {}
};

Preferences prefs;
std::map<String, Param> paramStore;
bool paramsLoaded = false;

void initParamStore() {
  // core servos
  paramStore["arm1"] = Param(1024, 0, 3072);
  paramStore["arm1_min"] = Param(0, 0, 3072, true);
  paramStore["arm1_max"] = Param(3072, 0, 3072, true);

  paramStore["arm2"] = Param(1024, 0, 3072);
  paramStore["arm2_min"] = Param(0, 0, 3072, true);
  paramStore["arm2_max"] = Param(3072, 0, 3072, true);

  paramStore["wrist"] = Param(1024, 0, 3072);
  paramStore["wrist_min"] = Param(0, 0, 3072, true);
  paramStore["wrist_max"] = Param(3072, 0, 3072, true);

  paramStore["grip1"] = Param(1024, 0, 3072);
  paramStore["grip1_min"] = Param(0, 0, 3072, true);
  paramStore["grip1_max"] = Param(3072, 0, 3072, true);

  paramStore["grip2"] = Param(1024, 0, 3072);
  paramStore["grip2_min"] = Param(0, 0, 3072, true);
  paramStore["grip2_max"] = Param(3072, 0, 3072, true);

  paramStore["base"] = Param(1024, 0, 3072);
  paramStore["base_min"] = Param(0, 0, 3072, true);
  paramStore["base_max"] = Param(3072, 0, 3072, true);

  // poses
  paramStore["arm1_0"] = Param(1024, 0, 3072);
  paramStore["arm1_1"] = Param(1024, 0, 3072);
  paramStore["arm1_2"] = Param(1024, 0, 3072);
  paramStore["arm1_read1"] = Param(1024, 0, 3072);
  paramStore["arm1_read2"] = Param(1024, 0, 3072);

  paramStore["arm2_0"] = Param(1024, 0, 3072);
  paramStore["arm2_1"] = Param(1024, 0, 3072);
  paramStore["arm2_2"] = Param(1024, 0, 3072);
  paramStore["arm2_read1"] = Param(1024, 0, 3072);
  paramStore["arm2_read2"] = Param(1024, 0, 3072);

  paramStore["wrist_0"] = Param(1024, 0, 3072);
  paramStore["wrist_90"] = Param(1024, 0, 3072);
  paramStore["wrist_180"] = Param(1024, 0, 3072);
  paramStore["wrist_minus90"] = Param(1024, 0, 3072);

  paramStore["grip_0"] = Param(1024, 0, 3072);
  paramStore["grip_1"] = Param(1024, 0, 3072);

  paramStore["base_0"] = Param(1024, 0, 3072);
  paramStore["base_90"] = Param(1024, 0, 3072);
  paramStore["base_180"] = Param(1024, 0, 3072);
  paramStore["base_minus90"] = Param(1024, 0, 3072);

  // poses for reading edge cells
  paramStore["base_minus60"] = Param(1024, 0, 3072);
  paramStore["base_minus30"] = Param(1024, 0, 3072);
  paramStore["base_30"] = Param(1024, 0, 3072);
  paramStore["base_60"] = Param(1024, 0, 3072);
}

int angle_to_ticks(int zero_tick, int ninety_tick, int angle_deg) {
  // Integer linear interpolation between 0° and +90°
  // Handles direction automatically (if ninety_tick < zero_tick)
  long delta = (long)ninety_tick - (long)zero_tick;
  long tick = zero_tick + (delta * angle_deg) / 90L;

  // Clamp within Dynamixel range (0–4095)
  if (tick < 0) tick = 0;
  if (tick > 4095) tick = 4095;

  return (int)tick;
}

// reading params are updated when base_0 is updated, to keep the UI simpler
void updateColorReadParams() {
  int base_0 = getParamValue("base_0");
  int base_90 = getParamValue("base_90");

  setParamValue("base_minus60", angle_to_ticks(base_0, base_90, -60));
  setParamValue("base_minus30", angle_to_ticks(base_0, base_90, -60));
  setParamValue("base_30", angle_to_ticks(base_0, base_90, 30));
  setParamValue("base_60", angle_to_ticks(base_0, base_90, 60));
}

void loadParamsOnce() {
  if (paramsLoaded) return;
  prefs.begin("rubik", true);
  for (auto &kv : paramStore) {
    auto &k = kv.first;
    auto &p = kv.second;
    if (p.fixed) {
      p.value = prefs.getInt(k.c_str(), p.value);
    } else {
      p.value = prefs.getInt((k + "_value").c_str(), p.value);
      p.minv = prefs.getInt((k + "_min").c_str(), p.minv);
      p.maxv = prefs.getInt((k + "_max").c_str(), p.maxv);
    }
  }
  prefs.end();
  paramsLoaded = true;
}

void saveParamIfChanged(const String &k) {
  auto it = paramStore.find(k);
  if (it == paramStore.end()) return;
  Param &p = it->second;

  prefs.begin("rubik", false);
  if (p.fixed) {
    int sv = prefs.getInt(k.c_str(), INT_MIN);
    if (sv != p.value) prefs.putInt(k.c_str(), p.value);
  } else {
    String kvV = k + "_value";
    String kvN = k + "_min";
    String kvX = k + "_max";
    int sv = prefs.getInt(kvV.c_str(), INT_MIN);
    int sn = prefs.getInt(kvN.c_str(), INT_MIN);
    int sx = prefs.getInt(kvX.c_str(), INT_MIN);
    if (sv != p.value) prefs.putInt(kvV.c_str(), p.value);
    if (sn != p.minv) prefs.putInt(kvN.c_str(), p.minv);
    if (sx != p.maxv) prefs.putInt(kvX.c_str(), p.maxv);
  }
  if (sv != p.value) updateColorReadParams();
  prefs.end();
}

int getParamValue(const String &k) {
  loadParamsOnce();
  auto it = paramStore.find(k);
  return (it == paramStore.end()) ? 0 : it->second.value;
}
int getParamMin(const String &k) {
  loadParamsOnce();
  auto it = paramStore.find(k);
  return (it == paramStore.end()) ? 0 : it->second.minv;
}
int getParamMax(const String &k) {
  loadParamsOnce();
  auto it = paramStore.find(k);
  return (it == paramStore.end()) ? 0 : it->second.maxv;
}

void setParamValue(const String &k, int v) {
  loadParamsOnce();
  auto it = paramStore.find(k);
  if (it == paramStore.end()) return;
  Param &p = it->second;
  if (p.fixed) {
    if (v < p.fixedMin) v = p.fixedMin;
    if (v > p.fixedMax) v = p.fixedMax;
  } else {
    if (v < p.minv) v = p.minv;
    if (v > p.maxv) v = p.maxv;
  }
  if (p.value != v) {
    p.value = v;
    saveParamIfChanged(k);
    // keep linked mins/maxs in sync for base param
    if (k.endsWith("_min")) {
      String base = k.substring(0, k.length() - 4);
      if (paramStore.count(base)) {
        paramStore[base].minv = v;
        saveParamIfChanged(base);
      }
    } else if (k.endsWith("_max")) {
      String base = k.substring(0, k.length() - 4);
      if (paramStore.count(base)) {
        paramStore[base].maxv = v;
        saveParamIfChanged(base);
      }
    }
  }
}