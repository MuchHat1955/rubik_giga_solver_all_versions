/*

// ================= PARAM CLASS FOR ANY GLOBAL VAR IN MENUS =================

class Param {
private:
  const char* _name;
  long* _ptr;
  long _min_val, _max_val, _def_val;
  bool _initialized;

public:
  Param(const char* n, long* p, long mn, long mx, long dv)
    : _name(n), _ptr(p), _min_val(mn), _max_val(mx), _def_val(dv), _initialized(false) {
    *_ptr = _def_val;
  }

  const char* getName() const {
    return _name;
  }
  long getMin() const {
    return _min_val;
  }
  long getMax() const {
    return _max_val;
  }

  long get() {
    if (!_initialized) {
      String key = "var_" + String(_name);
      long stored = prefs.getLong(key.c_str(), _def_val);
      if (stored < _min_val || stored > _max_val) {
        stored = _def_val;
      }
      *_ptr = stored;
      _initialized = true;
    }
    return *_ptr;
  }

  void set(long v) {
    if (v < _min_val) v = _min_val;
    if (v > _max_val) v = _max_val;
    if (*_ptr != v) {
      *_ptr = v;
      String key = "var_" + String(_name);
      prefs.putLong(key.c_str(), v);
    }
  }
};

// ================= PARAM REGISTRY =================
long servo1_val, servo2_val, servo3_val, servo4_val, servo5_val, servo6_val;

Param params[] = {
  Param("arm1", &servo1_val, 0, 180, 90),
  Param("arm2", &servo2_val, 0, 180, 90),
  Param("wrist", &servo3_val, 0, 180, 90),
  Param("grip1", &servo3_val, 0, 180, 90),
  Param("grip2", &servo3_val, 0, 180, 90),
  Param("base", &servo3_val, 0, 180, 90)
};
const int param_count = sizeof(params) / sizeof(params[0]);

Param* findParam(const char* name) {
  for (int i = 0; i < param_count; i++) {
    if (strcmp(name, params[i].getName()) == 0) {
      return &params[i];
    }
  }
  return nullptr;
}

long getValue(const char* name) {
  Param* p = findParam(name);
  return p ? p->get() : 0;
}

void setValue(const char* name, long v) {
  Param* p = findParam(name);
  if (p) p->set(v);
}

// ================= PRESETS =================
class Preset {
public:
  const char* name;
  long _arm1, _arm2, _wrist, _grip1, _grip2, _base;

  // Constructor taking 6 individual longs
  Preset(const char* n, long v0, long v1, long v2, long v3, long v4, long v5) {
    name = n;
    _arm1 = v0;
    _arm2 = v1;
    _wrist = v2;
    _grip1 = v3;
    _grip2 = v4;
    _base = v5;
  }
};

// ================= PRESETS REGISTRY =================

#define _NA 999999  // it means that servo value stays unchanged

//TODOTODO
Preset presets[] = {
  Preset("zero", 0, 0, 0, 0, 0, 0),
  Preset("z_level_1", 0, 0, 0, 0, 0, 0),
  Preset("z_level_2", 0, 0, 0, 0, 0, 0),
  Preset("wrist_0", 0, 0, 0, 0, 0, 0),
  Preset("wrist_180", 0, 0, 0, 0, 0, 0),
  Preset("wrist_90", 0, 0, 0, 0, 0, 0),
  Preset("wrist_minus90", 0, 0, 0, 0, 0, 0),
  Preset("grip_open", 0, 0, 0, 0, 0, 0),
  Preset("grip_closed", 0, 0, 0, 0, 0, 0),
  Preset("base_0", 0, 0, 0, 0, 0, 0),
  Preset("base_90", 0, 0, 0, 0, 0, 0),
  Preset("base_minus90", 0, 0, 0, 0, 0, 0),
  Preset("base_180", 0, 0, 0, 0, 0, 0),
};

int preset_count = sizeof(presets) / sizeof(presets[0]);

void loadPreset(Preset& p) {
  for (int i = 0; i < 6; i++) {
    char key[32];
    sprintf(key, "preset_%s_s%d", p.name, i + 1);
    p.values[i] = prefs.getLong(key, 0);
  }
}
*/
