// servos.cpp
#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "param_store.h"
#include "logging.h"
#include "servos.h"

// ---- EDIT to match your wiring ----
#define DXL_SERIAL Serial1
#define DXL_DIR_PIN 10
#define DXL_BAUD 57600

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

struct {
  const char* name;
  uint8_t id;
  bool invert;
} smap[] = {
  { "arm1", 11, 0 }, { "arm2", 12, 0 }, { "wrist", 13, 0 }, { "grip1", 14, 0 }, { "grip2", 15, 0 }, { "base", 16, 0 }, { nullptr, 0, 0 }
};

static int getId(const std::string& n) {
  for (int i = 0; smap[i].name; ++i)
    if (n == smap[i].name) return smap[i].id;
  return -1;
}

static int getInverted(const std::string& n) {
  for (int i = 0; smap[i].name; ++i)
    if (n == smap[i].name) return smap[i].invert;
  return 0;
}

bool hasDynamixel = false;

void dynamixel_begin() {
  LOG_SECTION_START("dynamixel_begin");

  DXL_SERIAL.begin(DXL_BAUD);
  dxl.begin(DXL_BAUD);
  bool ok = (dxl.ping(1) || dxl.ping(11) || dxl.ping(12) || dxl.ping(13) || dxl.ping(14) || dxl.ping(15) || dxl.ping(16));

  if (!ok) {
    LOG_VAR("servos", "none found");
    LOG_SECTION_END();
    return;
  }

  hasDynamixel = true;
  dxl.setPortProtocolVersion(2.0);

  LOG_SECTION_END();
}

int readServoTicks(const std::string& name) {
  if (!hasDynamixel) return 0;
  int id = getId(name);
  if (id < 0) return 0;
  return dxl.getPresentPosition(id);
}

int getServoTicksForAngle(const std::string& servo, float f_angle_deg) {
  int z = getParamValue(servo + "_0");
  int min = getParamValue(servo + "_min");
  int max = getParamValue(servo + "_max");
  bool inv = getInverted(servo);

  long t = 0;
  if (inv)
    t = z + (long)((f_angle_deg * (float)XL430_360DEG_TICKS) / 360.0);
  else
    t = z - (long)((f_angle_deg * (float)XL430_360DEG_TICKS) / 360.0);

  if (t < 0) t = 0;
  if (t > 4095) t = 4095;
  if (t > max) t = max;
  if (t < min) t = min;

  return (int)t;
}

void rotateToAngle(const std::string& name, float angle_deg) {
  LOG_SECTION_START_VAR("rotateToAngle", "servo", name.c_str());

  if (!hasDynamixel) {
    LOG_SECTION_END();
    return;
  }

  int t = getServoTicksForAngle(name, angle_deg);
  int id = getId(name);
  if (id < 10) {
    LOG_SECTION_END();
    return;
  }

  dxl.setGoalPosition(id, (int)t);

  LOG_SECTION_END();
}
