#include "utils.h"
#include <Dynamixel2Arduino.h>

// -------------------------------------------------------------------
//                         UTILS - IMPLEMENTATION
// -------------------------------------------------------------------

// ---------------- Numeric helpers ----------------
double clamp(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi ? hi : v);
}

double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}

double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

// -------------------------------------------------------------------
//              PROFILE & TIMING UTILS (Dynamic Wait)
// -------------------------------------------------------------------
const float PV_UNIT_RPM = 0.229f;
const float TICKS_PER_REV = 4096.0f;
const float TIME_SAFETY_FACTOR = 1.20f;
const uint32_t MIN_WAIT_MS = 120;
const uint32_t EXTRA_SETTLE_MS = 50;

float pvToTicksPerSec(int pvLSB) {
  float rpm = (pvLSB <= 0 ? 0.0f : pvLSB * PV_UNIT_RPM);
  float rps = rpm / 60.0f;
  float tps = rps * TICKS_PER_REV;
  return max(tps, 1.0f);
}

uint32_t estimateTravelTimeMs(uint8_t id, int deltaTicks) {
  extern Dynamixel2Arduino dxl;
  int pv = dxl.readControlTableItem(ControlTableItem::PROFILE_VELOCITY, id);
  float tps = pvToTicksPerSec(pv);
  float moveMs = (fabs((float)deltaTicks) / tps) * 1000.0f * TIME_SAFETY_FACTOR;
  float total = MIN_WAIT_MS + moveMs + EXTRA_SETTLE_MS;
  return (uint32_t)total;
}

// -------------------------------------------------------------------
//                     FLOAT MAPPING HELPERS
// -------------------------------------------------------------------
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
  if (fabs(in_max - in_min) < 1e-9) return out_min;  // prevent div0
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double constrainf(double val, double min_val, double max_val) {
  return (val < min_val) ? min_val : (val > max_val ? max_val : val);
}

// Explicit template instantiation (needed for Arduino build)
template void serial_printf_verbose<>(const char *fmt);
