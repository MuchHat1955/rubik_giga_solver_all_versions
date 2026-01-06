#pragma once
#include <Arduino.h>
#include <math.h>

// TODO change below for debugging to Serial
#define __serial Serial1

// -------------------------------------------------------------------
//                         UTILS - DECLARATIONS
// -------------------------------------------------------------------

// ---------------- Numeric helpers ----------------
double clamp(double v, double lo, double hi);
double rad2deg(double rad);
double deg2rad(double deg);

extern const float PROTOCOL;

// ---------------- Timing / motion helpers ----------------
float pvToTicksPerSec(int pvLSB);
uint32_t estimateTravelTimeMs(uint8_t id, int deltaTicks);

// ---------------- Mapping helpers ----------------
double mapf(double val, double in_min, double in_max, double out_min, double out_max);
double constrainf(double val, double min_val, double max_val);
