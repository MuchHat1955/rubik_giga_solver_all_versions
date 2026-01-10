#pragma once
#include <Arduino.h>
#include <math.h>

// -------------------------------------------------------------------
//                         UTILS - DECLARATIONS
// -------------------------------------------------------------------


#define Serial_giga Serial3
#define Serial_giga_begin(b) \
  do { Serial3.begin(b); } while (0)
#define Serial_giga_available() (Serial3.available())
#define Serial_giga_read() (Serial3.read())

#define Serial_giga_print(v) \
  do { Serial3.print(v); } while (0)
#define Serial_giga_println(v) \
  do { Serial3.println(v); } while (0)
#define Serial_giga_println_empty() \
  do { Serial3.println(); } while (0)

/*
  #define Serial_giga Serial
#define Serial_giga_begin(b) \
  do { Serial.begin(b); } while (0)
#define Serial_giga_available() (Serial.available())
#define Serial_giga_read() (Serial.read())

#define Serial_giga_print(v) \
  do { Serial.print(v); } while (0)
#define Serial_giga_println(v) \
  do { Serial.println(v); } while (0)
#define Serial_giga_println_empty() \
  do { Serial.println(); } while (0)

  */

/*
#define Serial_giga Serial2
#define Serial_giga_begin(b)        do { Serial2.begin(b); } while (0)
#define Serial_giga_available()     (Serial2.available())
#define Serial_giga_read()          (Serial2.read())

#define Serial_giga_print(v)        do { Serial2.print(v); } while (0)
#define Serial_giga_println(v)      do { Serial2.println(v); } while (0)
#define Serial_giga_println_empty() do { Serial2.println(); } while (0)
*/

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
