#pragma once
#include <Arduino.h>

/*

EXAMPLE USAGE

#include "logging.h"

void setup() {
  Serial.begin(115200);

  LOG_SECTION_START("setup");
  LOG_VAR("version", "1.2");
  delay(10);

  LOG_SECTION_START_VAR("init motor", "ID", "12");
  delay(42);
  LOG_SECTION_END(); // end init motor

  delay(15);
  LOG_SECTION_END(); // end setup
}

void loop() {
  LOG_FLUSH();   // flush buffered logs (important if using ISRs)
  delay(1000);
}

EXAMPLE OUTPUT

---- {setup} start <12ms> ----
   version {1.2}
   ---- {init motor | ID {12}} start <23ms> ----
   ---- {init motor | ID {12}} end <65ms, Δ42ms> ----
---- {setup} end <80ms, Δ68ms> ----

*/

// ===== CONFIG =====
#define MAX_NESTED_SECTIONS 24
#define LOG_BUFFER_SIZE 1024  // ring buffer for deferred logging from ISRs
// ==================

extern bool logging_on;

void log_indent();
void log_section_start(const String& section_name);
void log_section_start_var(const String& title, const String& var_name, const String& var_val);
void log_section_end();
void log_indent_reset();

// Safe, deferred log flush (call periodically in loop)
void log_flush_buffer();

// Convenience macros
#define LOG_SECTION_START(title) \
  do { \
    if (logging_on) log_section_start(title); \
  } while (0)

#define LOG_SECTION_START_VAR(title, name, val) \
  do { \
    if (logging_on) log_section_start_var(title, name, val); \
  } while (0)

#define LOG_SECTION_END() \
  do { \
    if (logging_on) log_section_end(); \
  } while (0)

#define LOG_RESET() \
  do { log_indent_reset(); } while (0)

#define LOG_FLUSH() \
  do { log_flush_buffer(); } while (0)

// -----------------------------------------------------------
// Portable variable logging macros (no Serial.printf required)
// -----------------------------------------------------------
#define LOG_VAR(name, val) \
  do { \
    if (logging_on) { \
      log_indent(); \
      Serial.print(name); \
      Serial.print(" {"); \
      Serial.print(val); \
      Serial.println("}"); \
    } \
  } while (0)

#define LOG_VAR_(name, val) \
  do { \
    if (logging_on) { \
      log_indent(); \
      Serial.print(name); \
      Serial.print(" {"); \
      Serial.print(val); \
      Serial.print("}"); \
    } \
  } while (0)

#define LOG_VAR_CONT(name, val) \
  do { \
    if (logging_on) { \
      Serial.print(" | "); \
      Serial.print(name); \
      Serial.print(" {"); \
      Serial.print(val); \
      Serial.println("}"); \
    } \
  } while (0)

#define LOG_VAR_CONT_(name, val) \
  do { \
    if (logging_on) { \
      Serial.print(" | "); \
      Serial.print(name); \
      Serial.print(" {"); \
      Serial.print(val); \
      Serial.print("}"); \
    } \
  } while (0)

#define LOG_VAR2(n1, v1, n2, v2) \
  do { \
    if (logging_on) { \
      log_indent(); \
      Serial.print(n1); \
      Serial.print(" {"); \
      Serial.print(v1); \
      Serial.print("} | "); \
      Serial.print(n2); \
      Serial.print(" {"); \
      Serial.print(v2); \
      Serial.println("}"); \
    } \
  } while (0)

#define LOG_VAR2_(n1, v1, n2, v2) \
  do { \
    if (logging_on) { \
      log_indent(); \
      Serial.print(n1); \
      Serial.print(" {"); \
      Serial.print(v1); \
      Serial.print("} | "); \
      Serial.print(n2); \
      Serial.print(" {"); \
      Serial.print(v2); \
      Serial.print("}"); \
    } \
  } while (0)

#define LOG_VAR2_CONT(n1, v1, n2, v2) \
  do { \
    if (logging_on) { \
      Serial.print(" | "); \
      Serial.print(n1); \
      Serial.print(" {"); \
      Serial.print(v1); \
      Serial.print("} | "); \
      Serial.print(n2); \
      Serial.print(" {"); \
      Serial.print(v2); \
      Serial.println("}"); \
    } \
  } while (0)

#define LOG_VAR2_CONT_(n1, v1, n2, v2) \
  do { \
    if (logging_on) { \
      Serial.print(" | "); \
      Serial.print(n1); \
      Serial.print(" {"); \
      Serial.print(v1); \
      Serial.print("} | "); \
      Serial.print(n2); \
      Serial.print(" {"); \
      Serial.print(v2); \
      Serial.print("}"); \
    } \
  } while (0)

// ISR-safe print (queues string)
void log_enqueue(const char* msg);
