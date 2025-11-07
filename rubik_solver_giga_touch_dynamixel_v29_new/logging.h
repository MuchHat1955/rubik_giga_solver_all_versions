#pragma once

/*

EXAMPLE USAGE

#include "logging.h"

void setup() {
  Serial.begin(115200);

  LOG_SECTION_START("setup");
  LOG_PRINTF("version 1.2\n");
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

// ----- CONFIG -----
#define MAX_NESTED_SECTIONS 24
#define LOG_BUFFER_SIZE 1024  // ring buffer for deferred logging from ISRs
// ------------------

extern bool logging_on;

void log_indent();
void log_section_start(const String& section_name);
void log_section_start_var(const String& title, const String& var_name, const String& var_val);
void log_section_end();
void log_indent_reset();

// Safe, deferred log flush (call periodically in loop)
void log_flush_buffer();


template<typename... Args>
inline void serial_printf(const char* fmt, Args... args) {
  char buf[200];
  snprintf(buf, sizeof(buf), fmt, args...);
  Serial.print(buf);
}

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
// Formatted logging macros using serial_printf
// -----------------------------------------------------------

#define LOG_PRINTF(fmt, ...) \
  do { \
    if (logging_on) { \
      log_indent(); \
      serial_printf(fmt, ##__VA_ARGS__); \
      Serial.println(); \
    } \
  } while (0)

#define LOG_SECTION_START_PRINTF(title, fmt, ...) \
  do { \
    if (logging_on) { \
      log_indent(); \
      serial_printf("---- {{%s}} start ", title); \
      serial_printf(fmt, ##__VA_ARGS__); \
      Serial.println(" ----"); \
    } \
  } while (0)


// -----------------------------------------------------------
// Portable variable logging macros (robust for String, char*, bool, numbers)
// -----------------------------------------------------------

// ---------- generic helpers ----------

// Overload for Arduino String
inline void log_var_value(const String& val) {
  Serial.print(val);
}

// Overload for const char*
inline void log_var_value(const char* val) {
  Serial.print(val ? val : "(null)");
}

// Overload for bool
inline void log_var_value(bool val) {
  Serial.print(val ? "true" : "false");
}

// Template for all other types (int, float, etc.)
template<typename T>
inline void log_var_value(const T& val) {
  Serial.print(val);
}

// ---------- core logging function ----------
inline void log_var(const char* name, const auto& val, bool newline = true, bool cont = false) {
  if (!logging_on) return;
  if (!cont) log_indent();

  if (cont) Serial.print(" | ");
  Serial.print(name);
  Serial.print(" {");
  log_var_value(val);
  Serial.print("}");
  if (newline) Serial.println();
}

// ---------- user macros ----------
#define LOG_VAR(name, val) log_var(name, val, true, false)
#define LOG_VAR_(name, val) log_var(name, val, false, false)
#define LOG_VAR_CONT(name, val) log_var(name, val, true, true)
#define LOG_VAR_CONT_(name, val) log_var(name, val, false, true)

// ISR-safe print (queues string)
void log_enqueue(const char* msg);
