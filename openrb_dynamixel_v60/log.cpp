#include <Arduino.h>
#include <stdarg.h>
#include "log.h"

// ============================================================
// CONFIG
// ============================================================

#ifndef DEBUG_ENABLE
#define DEBUG_ENABLE 1
#endif

#ifndef LOG_ENABLE
#define LOG_ENABLE 1
#endif

// ============================================================
// INTERNAL STATE
// ============================================================

bool log_line_open = false;
int cmd_no = 0;

void increment_cmd_no() {
  cmd_no++;
}

// ============================================================
// LOG LINE CONTROL
// ============================================================

void log_begin(log_module_t enum_module,
               int i_type,
               const char *description) {
#if LOG_ENABLE || DEBUG_ENABLE
  if (log_line_open) {
    Serial.println();
  }

  if (i_type == debug_info_type) Serial.print("   ---- ");
  if (i_type == debug_error_type) Serial.print("   ---- [!] ERR ");
  if (i_type == log_error_type) Serial.print("[!] ERR ");

  if (enum_module == MOD_RUN) Serial.print("RUN");
  if (enum_module == MOD_MOVE) Serial.print("MOVE");
  if (enum_module == MOD_SERVO) Serial.print("SERVO");
  if (enum_module == MOD_COLORSCAN) Serial.print("COLORSCAN");
  if (enum_module == MOD_COLORCHECK) Serial.print("COLORCHECK");
  if (enum_module == MOD_CUBEORI) Serial.print("CUBEORI");
  if (enum_module == MOD_CUBEMOVE) Serial.print("CUBEMOVE");
  if (enum_module == MOD_ROBOTMOVE) Serial.print("ROBOTMOVE");
  if (enum_module == MOD_CMD) Serial.print("CMD");

  Serial.print("()");
  Serial.print(cmd_no);
  Serial.print(") ");


  if (i_type == log_error_type) Serial.print("error=");
  else if (i_type == log_info_type) Serial.print("info=");
  else if (i_type == debug_error_type) Serial.print("debug_error=");
  else if (i_type == debug_info_type) Serial.print("debug_info=");
  else Serial.print("other=");
  Serial.print(description);

  log_line_open = true;
#else
  (void)e_module;
  (void)i_type;
  (void)description;
#endif
}

void log_end() {
#if DEBUG_ENABLE || LOG_ENABLE
  if (log_line_open) {
    Serial.println();
    log_line_open = false;
  }
#endif
}

// ============================================================
// KEY = VALUE HELPERS
// ============================================================

void log_kv(const char *key, const char *val_c_str) {
#if DEBUG_ENABLE || LOG_ENABLE
  Serial.print("");
  Serial.print(key);
  Serial.print("=");
  Serial.print(val_c_str);
#else
  (void)key;
  (void)val_c_str;
#endif
}

void log_kv(const char *key, int32_t val_i) {
#if DEBUG_ENABLE || LOG_ENABLE
  Serial.print("");
  Serial.print(key);
  Serial.print("=");
  Serial.print(val_i);
#else
  (void)key;
  (void)val_i;
#endif
}

void log_kv(const char *key, int val_i) {
#if DEBUG_ENABLE || LOG_ENABLE
  Serial.print("");
  Serial.print(key);
  Serial.print("=");
  Serial.print(val_i);
#else
  (void)key;
  (void)val_i;
#endif
}

void log_kv(const char *key, double val_d) {
#if DEBUG_ENABLE || LOG_ENABLE
  Serial.print("");
  Serial.print(key);
  Serial.print("=");
  Serial.print(val_d, 2);
#else
  (void)key;
  (void)val_d;
#endif
}

inline void log_kv(const char *key, const String &val_s) {
#if DEBUG_ENABLE || LOG_ENABLE
  log_kv(key, val_s.c_str());
#else
  (void)key;
  (void)val_s;
#endif
}

void log_kv(const char *key, char val_c) {
#if DEBUG_ENABLE || LOG_ENABLE
  char buf[2] = { val_c, '\0' };
  log_kv(key, buf);
#else
  (void)key;
  (void)val_c;
#endif
}
