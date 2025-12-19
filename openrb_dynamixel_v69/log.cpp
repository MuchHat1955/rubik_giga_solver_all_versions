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

int log_line_open = 0;
int cmd_no = 0;

extern bool serial_ok;

void increment_cmd_no() {
  cmd_no++;
}

// ============================================================
// LOG LINE CONTROL
// ============================================================

void serial_print_with_underscores(const char *description) {
  while (*description) {
    char c = *description++;
    if (c == ' ') c = '_';
    if (c == ',') c = '_';
    if (c == ':') c = '_';
    if (c == ',') c = '_';
    Serial.print(c);
  }
}

void log_begin(log_module_t e_module, int i_type, const char *key, int32_t v) {
  log_begin_str(e_module, i_type, key, String(v));
}
void log_begin(log_module_t e_module, int i_type, const char *key, int v) {
  log_begin_str(e_module, i_type, key, String(v));
}
void log_begin(log_module_t e_module, int i_type, const char *key, double v) {
  log_begin_str(e_module, i_type, key, String(v, 2));
}
void log_begin(log_module_t e_module, int i_type, const char *key, const char *v) {
  log_begin_str(e_module, i_type, key, String(v));
}
void log_begin(log_module_t e_module, int i_type, const char *key, char v) {
  log_begin_str(e_module, i_type, key, String(v));
}
void log_begin(log_module_t e_module, int i_type, const char *key, const String v) {
  log_begin_str(e_module, i_type, key, v);
}

void log_begin_str(log_module_t enum_module, int i_type, const char *key, const String v) {
#if LOG_ENABLE || DEBUG_ENABLE
  if (!serial_ok) return;
  if (log_line_open > 0) {
    Serial.println();
    log_line_open--;
    if (log_line_open > 0)
      Serial.println();
    log_line_open = 0;
  }

  if (enum_module != MOD_CMD && i_type == log_info_type) Serial.print("        ");
  if (enum_module != MOD_CMD && i_type == debug_info_type) Serial.print("        ");
  if (i_type == debug_error_type) Serial.print("[!] ERR ");
  if (i_type == log_error_type) Serial.print("[!] ERR ");


  if (enum_module == MOD_RUN) Serial.print("RUN");
  if (enum_module == MOD_SERVO_MOVE) Serial.print("SERVOMOVE");
  if (enum_module == MOD_SERVOS) Serial.print("SERVOS");
  if (enum_module == MOD_COLORSCAN) Serial.print("COLORSCAN");
  if (enum_module == MOD_COLORCHECK) Serial.print("COLORCHECK");
  if (enum_module == MOD_CUBEORI) Serial.print("CUBEORI");
  if (enum_module == MOD_CUBEMOVE) Serial.print("CUBEMOVE");
  if (enum_module == MOD_ROBOTMOVE) Serial.print("ROBOTMOVE");
  if (enum_module == MOD_CMD) Serial.print("CMD");

  Serial.print(" (");
  Serial.print(cmd_no);
  Serial.print(") ");


  Serial.print(key);
  Serial.print("=");
  serial_print_with_underscores(v.c_str());

  log_line_open = 1;
#else
  (void)e_module;
  (void)i_type;
  (void)description;
#endif
}

void log_end_() {
#if DEBUG_ENABLE || LOG_ENABLE
  if (!serial_ok) return;
  if (log_line_open > 0) {
    Serial.println();
    log_line_open = 0;
  }
#endif
}

void log_ln_() {
#if DEBUG_ENABLE || LOG_ENABLE
  if (!serial_ok) return;
  if (log_line_open > 0) {
    Serial.println();
  }
  Serial.println();
  log_line_open = 0;
#endif
}

// ============================================================
// KEY = VALUE HELPERS
// ============================================================

void log_var_(const char *key, const char *val_c_str) {
#if DEBUG_ENABLE || LOG_ENABLE
  if (!serial_ok) return;
  Serial.print(" ");
  Serial.print(key);
  Serial.print("=");
  Serial.print(val_c_str);
#else
  (void)key;
  (void)val_c_str;
#endif
}

void log_var_(const char *key, int32_t val_i) {
#if DEBUG_ENABLE || LOG_ENABLE
  if (!serial_ok) return;
  Serial.print(" ");
  Serial.print(key);
  Serial.print("=");
  Serial.print(val_i);
#else
  (void)key;
  (void)val_i;
#endif
}

void log_var_(const char *key, int val_i) {
#if DEBUG_ENABLE || LOG_ENABLE
  if (!serial_ok) return;
  Serial.print(" ");
  Serial.print(key);
  Serial.print("=");
  Serial.print(val_i);
#else
  (void)key;
  (void)val_i;
#endif
}

void log_var_(const char *key, double val_d) {
#if DEBUG_ENABLE || LOG_ENABLE
  if (!serial_ok) return;
  Serial.print(" ");
  Serial.print(key);
  Serial.print("=");
  Serial.print(val_d, 2);
#else
  (void)key;
  (void)val_d;
#endif
}

void log_var_(const char *key, const String &val_s) {
#if DEBUG_ENABLE || LOG_ENABLE
  log_var_(key, val_s.c_str());
#else
  (void)key;
  (void)val_s;
#endif
}

void log_var_(const char *key, char val_c) {
#if DEBUG_ENABLE || LOG_ENABLE
  char buf[2] = { val_c, '\0' };
  log_var_(key, buf);
#else
  (void)key;
  (void)val_c;
#endif
}
