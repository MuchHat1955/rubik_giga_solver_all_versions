#pragma once
#include <Arduino.h>
#include <stdint.h>

#define LOG_ENABLE 1
#define DEBUG_ENABLE 0

// ===================================================================
// ENABLE / DISABLE LOGGING SYSTEMS
// ===================================================================
#ifndef LOG_ENABLE
#define LOG_ENABLE 1  // protocol / GIGA logs
#endif

#ifndef DEBUG_ENABLE
#define DEBUG_ENABLE 0  // internal verbose logs
#endif

// ===================================================================
// MODULE IDS (shared)
// ===================================================================
enum log_module_t : uint8_t {
  MOD_CMD = 0,
  MOD_RUN,
  MOD_SERVO_MOVE,
  MOD_SERVOS,
  MOD_COLORSCAN,
  MOD_COLORCHECK,
  MOD_COLORSENSOR,
  MOD_CUBEORI,
  MOD_CUBEMOVE,
  MOD_ROBOTMOVE,
  MOD_KIN
};


// ===================================================================
// EXECUTION CONTEXT (implemented elsewhere)
// ===================================================================
void increment_cmd_no();

// ===================================================================
// LOW-LEVEL EMIT (implemented in log.cpp)
// ===================================================================
void log_begin_str(log_module_t enum_module, int i_type, const char *key, const String v);
void log_begin(log_module_t e_module, int i_type, const char *key, int32_t v);
void log_begin(log_module_t e_module, int i_type, const char *key, int v);
void log_begin(log_module_t e_module, int i_type, const char *key, double v);
void log_begin(log_module_t e_module, int i_type, const char *key, const char *v);
void log_begin(log_module_t e_module, int i_type, const char *key, char v);
void log_begin(log_module_t e_module, int i_type, const char *key, const String v);

void log_var_(const char *key, int32_t v);
void log_var_(const char *key, int v);
void log_var_(const char *key, double v);
void log_var_(const char *key, const char *s);
void log_var_(const char *key, char c);
void log_var_(const char *key, const String &s);
void log_end_();
void log_ln_();

// ===================================================================
// ======================= LOG_ (PROTOCOL) =============================
// ===================================================================

#define log_error_type 0
#define log_info_type 1
#define debug_error_type 2
#define debug_info_type 3

#if LOG_ENABLE

#define LOG_INFO(mod, key, val) \
  do { log_begin(mod, log_info_type, key, val); } while (0)

#define LOG_ERR(mod, key, val) \
  do { log_begin(mod, log_error_type, key, val); } while (0)

#define LOG_VAR(key, val) \
  do { log_var_(key, val); } while (0)

#define LOG_END() \
  do { log_end_(); } while (0)

#define LOG_LN() \
  do { log_ln_(); } while (0)

#else

#define LOG_INFO(mod, tag, val) \
  do { \
  } while (0)
#define LOG_ERR(mod, tag, val) \
  do { \
  } while (0)
#define LOG_VAR(key, val) \
  do { \
  } while (0)
#define LOG_END() \
  do { \
  } while (0)

#endif


// ===================================================================
// ======================= DEBUG_ (VERBOSE) ==============================
// ===================================================================
#if DEBUG_ENABLE

#define DEBUG_INFO(mod, tag) \
  do { log_begin(mod, debug_info_type, key, val); } while (0)

#define DEBUG_ERR(mod, tag) \
  do { log_begin(mod, debug_error_type, key, val); } while (0)

#define DEBUG_VAR(key, val) \
  do { log_var_(key, val); } while (0)

#define DEBUG_END() \
  do { log_end_(); } while (0)

#else

#define DEBUG_INFO(mod, tag) \
  do { \
  } while (0)
#define DEBUG_ERR(mod, tag) \
  do { \
  } while (0)
#define DEBUG_VAR(key, val) \
  do { \
  } while (0)
#define DEBUG_END() \
  do { \
  } while (0)

#endif

#define RUN_CMD(expr, desc) \
  do { \
    if (!(expr)) { \
      LOG_ERR(MOD_RUN, "cmd_failed", desc); \
      LOG_VAR("expr", #expr); \
      LOG_VAR("func", __FUNCTION__); \
      return false; \
    } \
  } while (0)

String servo_id2name(uint8_t id);

#define RUN_PING(id) \
  do { \
    if (!dxl_ping_cached(id)) { \
      LOG_ERR(MOD_RUN, "ping_failed", servo_id2name(id)); \
      LOG_VAR("func", __FUNCTION__); \
      return false; \
    } \
  } while (0)
