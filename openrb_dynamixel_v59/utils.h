#pragma once
#include <Arduino.h>
#include <math.h>
#include <stdarg.h>

// -------------------------------------------------------------------
//                         UTILS - DECLARATIONS
// -------------------------------------------------------------------

// ---------------- Numeric helpers ----------------
double clamp(double v, double lo, double hi);
double rad2deg(double rad);
double deg2rad(double deg);

extern const float PROTOCOL;
extern bool verboseOn;

// ---------------- Serial helper ----------------
template<typename... Args>
void serial_printf_verbose(const char *fmt, Args... args) {
  if (!verboseOn) return;
  char buf[200];
  snprintf(buf, sizeof(buf), fmt, args...);
  Serial.print(buf);
}
template<typename... Args>
void serial_printf(const char *fmt, Args... args) {
  char buf[200];
  snprintf(buf, sizeof(buf), fmt, args...);
  Serial.print(buf);
}

// ---------------- Timing / motion helpers ----------------
float pvToTicksPerSec(int pvLSB);
uint32_t estimateTravelTimeMs(uint8_t id, int deltaTicks);

// ---------------- Mapping helpers ----------------
double mapf(double val, double in_min, double in_max, double out_min, double out_max);
double constrainf(double val, double min_val, double max_val);

// ============================================================
// MODULE-SPECIFIC LOGGING WRAPPERS
// ============================================================

#pragma once
#include <Arduino.h>
#include <stdarg.h>

/*
======================================================================
RB → GIGA SERIAL RESPONSE FORMAT (PROTOCOL v1)
======================================================================

GENERAL FORMAT
--------------
MODULE info=<token> (<id>) key=value key=value
ERR MODULE err=<token> (<id>) key=value key=value

(<id>) FORMAT
-------------
(letter)(number)

- number : monotonic command counter since reboot (global)
- letter : command category (origin from GIGA)

LETTER MEANINGS
---------------
c = cube move (logical cube move)
r = robot move (explicit hardware move)
s = color scan
a = analysis (color check / orientation report)
v = verification (servo / hardware check)

EXAMPLES
--------
RUN info=end (c36) status=ok duration_s=1.842
RUN info=end (s12) status=error source=COLORSCAN err=no_callback duration_s=0.004
ERR ROBOTMOVE err=servo_fault (c36) id=3

VERSION QUERY COMMAND (FROM GIGA)
--------------------------------
Command:
  proto?

Response:
  PROTOCOL info=version version=1

Optional extended response:
  PROTOCOL info=capabilities
    id_format=(letter+number)
    duration=seconds
    start_end=RUN
    modules=COLORSCAN,COLORCHECK,CUBEORI,CUBEMOVE,ROBOTMOVE,MOTIONPLAN,SERVO

======================================================================
*/

// ============================================================
// COMMAND DEFINITIONS (CMDROUTER)
// ============================================================

enum cmd_type_t {
  CMD_CUBE_MOVE,
  CMD_ROBOT_MOVE,
  CMD_COLOR_SCAN,
  CMD_ANALYSIS,
  CMD_VERIFY,
  CMD_PROTO
};

struct cmd_def_t {
  const char *name;
  cmd_type_t type;
  char id_letter;
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ============================================================
// PROTOCOL VERSION
// ============================================================
#define RB_PROTOCOL_VERSION 3

// ============================================================
// EXECUTION CONTEXT (GLOBAL, SINGLE INSTANCE)
// ============================================================
void increment_cmd_id(char letter);
char get_cmd_id_letter();
char get_cmd_id_num();
unsigned long get_start_millis();
void set_start_millis();

// ============================================================
// ID HELPERS
// ============================================================
void rb_make_id(char *out, size_t len);

// ALWAYS declare rb_id_ locally when needed
#define RB_DECLARE_ID() \
  char rb_cmd_id_[12]; \
  rb_make_id(rb_cmd_id_, sizeof(rb_cmd_id_))

// ============================================================
// LOW-LEVEL EMIT (USES serial_printf)
// ============================================================
void serial_vprintf(const char *fmt, va_list ap); 
void rb_emit_info(const char *module_name, const char *info_name, const char *fmt, ...);
void rb_emit_err(const char *module_name, const char *error_description, const char *fmt, ...);

// ============================================================
// GENERIC INFO / ERR (CORE)
// ============================================================
#define RB_INFO(module_name, info_description, fmt, ...) rb_emit_info(module_name, info_description, fmt, ##__VA_ARGS__)
#define RB_ERR(module_name, error_description, fmt, ...) rb_emit_err(module_name, error_description, fmt, ##__VA_ARGS__)

// ============================================================
// RUN LIFECYCLE (COMMAND START / END)
// ============================================================

// START
#define RB_CMD_START(letter, cmd, params_cstr) \
  do { \
    set_start_millis(); \
    increment_cmd_id(letter); \
    RB_INFO("CMD", "start", "cmd=%s params=%s", cmd, params_cstr); \
  } while (0)

// END OK
#define RB_CMD_END_OK() \
  do { \
    float _dur_s = (millis() - get_start_millis()) / 1000.0f; \
    RB_INFO("CMD", "end", "status=%s duration_s=%.1f", \
            "ok", _dur_s); \
  } while (0)

// END ERROR
#define RB_CMD_END_ERR(err_text) \
  do { \
    float _dur_s = (millis() - get_start_millis()) / 1000.0f; \
    RB_INFO("CMD", "end", "status=error err=%s duration_s=%.1f", \
            err_text, _dur_s); \
  } while (0)

// ============================================================
// MODULE-SPECIFIC MACROS (UNCHANGED CALL STYLE)
// ============================================================

// COLOR SCAN
#define RB_INFO_COLORSCAN(info_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("COLORSCAN", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_COLORSCAN(error_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("COLORSCAN", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// CUBE ORIENTATION
#define RB_INFO_CUBEORI(info_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("CUBEORI", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_CUBEORI(error_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("CUBEORI", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// ROBOT
#define RB_INFO_ROBOTMOVE(info_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("ROBOTMOVE", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_ROBOTMOVE(error_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("ROBOTMOVE", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// CUBE
#define RB_INFO_CUBEMOVE(info_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("CUBEMOVE", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_CUBEMOVE(error_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("CUBEMOVE", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// COLORCHECK
#define RB_INFO_COLORCHECK(info_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("COLORCHECK", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_COLORCHECK(error_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("COLORCHECK", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// SERVO
#define RB_INFO_SERVO(info_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("SERVO", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_SERVO(error_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("SERVO", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// MOVE
#define RB_INFO_MOVE(info_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("MOVE", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_MOVE(error_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("MOVE", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// RUN
#define RB_INFO_RUN(info_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("RUN", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_RUN(error_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("RUN", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// CMD
#define RB_INFO_CMD(info_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("CMD", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_CMD(error_description, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("CMD", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// ============================================================
// PROTOCOL QUERY
// ============================================================
inline void rb_report_protocol() {
  serial_printf("PROTOCOL info=version version=%d\n", RB_PROTOCOL_VERSION);
}
