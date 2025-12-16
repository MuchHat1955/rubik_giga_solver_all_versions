#pragma once
#include <Arduino.h>
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
void LOG_VERBOSE(const char *fmt, ...);
void LOG_RB(const char *fmt, ...);

// ---------------- Timing / motion helpers ----------------
float pvToTicksPerSec(int pvLSB);
uint32_t estimateTravelTimeMs(uint8_t id, int deltaTicks);

// ---------------- Mapping helpers ----------------
double mapf(double val, double in_min, double in_max, double out_min, double out_max);
double constrainf(double val, double min_val, double max_val);

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

#define LOG_VERBOSE(...) //
#define LOG_RB(...) //

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
int get_cmd_id_num();
unsigned long get_start_millis();
void set_start_millis();

// ============================================================
// ID HELPERS
// ============================================================
void rb_make_id(char *out, size_t len);

// ============================================================
// LOW-LEVEL EMIT (USES LOG_RB)
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
    RB_INFO("CM", "start", "cmd=%s params=%s", cmd, params_cstr); \
  } while (0)

// END OK
#define RB_CMD_END_OK() \
  do { \
    float _dur_s = (millis() - get_start_millis()) / 1000.0f; \
    RB_INFO("CM", "end", "status=%s duration_s=%.1f", \
            "ok", _dur_s); \
  } while (0)

// END ERROR
#define RB_CMD_END_ERR(err_text) \
  do { \
    float _dur_s = (millis() - get_start_millis()) / 1000.0f; \
    RB_INFO("CM", "end", "status=error err=%s duration_s=%.1f", \
            err_text, _dur_s); \
  } while (0)

// ============================================================
// MODULE-SPECIFIC MACROS (UNCHANGED CALL STYLE)
// ============================================================

// COLOR SCAN
#define RB_INFO_COLORSCAN(info_description, fmt, ...) \
  do { \
    RB_INFO("S", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_COLORSCAN(error_description, fmt, ...) \
  do { \
    RB_ERR("S", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// CUBE ORIENTATION
#define RB_INFO_CUBEORI(info_description, fmt, ...) \
  do { \
    RB_INFO("O", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_CUBEORI(error_description, fmt, ...) \
  do { \
    RB_ERR("O", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// ROBOT
#define RB_INFO_ROBOTMOVE(info_description, fmt, ...) \
  do { \
    RB_INFO("R", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_ROBOTMOVE(error_description, fmt, ...) \
  do { \
    RB_ERR("R", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// CUBE
#define RB_INFO_CUBEMOVE(info_description, fmt, ...) \
  do { \
    RB_INFO("C", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_CUBEMOVE(error_description, fmt, ...) \
  do { \
    RB_ERR("C", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// COLORCHECK
#define RB_INFO_COLORCHECK(info_description, fmt, ...) \
  do { \
    RB_INFO("C", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_COLORCHECK(error_description, fmt, ...) \
  do { \
    RB_ERR("C", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// SERVO
#define RB_INFO_SERVO(info_description, fmt, ...) \
  do { \
    RB_INFO("S", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_SERVO(error_description, fmt, ...) \
  do { \
    RB_ERR("S", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// MOVE
#define RB_INFO_MOVE(info_description, fmt, ...) \
  do { \
    RB_INFO("M", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_MOVE(error_description, fmt, ...) \
  do { \
    RB_ERR("M", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// RUN
#define RB_INFO_RUN(info_description, fmt, ...) \
  do { \
    RB_INFO("R", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_RUN(error_description, fmt, ...) \
  do { \
    RB_ERR("R", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// CMD
#define RB_INFO_CMD(info_description, fmt, ...) \
  do { \
    RB_INFO("D", info_description, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_CMD(error_description, fmt, ...) \
  do { \
    RB_ERR("D", error_description, fmt, ##__VA_ARGS__); \
  } while (0)

// ============================================================
// PROTOCOL QUERY
// ============================================================
void rb_report_protocol();
