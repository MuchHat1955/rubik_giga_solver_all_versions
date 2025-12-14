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
  char rb_id_[12]; \
  rb_make_id(rb_id_, sizeof(rb_id_))

// ============================================================
// LOW-LEVEL EMIT (USES serial_printf)
// ============================================================
inline void rb_emit_info(const char *info_name, const char *fmt, ...) {
  char id[12];
  rb_make_id(id, sizeof(id));

  serial_printf("INFO %s info=%s ", id, info_name);

  va_list ap;
  va_start(ap, fmt);
  serial_printf(fmt, ap);
  va_end(ap);

  serial_printf("\n");
}

inline void rb_emit_err(const char *err_name, const char *fmt, ...) {
  char id[12];
  rb_make_id(id, sizeof(id));

  serial_printf("ERR err=%s %s", id, err_name);

  va_list ap;
  va_start(ap, fmt);
  serial_printf(fmt, ap);
  va_end(ap);

  serial_printf("\n");
}

// ============================================================
// GENERIC INFO / ERR (CORE)
// ============================================================
#define RB_INFO(ev, fmt, ...) rb_emit_info(ev, fmt, ##__VA_ARGS__)
#define RB_ERR(ev, fmt, ...) rb_emit_err(ev, fmt, ##__VA_ARGS__)

// ============================================================
// DEBUG (NOT PARSED BY HOST)
// ============================================================
#define RB_DEBUG(fmt, ...) \
  serial_printf_verbose("DEBUG " fmt "\n", ##__VA_ARGS__)

// ============================================================
// RUN LIFECYCLE (COMMAND START / END)
// ============================================================

// START
#define RB_RUN_START(letter, params_cstr) \
  do { \
    set_start_millis(); \
    increment_cmd_id(letter); \
    RB_INFO("RUN", \
            "start=%c%d params=%s", get_cmd_id_letter(), get_cmd_id_num(), params_cstr); \
  } while (0)

// END OK
#define RB_RUN_END_OK() \
  do { \
    float _dur_s = (millis() - get_start_millis()) / 1000.0f; \
    RB_INFO("RUN", \
            "end=%c%d status=ok duration_s=%.3f", \
            get_cmd_id_letter(), get_cmd_id_num(), _dur_s); \
  } while (0)

// END ERROR
#define RB_RUN_END_ERR(err_text) \
  do { \
    float _dur_s = (millis() - get_start_millis()) / 1000.0f; \
    RB_ERR("RUN", \
           "err=%c%d status=error err=%s duration_s=%.3f", \
           get_cmd_id_letter(), get_cmd_id_num(), err_text, _dur_s); \
  } while (0)

// ============================================================
// MODULE-SPECIFIC MACROS (UNCHANGED CALL STYLE)
// ============================================================

// COLOR SCAN
#define RB_INFO_COLORSCAN(ev, id, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("COLORSCAN", ev, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_COLORSCAN(ev, id, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("COLORSCAN", ev, fmt, ##__VA_ARGS__); \
  } while (0)

// CUBE ORIENTATION
#define RB_INFO_CUBEORI(ev, id, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("CUBEORI", ev, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_CUBEORI(ev, id, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("CUBEORI", ev, fmt, ##__VA_ARGS__); \
  } while (0)

// ROBOT
#define RB_INFO_ROBOTMOVE(ev, id, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("ROBOTMOVE", ev, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_ROBOTMOVE(ev, id, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("ROBOTMOVE", ev, fmt, ##__VA_ARGS__); \
  } while (0)

// CUBE
#define RB_INFO_CUBEMOVE(ev, id, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("CUBEMOVE", ev, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_CUBEMOVE(ev, id, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("CUBEMOVE", ev, fmt, ##__VA_ARGS__); \
  } while (0)

// COLORCHECK
#define RB_INFO_COLORCHECK(ev, id, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("COLORCHECK", ev, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_COLORCHECK(ev, id, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("COLORCHECK", ev, fmt, ##__VA_ARGS__); \
  } while (0)

// SERVO
#define RB_INFO_SERVO(ev, id, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_INFO("SERVO", ev, fmt, ##__VA_ARGS__); \
  } while (0)

#define RB_ERR_SERVO(ev, id, fmt, ...) \
  do { \
    RB_DECLARE_ID(); \
    RB_ERR("SERVO", ev, fmt, ##__VA_ARGS__); \
  } while (0)

// ============================================================
// PROTOCOL QUERY
// ============================================================
inline void rb_report_protocol() {
  serial_printf("PROTOCOL info=version version=%d\n", RB_PROTOCOL_VERSION);
}
