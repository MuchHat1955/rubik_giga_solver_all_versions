#pragma once
#include <Arduino.h>
#include "servos.h"
#include "servo_move.h"
#include "vertical_kinematics.h"
#include "utils.h"
#include "log.h"

// -------------------------------------------------------------------
//                      COMMAND PARSER - DECLARATION
// -------------------------------------------------------------------

// Lightweight Arduino scanf alternative
int arduino_sscanf(const String &line, const char *fmt, void *out);

// Command dispatcher
void process_serial_command(String &line);

// Helpers for showing servo info
void cmdInfo(uint8_t id);
void cmdSetLimit(uint8_t id, int minL, int maxL);
void print_pos(uint8_t id);
String get_help_text();

bool resetBase(int baseTurnToAccomodate);
bool liftCube();
bool lowerCube();

// ============================================================
// All command handler function declarations
// ============================================================

// Verbose control
bool cmd_verbose_on(int argc, double *argv);
bool cmd_verbose_off(int argc, double *argv);

// Cartesian XY move
bool cmd_move_xy(int argc, double *argv);

// Servo movement
bool cmd_move_deg(int argc, double *argv);
bool cmd_move_ticks(int argc, double *argv);
bool cmd_move_per(int argc, double *argv);

// Servo min/max
bool cmd_set_servo_min(int argc, double *argv);
bool cmd_set_servo_max(int argc, double *argv);

// Center (currently disabled)
bool cmd_move_center(int argc, double *argv);

// X/Y motion
bool cmd_move_y(int argc, double *argv);
bool cmd_move_x(int argc, double *argv);

// Gripper control
bool cmd_move_clamp(int argc, double *argv);
bool cmd_move_gripper(int argc, double *argv);

// Wrist
bool cmd_move_wrist_vert(int argc, double *argv);

// Color sensor
bool cmd_color(int argc, double *argv);

// Run sequences
bool cmd_run(int argc, double *argv);

// Servo info
bool cmd_read_servo(int argc, double *argv);
bool cmd_reboot_servos(int argc, double *argv);
bool cmd_set_servo_flag_servos_stop_all(int argc, double *argv);
bool cmd_clear_flag_servos_stop_all(int argc, double *argv);
bool cmd_servo_info(int argc, double *argv);

// Led
bool cmd_ledon(int argc, double *argv);
bool cmd_ledoff(int argc, double *argv);

// Help
bool cmd_help(int argc, double *argv);

bool cmd_getori_data(int argc, double *argv);
bool cmd_clear_ori_data(int argc, double *argv);
bool cmd_restore_ori(int argc, double *argv);
bool cmd_read_cube_colors(int argc, double *argv);
bool cmd_read_one_color(int argc, double *argv);
char read_one_color_cb(int slot);
bool cmd_read_one_face_colors(int argc, double *argv);
bool cmd_getcolor_data(int argc, double *argv);
void print_colors_analyzer_detail();
bool cmd_get_version(int argc, double *argv);

void process_serial_command(const char* line);

// ============================================================
// class serial_line_history
// ============================================================

#define LINE_MAX_LEN   128
#define LINE_HISTORY   10

class serial_line_history {
public:
  explicit serial_line_history();

  // Call frequently from loop()
  void poll();

  // ===== line history =====
  int count() const;                 // number of complete buffered lines
  const char* peek(int idx) const;   // 0 = newest, 1 = previous, ...
  const char* read(int idx);         // peek + remove
  void clear(int idx);               // idx >= 0 removes one, -1 clears all

  // ===== partial line =====
  bool has_partial() const;          // bytes received but no newline yet
  int  partial_len() const;          // length of partial line
  const char* peek_partial() const;  // inspect partial line (read-only)
  void clear_partial();              // discard partial line

private:
  // ring buffer for complete lines
  char lines_[LINE_HISTORY][LINE_MAX_LEN];
  int head_;     // index of newest line
  int count_;    // number of valid lines

  // current assembling line
  char cur_[LINE_MAX_LEN];
  int cur_len_;

  void commit_line();
  int resolve_index(int idx) const;
};

extern serial_line_history serial_line;

