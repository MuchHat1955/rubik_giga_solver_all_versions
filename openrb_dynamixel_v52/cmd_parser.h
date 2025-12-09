#pragma once
#include <Arduino.h>
#include "servos.h"
#include "movement.h"
#include "vertical_kinematics.h"
#include "utils.h"

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
bool cmd_set_min(int argc, double *argv);
bool cmd_set_max(int argc, double *argv);

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
bool cmd_read(int argc, double *argv);
bool cmd_info(int argc, double *argv);

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
