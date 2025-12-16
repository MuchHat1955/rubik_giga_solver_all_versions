#pragma once

#include <Arduino.h>

// -------------------------------------------------------------------
//                         EXTERNAL VARIABLES
// -------------------------------------------------------------------

extern double max_xmm;
extern double max_ymm;
extern double min_ymm;
extern double speed;

extern CubeOri ori;
extern CubeColorReader color_reader;
extern ColorAnalyzer color_analyzer;

extern char crrColorChar;

// -------------------------------------------------------------------
//                         POSITION CONSTANTS
// -------------------------------------------------------------------

// ---- y poses
#define Y_CENTER 76
#define Y_ALIGN 68
#define Y_MID 97
#define Y_UP 112
#define Y_DOWN 33
#define Y_ROTATE_BASE 93
#define Y_ABOVE_DROP 93
#define Y_DROP 83

// ---- y read color poses
#define Y_C_TOP 65
#define Y_C_MID 48

// ---- x poses
#define X_CENTER 1.0

// ---- x read color poses
#define X_C_LEFT -14.0
#define X_C_RIGHT 20.0
#define X_C_CENTER 2.0

// ---- gripper poses
#define G_OPEN 28
#define G_WIDE_OPEN -1
#define G_CLOSE 100
#define G_SOFT_CLOSE 60
#define G_ALIGN_LEFT 100
#define G_ALIGN_RIGHT 100

// ---- base poses
#define B_CENTER 0
#define B_RIGHT 90
#define B_LEFT -90
#define B_BACK -180
#define B_ERR 3
#define B_TOL 3

// -------------------------------------------------------------------
//                        RUN COMMAND CONSTANTS
// -------------------------------------------------------------------

static constexpr int RUN_ZERO = 0;

static constexpr int RUN_RIGHT_DOWN = 11;
static constexpr int RUN_LEFT_DOWN = 12;
static constexpr int RUN_BACK_DOWN = 13;
static constexpr int RUN_TOP_DOWN = 14;

static constexpr int RUN_DOWN_RIGHT = 21;
static constexpr int RUN_DOWN_LEFT = 22;
static constexpr int RUN_DOWN_BACK = 23;

static constexpr int RUN_CUBE_RIGHT = 31;
static constexpr int RUN_CUBE_LEFT = 32;
static constexpr int RUN_CUBE_BACK = 33;

static constexpr int RUN_RESET_RIGHT = 41;
static constexpr int RUN_RESET_LEFT = 42;
static constexpr int RUN_RESET_BACK = 43;

static constexpr int RUN_ALIGN_CUBE = 60;

// -------------------------------------------------------------------
//                           HELP TEXT
// -------------------------------------------------------------------

extern const char runHelp[] PROGMEM;

// -------------------------------------------------------------------
//                       FUNCTION DECLARATIONS
// -------------------------------------------------------------------

// ---- core motion / helpers
bool prepBaseForRotation(double nextBaseMoveRelative);
bool rotateBaseRelative(double baseMoveRelative, bool gripperOn = false);
bool alignCube();
bool liftCube();
bool lowerCube();

// ---- robot move dispatcher
bool cmd_run(int argc, double *argv);
bool robot_move_callback(const String &mv);

// ---- orientation commands
bool cmd_getori_data(int argc, double *argv);
bool cmd_clear_ori_data(int argc, double *argv);
bool cmd_restore_ori(int argc, double *argv);

// ---- movement commands
bool cmd_move_xy(int argc, double *argv);
bool cmd_move_x(int argc, double *argv);
bool cmd_move_y(int argc, double *argv);
bool cmd_move_deg(int argc, double *argv);
bool cmd_move_ticks(int argc, double *argv);
bool cmd_move_per(int argc, double *argv);
bool cmd_move_clamp(int argc, double *argv);
bool cmd_move_gripper(int argc, double *argv);
bool cmd_move_wrist_vert(int argc, double *argv);

// ---- servo config
bool cmd_set_min(int argc, double *argv);
bool cmd_set_max(int argc, double *argv);

// ---- color / cube read
bool cmd_color(int argc, double *argv);
bool cmd_read_one_color(int argc, double *argv);
bool cmd_read_one_face_colors(int argc, double *argv);
bool cmd_read_cube_colors_string(const String &mode);
bool cmd_getcolor_data(int argc, double *argv);

// ---- callbacks
char read_one_color_cb(int slot);

// ---- info / diagnostics
bool cmd_read(int argc, double *argv);
bool cmd_info(int argc, double *argv);
bool cmd_ledon(int argc, double *argv);
bool cmd_ledoff(int argc, double *argv);

void print_info(uint8_t id);
void print_colors_detail(char *txt);


// base / cube positioning
bool cmd_run_zero();
bool cmd_run_right_down();
bool cmd_run_left_down();
bool cmd_run_top_down();
bool cmd_run_back_down();

// bottom-layer only rotations
bool cmd_run_down_layer(int run_no);

// full-cube rotations
bool cmd_run_cube_right();
bool cmd_run_cube_left();
bool cmd_run_cube_back();

// base reset helpers
bool cmd_run_reset_right();
bool cmd_run_reset_left();
bool cmd_run_reset_back();

// cube alignment
bool cmd_run_align_cube();
