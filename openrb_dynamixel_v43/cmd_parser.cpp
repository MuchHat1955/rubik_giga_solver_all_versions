#include "cmd_parser.h"
#include "movement.h"
#include "servos.h"
#include "color_read.h"
#include "ori.h"

void print_info(uint8_t id);

extern double max_xmm;
extern double max_ymm;
extern double min_ymm;
extern double speed;
extern CubeOri ori;

// -------------------------------------------------------------------
//                      COMMAND TABLE
// -------------------------------------------------------------------

// RUN command constants (placed next to help text)
static constexpr int RUN_ZERO = 0;

static constexpr int RUN_RIGHT_DOWN = 11;
static constexpr int RUN_LEFT_DOWN = 12;
static constexpr int RUN_BACK_DOWN = 13;

static constexpr int RUN_BOTTOM_RIGHT = 21;
static constexpr int RUN_BOTTOM_LEFT = 22;
static constexpr int RUN_BOTTOM_BACK = 23;

static constexpr int RUN_CUBE_RIGHT = 31;
static constexpr int RUN_CUBE_LEFT = 32;
static constexpr int RUN_CUBE_BACK = 33;

static constexpr int RUN_RESET_RIGHT = 41;
static constexpr int RUN_RESET_LEFT = 42;
static constexpr int RUN_RESET_BACK = 43;

static constexpr int RUN_READ_COLORS = 50;

static constexpr int RUN_ALIGN_CUBE = 60;

const char runHelp[] PROGMEM =
  "RUN <no>\n"
  "      0 pos zero |\n"
  "     11 right down   | 12 left down    | 13 back down\n"
  "     21 bottom right | 22 bottom right | 23 bottom back\n"
  "     31 cube right   | 32 cube left    | 33 cube back\n"
  "     41 reset right  | 42 reset left   | 143 reset back\n"
  "     50 read colors\n"
  "     60 align\n";

struct CommandEntry {
  const char *name;
  const char *fmt;
  bool (*handler)(int argc, double *argv);
  const char *desc;
};

static CommandEntry command_table[] = {
  { "VERBOSEON", "", cmd_verbose_on, "VERBOSEON - enable verbose output" },
  { "VERBOSEOFF", "", cmd_verbose_off, "VERBOSEOFF - disable verbose output" },

  { "SETMIN", "%d %d", cmd_set_min, "SETMIN <id> <ticks> - set the min ticks" },
  { "SETMAX", "%d %d", cmd_set_max, "SETMAX <id> <ticks> - set the max ticks" },

  { "MOVETICKS", "%d %d", cmd_move_ticks, "MOVEDEG <id> <ticks goal> - move one servo to ticks (not smooth)" },
  { "MOVEDEG", "%d %f", cmd_move_deg, "MOVEDEG <id> <deg goal> - move one servo to degree (smooth)" },
  { "MOVEPER", "%d %f", cmd_move_per, "MOVEPER <id> <per goal> - move one servo to percent (smooth)" },

  { "MOVEYMM", "%f", cmd_move_y, "MOVEYMM <float mm> - vertical move (42 to 102)" },
  { "MOVEXMM", "%f", cmd_move_x, "MOVEXMM <float mm> - lateral move (-30 to 30)" },
  { "MOVEXYMM", "%f %f", cmd_move_xy, "MOVEXYMM <float mm> <float mm> - lateral then vertical move (-25 to 25)(42 to 102)" },
  { "MOVEGRIPPER", "%f", cmd_move_gripper, "MOVEGRIPPER <percentage> - move both grips to percentage (0 to 100)" },
  { "MOVEWRISTVERTDEG", "%f", cmd_move_wrist_vert, "MOVEWRISTVERTDEG <deg> - move wrist relative to vertical (-5 to 185)" },
  { "CLAMP", "", cmd_move_clamp, "CLAMP - clamp gripper" },

  { "RUN", "%d", cmd_run, runHelp },

  { "READ", "%d", cmd_read, "READ <id> - show servo summary status" },
  { "INFO", "%d", cmd_info, "INFO <id> - show servo full status" },

  { "COLOR", "%d", cmd_color, "COLOR <count> - read color <count> times" },

  { "LEDON", "%d", cmd_ledon, "LEDON <id> - turn servo LED on" },
  { "LEDOFF", "%d", cmd_ledoff, "LEDOFF <id> - turn servo LED off" },

  // NEW: string-based move commands using CubeOri
  { "MOVEROBOT", "<moves>", nullptr, "MOVEROBOT <moves> - robot moves (y+, y', z2, d+, etc; space-separated list allowed)" },
  { "MOVECUBE", "<moves>", nullptr, "MOVECUBE <moves> - cube moves (F, R', U2, etc; space-separated list allowed)" },

  { "GETORI", "", cmd_getori, "GETORI - print orientation move log" },
  { "RESETORI", "", cmd_resetori, "RESETORI - reset orientation and clear log" },

  { "HELP", "", cmd_help, "HELP list of commands" },
};

static constexpr int COMMAND_COUNT = sizeof(command_table) / sizeof(command_table[0]);

// -------------------------------------------------------------------
//                            PARSE HELPERS
// -------------------------------------------------------------------

static int parse_args(const String &line, const char *fmt, double *out, int max_args, String *raw_tokens = nullptr) {
  // Skip command word
  int space_idx = line.indexOf(' ');
  if (space_idx < 0) return 0;
  String params = line.substring(space_idx + 1);
  params.trim();

  if (params.length() == 0) return 0;

  // Tokenize (safe for mixed types)
  int argc = 0;
  int pos = 0;
  while (argc < max_args) {
    int next_space = params.indexOf(' ', pos);
    String token = (next_space == -1) ? params.substring(pos) : params.substring(pos, next_space);
    token.trim();
    if (token.length() == 0) break;
    if (raw_tokens) raw_tokens[argc] = token;
    out[argc] = token.toDouble();
    argc++;
    if (next_space == -1) break;
    pos = next_space + 1;
  }
  return argc;
}

// -------------------------------------------------------------------
//                       COMMAND HANDLERS (LOGIC ONLY)
// -------------------------------------------------------------------

// ------------------------------------------------------------
// GETORI
// ------------------------------------------------------------
bool cmd_getori(int argc, double *argv) {
  serial_printf("ORI LOG: %s\n", ori.get_orientation_log().c_str());
  return true;
}

// ------------------------------------------------------------
// RESETORI
// ------------------------------------------------------------
bool cmd_resetori(int argc, double *argv) {
  ori.reset();
  serial_printf("Orientation reset.\n");
  return true;
}

bool cmd_verbose_on(int argc, double *argv) {
  verboseOn = true;
  Serial.println("Verbose ON");
  return true;
}

bool cmd_verbose_off(int argc, double *argv) {
  verboseOn = false;
  Serial.println("Verbose OFF");
  return true;
}

bool cmd_move_xy(int argc, double *argv) {
  double goal_xmm = argv[0];
  if (goal_xmm < -max_xmm || goal_xmm > max_xmm) {
    serial_printf("ERR invalid x mm: %.2f expected range (%.2f mm to %.2f mm)\n", goal_xmm, -max_xmm, max_xmm);
    return false;
  }
  double goal_ymm = argv[1];
  if (goal_ymm < min_ymm || goal_ymm > max_ymm) {
    serial_printf("ERR invalid y mm: %.2f expected range (%.2f mm to %.2f mm)\n", goal_ymm, min_ymm, max_ymm);
    return false;
  }
  if (!cmdMoveXmm(goal_xmm)) return false;
  if (!cmdMoveYmm(goal_ymm)) return false;
  return true;
}

bool cmd_move_deg(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  double goal_deg = (double)argv[1];

  if (goal_deg < -185.0 || goal_deg > 360.0) {
    serial_printf("ERR invalid servo deg: %.2f expected range (-185.0 deg to 360.0 deg)\n", goal_deg);
    return false;
  }
  int goal_ticks = per2ticks(id, goal_deg);
  serial_printf_verbose("cmd_move_deg: id=%d deg=%d\n", id, goal_deg);

  if (!cmdMoveServoDeg((uint8_t)id, goal_deg)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_move_ticks(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  int goal_ticks = (int)argv[1];
  serial_printf_verbose("cmd_move_ticks: id=%d ticks=%d\n", id, goal_ticks);

  if (!safeSetGoalPosition(id, goal_ticks)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_move_per(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  double goal_per = argv[1];
  if (goal_per < -15.0 || goal_per > 115.0) {
    serial_printf("ERR invalid servo percentage: %.2f expected range (-15.0 per to 115.0 per)\n", goal_per);
    return false;
  }

  double goal_deg = per2deg(id, goal_per);
  serial_printf_verbose("cmd_move_per: id=%d per=%d deg=%d\n", id, goal_per, goal_deg);

  if (!cmdMoveServoDeg((uint8_t)id, goal_deg)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_set_min(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  int t = (int)argv[1];

  if (auto *s = find_servo(id)) {
    serial_printf_verbose("cmd_set_min: id=%d ticks=%d\n", id, t);
    s->set_min_ticks(t);
    return true;
  }
  return false;
}

bool cmd_set_max(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  int t = (int)argv[1];

  if (auto *s = find_servo(id)) {
    serial_printf_verbose("cmd_set_max: id=%d ticks=%d\n", id, t);
    s->set_max_ticks(t);
    return true;
  }
  return false;
}

bool cmd_move_y(int argc, double *argv) {
  double goal_mm = argv[0];
  if (goal_mm < min_ymm || goal_mm > max_ymm) {
    serial_printf("ERR invalid y mm: %.2f expected range (%.2f mm to %.2f mm)\n", goal_mm, min_ymm, max_ymm);
    return false;
  }

  serial_printf_verbose("cmd_move_y: y=%.2fmm\n", goal_mm);
  if (!cmdMoveYmm(goal_mm)) return false;
  return true;
}

bool cmd_move_x(int argc, double *argv) {
  double goal_mm = argv[0];
  if (goal_mm < -max_xmm || goal_mm > max_xmm) {
    serial_printf("ERR invalid x mm: %.2f expected range (%.2f mm to %.2f)\n", goal_mm, -max_xmm, max_xmm);
    return false;
  }

  serial_printf_verbose("cmd_move_x: x=%.2fmm\n", goal_mm);
  if (!cmdMoveXmm(goal_mm)) return false;
  return true;
}

bool cmd_move_clamp(int argc, double *argv) {
  if (!dxl.ping(ID_GRIP1) || !dxl.ping(ID_GRIP2)) return false;

  serial_printf_verbose("cmd_move_clamp\n");
  if (!cmdMoveGripperClamp()) return false;
  print_servo_status(ID_GRIP1);
  print_servo_status(ID_GRIP2);
}

bool cmd_move_gripper(int argc, double *argv) {
  if (!dxl.ping(ID_GRIP1) || !dxl.ping(ID_GRIP2)) return false;

  double goal_deg = argv[0];

  if (goal_deg < -5.0 || goal_deg > 115.0) {
    serial_printf("ERR invalid gripper percentage: %.2f expected range (-5.0 per to 105 per)\n", goal_deg);
    return false;
  }

  serial_printf_verbose("cmd_move_gripper: deg=%.2f\n", goal_deg);
  if (!cmdMoveGripperPer(goal_deg)) return false;
  print_servo_status(ID_GRIP1);
  print_servo_status(ID_GRIP2);
  return true;
}

bool cmd_move_wrist_vert(int argc, double *argv) {
  if (!dxl.ping(ID_WRIST) || !dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2)) return false;

  double goal_deg = argv[0];

  if (goal_deg < -95 || goal_deg > 95) {
    serial_printf("ERR invalid wrist deg: %.2f expected range (-95deg to 95deg)\n", goal_deg);
    return false;
  }

  serial_printf_verbose("cmd_move_wrist: deg=%.2f\n", goal_deg);

  if (!cmdMoveWristDegVertical(goal_deg)) return false;
  print_servo_status(ID_WRIST);
  return true;
}

bool cmd_color(int argc, double *argv) {
  int read_count = 1;

  if (argc == 1) {
    read_count = (int)argv[0];
  }
  if (read_count > 10) read_count = 10;
  if (read_count < 1) read_count = 1;

  serial_printf_verbose("cmd_color: count=%d\n", read_count);

  for (int i = 0; i < read_count; i++) {
    serial_printf("calling read color i=%d\n", i);
    String crrColor = read_color();
    serial_printf_verbose("COLOR READ clr=%s\n", crrColor.c_str());
    delay(555);
  }

  return true;
}

bool cmd_help(int argc, double *argv) {
  Serial.println();
  Serial.println(get_help_text());
  Serial.println();
  return true;
}

// ---- y poses
#define Y_CENTER 73.0
#define Y_ALIGN 68
#define Y_MID 97
#define Y_UP 117
#define Y_DOWN 33
#define Y_ROTATE_BASE 92
#define Y_ABOVE_DROP 86
#define Y_DROP 78

// ---- y read color poses
#define Y_C_TOP 65
#define Y_C_MID 48

// ---- x poses
#define X_CENTER -2.0

// ---- x read color poses
#define X_C_LEFT -14.0
#define X_C_RIGHT 16.0
#define X_C_CENTER 4.0

// ---- w poses
#define W_HORIZ -85
#define W_RIGHT 5

// ---- g poses
#define G_OPEN 22
#define G_WIDE_OPEN 0
#define G_CLOSE 100
#define G_SOFT_CLOSE 45
#define G_ALIGN_LEFT 100
#define G_ALIGN_RIGHT 100

// ---- b poses
#define B_CENTER 0
#define B_RIGHT 90
#define B_LEFT -90
#define B_BACK -180
#define B_ERR 3

#define B_TOL 3

bool prepBaseForRotation(int baseMove) {
  double b_pos = getPos_deg(ID_BASE);

  bool isBaseCenter = (b_pos > B_CENTER - B_TOL && b_pos < B_CENTER + B_TOL);
  bool isBaseRight = (b_pos > B_RIGHT - B_TOL && b_pos < B_RIGHT + B_TOL);
  bool isBaseLeft = (b_pos > B_LEFT - B_TOL && b_pos < B_LEFT + B_TOL);
  bool isBaseBack = (b_pos > B_BACK - B_TOL && b_pos < B_BACK + B_TOL);

  if (baseMove == B_RIGHT) {
    if (isBaseCenter) return true;
    if (isBaseLeft) return true;
    if (isBaseBack) return true;
  }
  if (baseMove == B_LEFT) {
    if (isBaseCenter) return true;
    if (isBaseRight) return true;
    if (isBaseLeft) return true;
  }
  if (baseMove == B_BACK) {
    if (isBaseCenter) return true;
    if (isBaseRight) return true;
  }

  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ)) return false;
  if (!cmdMoveGripperClamp()) return false;
  if (!cmdMoveYmm(Y_ROTATE_BASE)) return false;
  if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) return false;
  if (!lowerCube()) return false;
  return true;
}

/*
#define B_RIGHT 90
#define B_LEFT -90
#define B_BACK -180
*/

bool rotateBase(int baseMove) {
  if (!dxl.ping(ID_BASE)) return false;

  if (baseMove == B_CENTER) return true;

  if (!prepBaseForRotation(baseMove)) return false;
  return cmdMoveServoDeg(ID_BASE, baseMove);
}

bool alignCube() {
  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ)) return false;

  if (!cmdMoveServoPer(ID_GRIP2, G_ALIGN_LEFT)) return false;
  if (!cmdMoveServoDeg(ID_GRIP2, G_OPEN)) return false;
  if (!cmdMoveServoDeg(ID_GRIP1, G_ALIGN_RIGHT)) return false;  //TODOadjust
  if (!cmdMoveServoDeg(ID_GRIP1, G_OPEN)) return false;
  if (!cmdMoveGripperPer(G_OPEN)) return false;

  if (!cmdMoveYmm(Y_ALIGN)) return false;
  if (!cmdMoveGripperClamp()) return false;
  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;

  return true;
}

bool lowerCube() {
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_ABOVE_DROP)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_DROP)) return false;
  if (!cmdMoveGripperPer(G_OPEN)) return false;
  return true;
}

bool liftCube() {
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_ABOVE_DROP)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  return true;
}

bool cmd_run(int argc, double *argv) {
  if (argc < 1) {
    serial_printf("ERR: TESTNO missing argument\n");
    return false;
  }

  speed = 1.0;

  int run_no = (int)argv[0];

  // #define RUN_ZERO 0

  // the standby position
  if (run_no == RUN_ZERO) {
    if (!cmdMoveGripperPer(G_WIDE_OPEN)) return false;
    if (!rotateBase(B_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;
    if (!cmdMoveGripperPer(G_OPEN)) return false;
    if (!cmdMoveYmm(Y_DOWN)) return false;
    if (!cmdMoveGripperPer(G_SOFT_CLOSE)) return false;
    if (!cmdMoveGripperPer(G_WIDE_OPEN)) return false;
    return true;
  }

  // #define RUN_RIGHT_DOWN 11
  // #define RUN_LEFT_DOWN 12
  // #define RUN_BACK_DOWN 13

  // bring right face to down
  if (run_no == RUN_RIGHT_DOWN) {
    // prep
    if (!cmdMoveGripperPer(G_OPEN)) return false;

    // lift cube to rotate vertically
    if (!cmdMoveYmm(Y_CENTER)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveGripperClamp()) return false;
    if (!liftCube()) return false;
    if (!cmdMoveYmm(Y_UP)) return false;

    // rotate cube vertical right
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_RIGHT)) return false;

    // lower cube
    if (!lowerCube()) return false;
    if (!cmdMoveGripperPer(G_OPEN)) return false;

    // reset
    if (!cmdMoveYmm(Y_CENTER + 3)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;

    return true;
  }
  // bring left face to down
  if (run_no == RUN_LEFT_DOWN) {
    // prep
    if (!cmdMoveGripperPer(G_OPEN)) return false;

    // lift cube to rotate vertically
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveYmm(Y_CENTER + 3)) return false;
    if (!cmdMoveWristDegVertical(W_RIGHT)) return false;
    if (!cmdMoveYmm(Y_CENTER)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveGripperClamp()) return false;
    if (!liftCube()) return false;
    if (!cmdMoveYmm(Y_UP)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;

    // rotate cube vertical left
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;

    // lower cube
    if (!lowerCube()) return false;
    if (!cmdMoveGripperPer(G_OPEN)) return false;

    // reset
    if (!cmdMoveYmm(Y_CENTER + 3)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;

    return true;
  }

  // bring back face to down
  if (run_no == RUN_BACK_DOWN) {
    // prep
    if (!cmdMoveGripperPer(G_OPEN)) return false;
    // if (!prepBaseForRotation(B_RIGHT)) return false; // done in cmdMoveBaseLeft

    // rotate base to bring back face on the right
    if (!rotateBase(B_LEFT)) return false;

    // lift cube to rotate vertically
    if (!cmdMoveYmm(Y_CENTER)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveGripperClamp()) return false;
    if (!liftCube()) return false;
    if (!cmdMoveYmm(Y_UP)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;

    // rotate cube vertical right
    if (!cmdMoveWristDegVertical(W_RIGHT)) return false;

    // lower cube
    if (!lowerCube()) return false;
    if (!cmdMoveGripperPer(G_OPEN)) return false;

    // reset
    if (!cmdMoveYmm(Y_CENTER + 3)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;

    return true;
  }

  // #define RUN_BOTTOM_RIGHT 21
  // #define RUN_BOTTOM_LEFT 22
  // #define RUN_BOTTOM_BACK 23

  // rotate bottom layer only: right, left, back
  if (run_no == RUN_BOTTOM_RIGHT ||  //
      run_no == RUN_BOTTOM_LEFT ||   //
      run_no == RUN_BOTTOM_BACK) {
    // prep
    if (!cmdMoveGripperPer(G_OPEN)) return false;
    // prep the base
    if ((run_no == RUN_BOTTOM_RIGHT))
      if (!prepBaseForRotation(B_RIGHT)) return false;
    if ((run_no == RUN_BOTTOM_LEFT))
      if (!prepBaseForRotation(B_LEFT)) return false;
    if ((run_no == RUN_BOTTOM_BACK))
      if (!prepBaseForRotation(B_BACK)) return false;

    // move grip above bottom layer
    if (!cmdMoveYmm(Y_CENTER)) return false;  //TODO add resetWristHoriz and resetWristVert maybe
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;
    if (!cmdMoveYmm(Y_MID)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveGripperClamp()) return false;

    // rotate base
    if (run_no == RUN_BOTTOM_RIGHT) {
      if (!cmdMoveServoDeg(ID_BASE, B_RIGHT)) return false;
    }
    if (run_no == RUN_BOTTOM_LEFT) {
      if (!cmdMoveServoDeg(ID_BASE, B_LEFT)) return false;
    }
    if (run_no == RUN_BOTTOM_BACK) {
      if (!cmdMoveServoDeg(ID_BASE, B_BACK)) return false;
    }

    // release clamp
    if (!cmdMoveGripperPer(G_OPEN)) return false;
    if (!cmdMoveYmm(Y_CENTER)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;

    return true;
  }

  // #define RUN_CUBE_RIGHT 31
  // #define RUN_CUBE_LEFT 32
  // #define RUN_CUBE_BACK 33

  // rotate full cube: right, left, back
  if (run_no == RUN_CUBE_RIGHT) {
    if (!rotateBase(B_RIGHT)) return false;
    return true;
  }
  // 32  turn cube left to front
  if (run_no == RUN_CUBE_LEFT) {
    if (!rotateBase(B_LEFT)) return false;
    return true;
  }
  // 33  turn cube back to front
  if (run_no == RUN_CUBE_BACK) {
    if (!rotateBase(B_BACK)) return false;
    return true;
  }

  // #define RUN_READ_COLORS 40

  // read colors on front face
  if (run_no == RUN_READ_COLORS) {
    String crrColor = "na";

    while (1) {
      if (!cmdMoveGripperPer(G_WIDE_OPEN)) break;
      if (!cmdMoveWristDegVertical(W_HORIZ)) break;
      if (!cmdMoveXmm(X_CENTER)) break;
      if (!cmdMoveGripperPer(G_WIDE_OPEN)) break;

      speed = 0.25;

      // top row --------------------------------------------------
      if (!cmdMoveYmm(Y_C_TOP)) break;

      // top right ------------------------------------------------
      if (!cmdMoveXmm(X_C_RIGHT)) break;
      if (!cmdMoveWristDegVertical(W_HORIZ)) break;
      crrColor = read_color();
      serial_printf("COLOR READ TOP_R=%s\n", crrColor.c_str());

      // top center ------------------------------------------------
      if (!cmdMoveXmm(X_C_CENTER)) break;
      if (!cmdMoveWristDegVertical(W_HORIZ)) break;
      crrColor = read_color();
      serial_printf("COLOR READ TOP_C=%s\n", crrColor.c_str());

      // top left --------------------------------------------------
      if (!cmdMoveXmm(X_C_LEFT)) break;
      if (!cmdMoveWristDegVertical(W_HORIZ)) break;
      crrColor = read_color();
      serial_printf("COLOR READ TOP_L=%s\n", crrColor.c_str());

      // mid row ---------------------------------------------------
      if (!cmdMoveYmm(Y_C_MID)) break;

      // mid left
      if (!cmdMoveXmm(X_C_LEFT)) break;
      if (!cmdMoveWristDegVertical(W_HORIZ)) break;
      crrColor = read_color();
      serial_printf("COLOR READ MID_L=%s\n", crrColor.c_str());

      // mid center
      if (!cmdMoveXmm(X_CENTER)) break;
      if (!cmdMoveWristDegVertical(W_HORIZ)) break;
      crrColor = read_color();
      serial_printf("COLOR READ MID_C=%s\n", crrColor.c_str());

      // mid right
      if (!cmdMoveXmm(X_C_RIGHT)) break;
      if (!cmdMoveWristDegVertical(W_HORIZ)) break;
      crrColor = read_color();
      serial_printf("COLOR READ MID_R=%s\n", crrColor.c_str());

      // back to main pos
      if (!cmdMoveXmm(X_CENTER)) break;
      if (!cmdMoveXmm(Y_CENTER)) break;
      if (!cmdMoveWristDegVertical(W_HORIZ)) break;

      if (!cmdMoveGripperPer(G_WIDE_OPEN)) break;
      speed = 1.0;
      return true;
    }

    speed = 1.0;
    return false;
  }

  // #define RUN_RESET_RIGHT 51
  // #define RUN_RESET_LEFT 52
  // #define RUN_RESET_BACK 53

  // reset base needed to turn base right, left, back
  if (run_no == RUN_RESET_RIGHT) {
    if (!prepBaseForRotation(B_RIGHT)) return false;
    return true;
  }
  // reset base needed to turn base left
  if (run_no == 82) {
    if (!prepBaseForRotation(B_LEFT)) return false;
    return true;
  }
  // reset base  needed to turn base 180
  if (run_no == RUN_RESET_LEFT) {
    if (!prepBaseForRotation(B_BACK)) return false;
    return true;
  }

  // #define RUN_ALIGN_CUBE 60

  // align cube
  if (run_no == RUN_ALIGN_CUBE) {
    if (!alignCube()) return false;
    return true;
  }

  speed = 1.0;

  return false;
}

bool cmd_read(int argc, double *argv) {
  print_servo_status((argc > 0) ? (int)argv[0] : 0);
  return true;
}

bool cmd_info(int argc, double *argv) {
  print_info((uint8_t)argv[0]);
  return true;
}

bool cmd_ledon(int argc, double *argv) {
  dxl.ledOn((uint8_t)argv[0]);
  return true;
}

bool cmd_ledoff(int argc, double *argv) {
  dxl.ledOff((uint8_t)argv[0]);
  return true;
}

// -------------------------------------------------------------------
// INFO <id> : print key control-table data for one servo
// -------------------------------------------------------------------
void print_info(uint8_t id) {
  if (!dxl.ping(id)) {
    serial_printf("ERR Servo %d not found\n", id);
    return;
  }

  bool _ok = servo_ok(id);
  serial_printf("SERVO OK id=%d ok=%s\n", id, _ok ? "ok" : "not ok");
  if (!_ok) {
    serial_printf("SERVO RESETING id=%d\n", id);
    reset_servo(id);
    serial_printf("SERVO OK id=%d ok=%s\n", id, _ok ? "ok" : "not ok");
  }

  int op = dxl.readControlTableItem(ControlTableItem::OPERATING_MODE, id);
  int drv = dxl.readControlTableItem(ControlTableItem::DRIVE_MODE, id);
  int pv = dxl.readControlTableItem(ControlTableItem::PROFILE_VELOCITY, id);
  int pa = dxl.readControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id);
  int pos = dxl.getPresentPosition(id);
  int minL = dxl.readControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id);
  int maxL = dxl.readControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id);

  float rpm = pv * PV_UNIT_RPM;
  float tps = pvToTicksPerSec(pv);
  float spanTicks = (maxL > minL) ? (float)(maxL - minL) : TICKS_PER_REV;
  float spanDeg = spanTicks * DEG_PER_TICK;

  serial_printf("INFO id=%d", id);
  serial_printf(" op_mode=%d", op);
  serial_printf(" drive_mode=%d bit0=%s profil=", drv, (drv & 0x01) ? "TIME" : "VELOCITY");
  serial_printf(" profile_vel=%d  rpm=%.3frpm, tps=%.1f ticks_s)", pv, rpm, tps);
  serial_printf(" prifile_accel=%d", pa);
  serial_printf(" pos_min=%d", minL);
  serial_printf(" pos_max=%d", maxL);
  serial_printf(" span_deg=%.1f", spanDeg);
  serial_printf(" pos_present=%d\n", pos);
}

// -------------------------------------------------------------------
//                        GET HELP TEXT
// -------------------------------------------------------------------

String
get_help_text() {
  String help = "Supported Commands:\n";
  for (int i = 0; i < COMMAND_COUNT; i++) {
    help += "  ";
    help += command_table[i].desc;
    help += "\n";
  }
  return help;
}

// -------------------------------------------------------------------
//                    PROCESS SERIAL COMMAND (DISPATCHER)
// -------------------------------------------------------------------

void process_serial_command(String &line) {
  String U = line;
  U.trim();
  U.toUpperCase();

  // derive count of args and id flag from format
  auto derive_format_info = [](const char *fmt, int &min_args) {
    min_args = 0;
    for (const char *p = fmt; *p; p++) {
      if (*p == '%') {
        min_args++;
      }
    }
  };

  for (int i = 0; i < COMMAND_COUNT; i++) {
    const CommandEntry &cmd = command_table[i];
    if (U.startsWith(cmd.name)) {
      // -----------------------------------------------------------
      // Special handling for MOVEROBOT and MOVECUBE (string commands)
      // -----------------------------------------------------------
      if (strcmp(cmd.name, "MOVEROBOT") == 0 || strcmp(cmd.name, "MOVECUBE") == 0) {

        // Extract raw params after command
        int space_idx = line.indexOf(' ');
        if (space_idx < 0) {
          serial_printf("ERR %s missing argument\n", cmd.name);
          return;
        }
        String params = line.substring(space_idx + 1);
        params.trim();
        if (params.length() == 0) {
          serial_printf("ERR %s missing argument\n", cmd.name);
          return;
        }

        // ----------- UNIFIED LOGGING (same as numeric commands) ----------
        serial_printf_verbose("---- START %s params: %s ----\n", cmd.name, params.c_str());

        serial_printf("%s %s START | ", cmd.name, params.c_str());
        print_all_status();

        bool ok = false;

        if (strcmp(cmd.name, "MOVECUBE") == 0) {
          // pass full raw string — ori parses sequence itself
          serial_printf("calling ori.cube_move\n", params.c_str());
          ok = ori.cube_move(params);
        } else if (strcmp(cmd.name, "MOVEROBOT") == 0) {
          // pass full raw string — NO splitting
          serial_printf("calling ori.robot_move\n", params.c_str());
          ok = ori.robot_move(params);
        }

        serial_printf("%s %s END completed=%d ", cmd.name, params.c_str(), ok ? 1 : 0);
        print_all_status();

        return;
      }

      // ---------------- Existing numeric-command path ----------------
      serial_printf_verbose(
        "---- START %s params: %s ----\n", cmd.name, line.c_str());

      int min_args = 0;
      derive_format_info(cmd.fmt, min_args);

      double argv[8] = { 0 };
      int argc = 0;
      String raw[8];
      argc = parse_args(line, cmd.fmt, argv, 8, raw);

      // ---------------- Argument validation ----------------
      if (argc < min_args) {
        serial_printf("ERR %s usage is %s\n", cmd.name, cmd.desc);
        return;
      }

      // ---------------- Execute Command ----------------
      double p1 = 0;
      if (argc > 0) p1 = (double)argv[0];
      serial_printf("%s %.1f START | ", cmd.name, p1);
      print_all_status();
      bool ok = cmd.handler(argc, argv);
      serial_printf("%s %.1f END completed=%d ", cmd.name, p1, ok ? 1 : 0);
      print_all_status();
      return;
    }
  }

  serial_printf("ERR Unknown command\n\n");
  serial_printf("%s\n\n", get_help_text().c_str());
}
