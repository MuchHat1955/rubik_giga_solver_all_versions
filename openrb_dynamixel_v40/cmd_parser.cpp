#include "cmd_parser.h"
#include "movement.h"
#include "servos.h"
#include "color_read.h"

void print_info(uint8_t id);

extern double max_xmm;
extern double max_ymm;
extern double min_ymm;
extern double speed;

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
//                       ID RESOLVER (Name or Number)
// -------------------------------------------------------------------

static int resolve_id_or_name(const String &token) {
  // purely numeric
  bool numeric = true;
  for (unsigned int i = 0; i < token.length(); i++) {
    if (!isdigit(token[i])) {
      numeric = false;
      break;
    }
  }
  if (numeric) return token.toInt();

  // try by name
  ServoConfig *s = find_servo(token.c_str());
  if (s) return s->get_id();

  serial_printf_verbose(
    "Unknown servo name '%s'\n", token.c_str());
  return 0;
}

// -------------------------------------------------------------------
//                       COMMAND HANDLERS (LOGIC ONLY)
// -------------------------------------------------------------------

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

bool cmd_move_center(int argc, double *argv) {
  return false;
  // could cause colisions now TODO

  for (int i = 0; i < SERVO_COUNT; i++)
    safeSetGoalPosition(all_servos[i]->get_id(), 2048);
  return true;
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
#define Y_ROTATE_BASE 88

// ---- y read color poses
#define Y_C_TOP 65
#define Y_C_MID 48

// ---- x poses
#define X_CENTER -1.5

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

//TODO adjust grip left and right to be simetric

bool resetBase() {
  double b_pos = getPos_deg(ID_BASE);
  if (b_pos > B_CENTER - 3 && b_pos < B_CENTER + 3) return true;

  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ)) return false;
  if (!cmdMoveGripperPer(G_CLOSE)) return false;
  if (!cmdMoveYmm(Y_MID)) return false;
  if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) return false;
  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  return true;
}

bool alignCube() {
  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ)) return false;

  if (!cmdMoveServoPer(ID_GRIP2, G_ALIGN_LEFT)) return false;
  if (!cmdMoveServoDeg(ID_GRIP2, G_OPEN)) return false;
  if (!cmdMoveServoDeg(ID_GRIP1, G_ALIGN_RIGHT)) return false;  //TODOTODO this is not working
  if (!cmdMoveServoDeg(ID_GRIP1, G_OPEN)) return false;
  if (!cmdMoveGripperPer(G_OPEN)) return false;

  if (!cmdMoveYmm(Y_ALIGN)) return false;
  if (!cmdMoveGripperPer(G_CLOSE)) return false;
  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveYmm(Y_CENTER)) return false;
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

  if (run_no == 0) {
    if (!cmdMoveGripperPer(G_WIDE_OPEN)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveYmm(Y_ROTATE_BASE)) return false;
    if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;
    if (!cmdMoveGripperPer(G_OPEN)) return false;
    if (!cmdMoveYmm(Y_DOWN)) return false;
    if (!cmdMoveGripperPer(G_SOFT_CLOSE)) return false;
    if (!cmdMoveGripperPer(G_WIDE_OPEN)) return false;
    return true;
  }
  // bring faces to base
  if (run_no == 1) {
    if (!cmdMoveGripperPer(G_OPEN)) return false;
    if (!resetBase()) return false;

    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveYmm(Y_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;
    if (!cmdMoveGripperPer(G_CLOSE)) return false;
    if (!cmdMoveYmm(Y_UP)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;

    if (!cmdMoveWristDegVertical(W_RIGHT)) return false;

    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveYmm(Y_CENTER)) return false;
    if (!cmdMoveGripperPer(G_OPEN)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveYmm(Y_CENTER)) return false;
    if (!cmdMoveGripperPer(G_OPEN)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;
    if (!cmdMoveXmm(Y_CENTER)) return false;
    return true;
  }
  // bring faces to base
  if (run_no == 2) {
    if (!cmdMoveGripperPer(G_OPEN)) return false;
    if (!resetBase()) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveYmm(Y_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_RIGHT)) return false;
    if (!cmdMoveGripperPer(G_CLOSE)) return false;
    if (!cmdMoveYmm(Y_UP)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;

    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;

    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveYmm(Y_CENTER)) return false;
    if (!cmdMoveGripperPer(G_OPEN)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveYmm(Y_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;
    if (!cmdMoveXmm(Y_CENTER)) return false;
    return true;
  }
  // rotate base
  if (run_no == 3 || run_no == 4 || run_no == 5) {
    if (!cmdMoveGripperPer(G_OPEN)) return false;
    if (!resetBase()) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveYmm(Y_MID)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;
    if (!cmdMoveGripperPer(G_CLOSE)) return false;

    if (run_no == 3) {
      if (!cmdMoveServoDeg(ID_BASE, B_RIGHT + B_ERR)) return false;
      if (!cmdMoveGripperPer(G_OPEN)) return false;
      if (!cmdMoveServoDeg(ID_BASE, B_RIGHT)) return false;
    }
    if (run_no == 4) {
      if (!cmdMoveServoDeg(ID_BASE, B_LEFT - B_ERR)) return false;
      if (!cmdMoveGripperPer(G_OPEN)) return false;
      if (!cmdMoveServoDeg(ID_BASE, B_LEFT)) return false;
    }
    if (run_no == 5) {
      if (!cmdMoveServoDeg(ID_BASE, B_BACK - B_ERR)) return false;
      if (!cmdMoveGripperPer(G_OPEN)) return false;
      if (!cmdMoveServoDeg(ID_BASE, B_BACK)) return false;
    }

    if (!cmdMoveGripperPer(G_OPEN)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveYmm(Y_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ)) return false;

    // align TODO
    // if (!alignCube()) return false;
    return true;
  }
  // reset base
  if (run_no == 6) {
    if (!resetBase()) return false;
    return true;
  }
  // align cube
  if (run_no == 7) {
    if (!alignCube()) return false;
    return true;
  }

  while (1) {
    // read colors front
    if (run_no == 8) {
      String crrColor = "na";

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

      speed = 1.0;

      if (!cmdMoveGripperPer(G_WIDE_OPEN)) break;
      return true;
    }

    // TODO add rotate cube but lift the cube at Y_ROTATE_BASE such the sensor tube does not touch
  }

  speed = 1.0;

  // fix base
  if (run_no == 9) {
    if (!alignCube()) return false;
    return true;
  }
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
//                      COMMAND TABLE
// ------------------ -------------------------------------------------

struct CommandEntry {
  const char *name;
  const char *fmt;
  bool (*handler)(int argc, double *argv);
  const char *desc;
};

static CommandEntry command_table[] = {
  { "VERBOSEON", "", cmd_verbose_on, "VERBOSEON - enable verbose output" },
  { "VERBOSEOFF", "", cmd_verbose_off, "VERBOSEOFF - disable verbose output" },

  // TODO need an external FRAM RB does not have EEPRO

  { "SETMIN", "%d %d", cmd_set_min, "SETMIN <id> <ticks> - set the min ticks" },
  { "SETMAX", "%d %d", cmd_set_max, "SETMAX <id> <ticks> - set the max ticks" },
  // { "SETZERO", "%d %d", cmd_set_zero, "SETZERO <id> <ticks> - set the zero pos in ticks" },
  // { "SETDIRPLUS", "%d", cmd_set_dir_plus, "SETDIRPLUS <id> - set dir using the current pos > zero" },
  // { "SETDIRMINUS", "%d", cmd_set_dir_minus, "SETDIRMINUS <id> - set dir using the current pos < zero" },

  { "MOVETICKS", "%d %d", cmd_move_ticks, "MOVEDEG <id> <ticks goal> - move one servo to ticks (not smooth)" },
  { "MOVEDEG", "%d %f", cmd_move_deg, "MOVEDEG <id> <deg goal> - move one servo to degree (smooth)" },
  { "MOVEPER", "%d %f", cmd_move_per, "MOVEPER <id> <per goal> - move one servo to percent (smooth)" },
  // { "MOVECENTER", "", cmd_move_center, "MOVECENTER - move all to center" },
  { "MOVEYMM", "%f", cmd_move_y, "MOVEYMM <float mm> - vertical move (42 to 102)" },
  { "MOVEXMM", "%f", cmd_move_x, "MOVEXMM <float mm> - lateral move (-30 to 30)" },
  { "MOVEXYMM", "%f %f", cmd_move_xy, "MOVEXYMM <float mm> <float mm> - lateral then vertical move (-25 to 25)(42 to 102)" },
  { "MOVEGRIPPER", "%f", cmd_move_gripper, "MOVEGRIPPER <percentage> - move both grips to percentage (0 to 100)" },
  { "MOVEWRISTVERTDEG", "%f", cmd_move_wrist_vert, "MOVEWRISTVERTDEG <deg> - move wrist relative to vertical (-5 to 185)" },

  { "RUN", "%d", cmd_run, "RUN <no> - 0 center | 1 left down  | 2 right down\n                      | 3 base right | 4 base left | 5 base back\n                      | 6 reset base | 7 align cube | 8 read colors\n" },

  { "READ", "%d", cmd_read, "READ <id> - show servo summary status" },
  { "INFO", "%d", cmd_info, "INFO <id> - show servo full status" },

  { "COLOR", "%d", cmd_color, "COLOR <count> - read color <count> times" },

  { "LEDON", "%d", cmd_ledon, "LEDON <id> - turn servo LED on" },
  { "LEDOFF", "%d", cmd_ledoff, "LEDOFF <id> - turn servo LED off" },

  { "HELP", "", cmd_help, "HELP list of commands" },
};

static constexpr int COMMAND_COUNT = sizeof(command_table) / sizeof(command_table[0]);

// -------------------------------------------------------------------
//                        GET HELP TEXT
// -------------------------------------------------------------------

String get_help_text() {
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
      serial_printf("%s START ", cmd.name);
      print_all_status();
      bool ok = cmd.handler(argc, argv);
      serial_printf("%s END completed=%d ", cmd.name, ok ? 1 : 0);
      print_all_status();
      return;
    }
  }

  Serial.println("ERR Unknown command\n");
  Serial.println();
  Serial.println(get_help_text());
  Serial.println();
}
