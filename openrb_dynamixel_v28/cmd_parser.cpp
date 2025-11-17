#include "cmd_parser.h"
#include "movement.h"
#include "servos.h"

void print_info(uint8_t id);

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
  if (goal_xmm < -30.0 || goal_xmm > 30.0) {
    serial_printf("ERR invalid x mm: %.2f expected range (-30 mm to 30 mm)\n", goal_xmm);
    return false;
  }
  double goal_ymm = argv[1];
  if (goal_ymm < 35.0 || goal_ymm > 105.0) {
    serial_printf("ERR invalid y mm: %.2f expected range (35.0 mm to 105 mm)\n", goal_ymm);
    return false;
  }
  if (!cmdMoveXmm(goal_xmm)) return false;
  if (!cmdMoveYmm(goal_ymm)) return false;
  return true;
}

bool cmd_pos_zero(int argc, double *argv) {
  if (!cmdMoveGripperPer(0.0)) return false;
  if (!cmdMoveWristDegVertical(0.0)) return false;
  if (!cmdMoveXmm(0.0)) return false;
  if (!cmdMoveYmm(35.5)) return false;
  return true;
}

bool cmd_move_deg(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  int current = dxl.getPresentPosition(id);
  int goal_deg = (int)argv[1];

  if (goal_deg < -95.0 || goal_deg > 185.0) {
    serial_printf("ERR invalid servo deg: %.2f expected range (-95.0 deg to 185.0 deg)\n", goal_deg);
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

  if (!dxl.setGoalPosition(id, goal_ticks)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_move_per(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  double goal_per = argv[1];
  if (goal_per < 5.0 || goal_per > 105.0) {
    serial_printf("ERR invalid servo percentage: %.2f expected range (-5.0 per to 105.0 per)\n", goal_per);
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

bool cmd_set_zero(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  int t = (int)argv[1];

  if (auto *s = find_servo(id)) {
    serial_printf_verbose("cmd_set_zero: id=%d ticks=%d\n", id, t);
    s->set_zero_ticks(t);
    return true;
  }
  return false;
}

bool cmd_set_dir_plus(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  int t = (int)argv[1];

  if (auto *s = find_servo(id)) {
    int curr = dxl.getPresentPosition(id);
    int zero = s->zero_ticks();
    if (abs(curr - zero) < 100) {
      serial_printf("ERR move servo at least 100 ticks from zero to set dir: zero=%d curr=%d\n", zero, curr);
      return false;
    }
    double dir = 1.0;
    if (curr < zero) dir = -1.0;
    serial_printf("ERR cmd_set_dir_plus: id=%d zero=%d curr=%d dir=%.2f\n", id, zero, curr, dir);
    s->set_dir(dir);
    return true;
  }
  return false;
}

bool cmd_set_dir_minus(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  int t = (int)argv[1];

  if (auto *s = find_servo(id)) {
    int curr = dxl.getPresentPosition(id);
    int zero = s->zero_ticks();
    if (abs(curr - zero) < 100) {
      serial_printf("ERR move servo at least 100 ticks from zero to set dir: zero=%d curr=%d\n", zero, curr);
      return false;
    }
    double dir = 1.0;
    if (curr > zero) dir = -1.0;
    serial_printf("ERR cmd_set_dir_plus: id=%d zero=%d curr=%d dir=%.2f\n", id, zero, curr, dir);
    s->set_dir(dir);
    return true;
  }
  return false;
}

bool cmd_move_center(int argc, double *argv) {
  for (int i = 0; i < SERVO_COUNT; i++)
    dxl.setGoalPosition(all_servos[i]->get_id(), 2048);
  return true;
}

bool cmd_move_y(int argc, double *argv) {
  double goal_mm = argv[0];
  if (goal_mm < 35.0 || goal_mm > 105.0) {
    serial_printf("ERR invalid y mm: %.2f expected range (35.0 mm to 105 mm)\n", goal_mm);
    return false;
  }

  serial_printf_verbose("cmd_move_y: y=%.2fmm\n", goal_mm);
  if (!cmdMoveYmm(goal_mm)) return false;
  return true;
}

bool cmd_move_x(int argc, double *argv) {
  double goal_mm = argv[0];
  if (goal_mm < -30.0 || goal_mm > 30.0) {
    serial_printf("ERR invalid x mm: %.2f expected range (-30 mm to 30 mm)\n", goal_mm);
    return false;
  }

  serial_printf_verbose("cmd_move_x: x=%.2fmm\n", goal_mm);
  if (!cmdMoveXmm(goal_mm)) return false;
  return true;
}

bool cmd_move_gripper(int argc, double *argv) {
  if (!dxl.ping(ID_GRIP1) || !dxl.ping(ID_GRIP2)) return false;

  double goal_deg = argv[0];

  if (goal_deg < -5.0 || goal_deg > 105.0) {
    serial_printf("ERR invalid gripper percentage: %.2f expected range (-5.0 per to 105 per)\n", goal_deg);
    return false;
  }

  serial_printf_verbose("cmd_move_gripper: deg=%.2\n", goal_deg);
  if (!cmdMoveGripperPer(goal_deg)) return false;
  print_servo_status(ID_GRIP1);
  print_servo_status(ID_GRIP2);
  return true;
}

bool cmd_move_wrist_vert(int argc, double *argv) {
  if (!dxl.ping(ID_WRIST) || !dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2)) return false;

  double goal_deg = argv[0];

  if (goal_deg < -5 || goal_deg > 95) {
    serial_printf("ERR invalid wrist percentage: %.2f expected range (-5% to 95%)\n", goal_deg);
    return false;
  }

  serial_printf_verbose("cmd_move_wrist: deg=%.2\n", goal_deg);
  if (!cmdMoveWristDegVertical(goal_deg)) return false;
  print_servo_status(ID_WRIST);
  return true;
}

bool cmd_help(int argc, double *argv) {
  Serial.println();
  Serial.println(get_help_text());
  Serial.println();
  return true;
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
    serial_printf_verbose("Servo %d not found\n", id);
    return;
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

  // { "SETMIN", "%d %d", cmd_set_min, "SETMIN <id> <ticks> - set the min as ticks" },
  // { "SETMAX", "%d %d", cmd_set_max, "SETMAX <id> <ticks> - set the max as ticks" },
  // { "SETZERO", "%d %d", cmd_set_zero, "SETZERO <id> <ticks> - set the zero pos in ticks" },
  // { "SETDIRPLUS", "%d", cmd_set_dir_plus, "SETDIRPLUS <id> - set dir using the current pos > zero" },
  // { "SETDIRMINUS", "%d", cmd_set_dir_minus, "SETDIRMINUS <id> - set dir using the current pos < zero" },

  { "POSZERO", "", cmd_pos_zero, "POSZERO - move to pos zero" },
  { "MOVETICKS", "%d %d", cmd_move_ticks, "MOVEDEG <id> <ticks goal> - move one servo to ticks (not smooth)" },
  { "MOVEDEG", "%d %f", cmd_move_deg, "MOVEDEG <id> <deg goal> - move one servo to degree (smooth)" },
  { "MOVEPER", "%d %f", cmd_move_per, "MOVEPER <id> <per goal> - move one servo to percent (smooth)" },
  { "MOVECENTER", "", cmd_move_center, "MOVECENTER - move all to center" },
  { "MOVEYMM", "%f", cmd_move_y, "MOVEYMM <float mm> - vertical move" },
  { "MOVEXMM", "%f", cmd_move_x, "MOVEXMM <float mm> - lateral move" },
  { "MOVEXYMM", "%f %f", cmd_move_xy, "MOVEXYMM <float mm> <float mm> - lateral then vertical move" },
  { "MOVEGRIPPER", "%f", cmd_move_gripper, "MOVEGRIPPER <percentage> - move both grips to percentage" },
  { "MOVEWRISTVERTDEG", "%f", cmd_move_wrist_vert, "MOVEWRISTVERTDEG <deg> - move wrist relative to vertical" },

  { "READ", "%d", cmd_read, "READ <id> - show servo summary status" },
  { "INFO", "%d", cmd_info, "INFO <id> - show servo full status" },

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
