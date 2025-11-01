#include "cmd_parser.h"

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

  serial_printf(
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

bool cmd_move_deg(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  int current = dxl.getPresentPosition(id);
  int goal_deg = (int)argv[1];

  if (goal_deg < -95.0 || goal_deg > 95.0) {
    serial_printf("Invalid servo deg: %.2f expected range (-90.0 deg to 90.0 deg)\n", goal_deg);
    return false;
  }
  int goal_ticks = per2ticks(id, goal_deg);
  serial_printf("cmd_move_deg: id=%d deg=%d\n", id, goal_deg);

  if (!cmdMoveServoDeg((uint8_t)id, goal_deg)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_move_ticks(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  int goal_ticks = (int)argv[1];
  serial_printf("cmd_move_ticks: id=%d ticks=%d\n", id, goal_ticks);

  if (!dxl.setGoalPosition(id, goal_ticks)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_move_per(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  double goal_per = argv[1];
  if (goal_per < 5.0 || goal_per > 105.0) {
    serial_printf("Invalid servo percentage: %.2f expected range (-5.0 deg to 95.0 deg)\n", goal_per);
    return false;
  }

  double goal_deg = per2deg(id, goal_per);
  serial_printf("cmd_move_per: id=%d per=%d deg=%d\n", id, goal_per, goal_deg);

  if (!cmdMoveServoDeg((uint8_t)id, goal_deg)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_set_min(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  int t = (int)argv[1];

  if (auto *s = find_servo(id)) {
    serial_printf("cmd_set_min: id=%d ticks=%d\n", id, t);
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
    serial_printf("cmd_set_max: id=%d ticks=%d\n", id, t);
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
    serial_printf("cmd_set_zero: id=%d ticks=%d\n", id, t);
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
      serial_printf("Move servo at least 100 ticks from zero to set dir: zero=%d curr=%d\n", zero, curr);
      return false;
    }
    double dir = 1.0;
    if (curr < zero) dir = -1.0;
    serial_printf("cmd_set_dir_plus: id=%d zero=%d curr=%d dir=%.2f\n", id, zero, curr, dir);
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
      serial_printf("Move servo at least 100 ticks from zero to set dir: zero=%d curr=%d\n", zero, curr);
      return false;
    }
    double dir = 1.0;
    if (curr > zero) dir = -1.0;
    serial_printf("cmd_set_dir_plus: id=%d zero=%d curr=%d dir=%.2f\n", id, zero, curr, dir);
    s->set_dir(dir);
    return true;
  }
  return false;
}

bool cmd_extend_min(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  uint16_t t = (int)argv[1];
  uint16_t hw_min = dxl.readControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id);

  if (auto *s = find_servo(id)) {
    uint16_t hw_min = s->min_ticks();
    if (hw_min < t) hw_min = t;
    uint16_t tf = hw_min - t;
    serial_printf("cmd_extend_min: id=%d from=%d to ticks=%d\n", id, hw_min, tf);
    s->set_max_ticks(tf);
    return true;
  }
  return false;
}

bool cmd_extend_max(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  uint16_t t = (int)argv[1];

  if (auto *s = find_servo(id)) {
    uint16_t hw_max = s->max_ticks();
    uint16_t tf = hw_max + t;
    serial_printf("cmd_extend_max: id=%d from=%d to ticks=%d\n", id, hw_max, tf);
    s->set_max_ticks(tf);
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
  double goal_mm = argv[1];
  if (goal_mm < 40.0 || goal_mm > 110.0) {
    serial_printf("Invalid y mm: %.2f expected range (40 mm to 110 mm)\n", goal_mm);
    return false;
  }

  serial_printf("cmd_move_y: y=%.2fmm\n", goal_mm);
  return cmdMoveYmm(argv[0]);
}

bool cmd_move_x(int argc, double *argv) {
  double goal_mm = argv[1];
  if (goal_mm < -40.0 || goal_mm > 40.0) {
    serial_printf("Invalid x mm: %.2f expected range (-40 mm to 40 mm)\n", goal_mm);
    return false;
  }

  serial_printf("cmd_move_x: x=%.2fmm\n", goal_mm);
  return cmdMoveXmm(argv[0]);
}

bool cmd_move_gripper(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  double goal_deg = argv[1];

  if (goal_deg < 5.0 || goal_deg > 105.0) {
    serial_printf("Invalid gripper percentage: %.2f expected range (-5.0 deg to 95.0 deg)\n", goal_deg);
    return false;
  }

  serial_printf("cmd_move_gripper: deg=%.2\n", goal_deg);
  if (!cmdMoveGripperPer(goal_deg)) return false;
  print_servo_status(id);
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
    serial_printf("Servo %d not found\n", id);
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

  serial_printf("INFO id=%d\n", id);
  serial_printf("  OperatingMode : %d\n", op);
  serial_printf("  DriveMode     : %d (bit0=%s-profile)\n", drv, (drv & 0x01) ? "TIME" : "VELOCITY");
  serial_printf("  Profile Vel   : %d  (≈ %.3f rpm, %.1f ticks/s)\n", pv, rpm, tps);
  serial_printf("  Profile Accel : %d\n", pa);
  serial_printf("  PosLimits     : min=%d  max=%d  (span≈ %.1f°)\n", minL, maxL, spanDeg);
  serial_printf("  Present Pos   : %d (≈ %.2f°)\n", pos, pos * DEG_PER_TICK);
}

// -------------------------------------------------------------------
//                      COMMAND TABLE
// -------------------------------------------------------------------

struct CommandEntry {
  const char *name;
  const char *fmt;
  bool (*handler)(int argc, double *argv);
  const char *desc;
};

static CommandEntry command_table[] = {
  { "VERBOSEON", "", cmd_verbose_on, "VERBOSEON - enable verbose output" },
  { "VERBOSEOFF", "", cmd_verbose_off, "VERBOSEOFF - disable verbose output" },

  { "SETMIN", "%d", cmd_set_min, "SETMIN <id> - set current pos as min" },
  { "SETMAX", "%d", cmd_set_max, "SETMAX <id> - set current pos as max" },
  { "EXTENDMIN", "%d", cmd_extend_min, "SETLMIN <id> <ticks> - extend min with <ticks>" },
  { "EXTENDMAX", "%d", cmd_extend_max, "EXTENDMAX <id> <ticks> - extend max with <ticks>" },
  { "SETZERO", "%d", cmd_set_zero, "SETLIMITMAX <id> - set current pos as zero" },
  { "SETDIRPLUS", "%d", cmd_set_dir_plus, "SETDIRPLUS <id> - set dir using the current pos > zero" },
  { "SETDIRMINUS", "%d", cmd_set_dir_minus, "SETDIRMINUS <id> - set dir using the current pos < zero" },

  { "MOVETICKS", "%d %d", cmd_move_ticks, "MOVEDEG <id> <abs ticks goal> - move one servo to ticks (not smooth)" },
  { "MOVEDEG", "%d %f", cmd_move_deg, "MOVEDEG <id> <abs deg goal> - move one servo to degree (smooth)" },
  { "MOVEPER", "%d %f", cmd_move_per, "MOVEPER <id> <abs per goal> - move one servo to percent (smooth)" },
  { "MOVECENTER", "", cmd_move_center, "MOVECENTER - move all to center" },
  { "MOVEYMM", "%f", cmd_move_y, "MOVEYMM <float mm> - vertical move" },
  { "MOVEXMM", "%f", cmd_move_x, "MOVEXMM <float mm> - lateral move" },
  { "MOVEGRIPPER", "%f", cmd_move_gripper, "MOVEGRIPPER <percentage> - move both grips to percebtge" },

  { "READ", "%d", cmd_read, "READ [id|name] - show servo summary status" },
  { "INFO", "%id", cmd_info, "INFO <id> - show servo full status" },

  { "LEDON", "%d", cmd_ledon, "LEDON <id> - turn servo LED on" },
  { "LEDOFF", "%d", cmd_ledoff, "LEDOFF <id> - turn servo LED off" },
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
      serial_printf(
        "---- START %s params: %s ----\n", cmd.name, line.c_str());

      int min_args = 0;
      derive_format_info(cmd.fmt, min_args);

      double argv[8] = { 0 };
      int argc = 0;
      String raw[8];
      argc = parse_args(line, cmd.fmt, argv, 8, raw);

      // ---------------- Argument validation ----------------
      if (argc < min_args) {
        serial_printf(
          "Usage: %s\n", cmd.desc);
        serial_printf(
          "---- END %s ERR ----\n\n", cmd.name);
        return;
      }

      // ---------------- Execute Command ----------------
      bool ok = cmd.handler(argc, argv);
      serial_printf(
        "---- END %s %s ----\n\n", cmd.name, ok ? "OK" : "ERR");
      return;
    }
  }

  Serial.println("ERR: Unknown command\n");
}
