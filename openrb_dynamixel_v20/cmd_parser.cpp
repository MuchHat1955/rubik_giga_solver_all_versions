#include "cmd_parser.h"

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

// For MOVE: we must read from last_raw_tokens passed by dispatcher
static String last_raw_tokens[8];

bool cmd_move(int argc, double *argv) {
  int id = (int)argv[0];
  const char *valStr = last_raw_tokens[1].c_str();

  if (!dxl.ping(id)) return false;

  int current = dxl.getPresentPosition(id);
  double goal = 0;

  if (valStr[0] == '+' || valStr[0] == '-') {
    goal = current + atof(valStr);  // relative move
    serial_printf("Relative move: start=%d rel=%d goal=%d\n", current, atoi(valStr), goal);
  } else {
    goal = atof(valStr);  // absolute move
  }
  if (goal < -95.0 || goal > 95.0) {
    serial_printf("Invalid servo deg: %.2f expected range (-90.0 to 90.0)\n", goal);
    return false;
  }

  if (!cmdMoveSmoothServo((uint8_t)id, goal)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_movecenter(int argc, double *argv) {
  for (int i = 0; i < SERVO_COUNT; i++)
    dxl.setGoalPosition(all_servos[i]->get_id(), 2048);
  return true;
}

bool cmd_movey(int argc, double *argv) {
  return cmdMoveYmm(argv[0]);
}
bool cmd_movex(int argc, double *argv) {
  return cmdMoveXmm(argv[0]);
}

bool cmd_read(int argc, double *argv) {
  print_servo_status((argc > 0) ? (int)argv[0] : 0);
  return true;
}

bool cmd_info(int argc, double *argv) {
  cmdInfo((uint8_t)argv[0]);
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
void cmdInfo(uint8_t id) {
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
  { "MOVEDEG", "%id %s", cmd_move, "MOVEDEG <id|name> <absgoal|±rel> - move one servo to degree" },
  { "MOVECENTER", "", cmd_movecenter, "MOVECENTER - move all to center" },
  { "MOVEYMM", "%f", cmd_movey, "MOVEYMM <float mm> - vertical move" },
  { "MOVEXMM", "%f", cmd_movex, "MOVEXMM <float mm> - lateral move" },
  { "READ", "%id", cmd_read, "READ [id|name] - show servo summary status" },
  { "INFO", "%id", cmd_info, "INFO <id|name> - show servo full status" },
  { "LEDON", "%id", cmd_ledon, "LEDON <id|name> - turn servo LED on" },
  { "LEDOFF", "%id", cmd_ledoff, "LEDOFF <id|name> - turn servo LED off" },
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
  auto derive_format_info = [](const char *fmt, int &min_args, bool &first_is_id) {
    min_args = 0;
    first_is_id = false;
    for (const char *p = fmt; *p; p++) {
      if (*p == '%') {
        min_args++;
        if (*(p + 1) == 'i' && *(p + 2) == 'd') first_is_id = true;
      }
    }
  };

  for (int i = 0; i < COMMAND_COUNT; i++) {
    const CommandEntry &cmd = command_table[i];
    if (U.startsWith(cmd.name)) {
      serial_printf(
        "---- START %s params: %s ----\n", cmd.name, line.c_str());

      int min_args = 0;
      bool first_is_id = false;
      derive_format_info(cmd.fmt, min_args, first_is_id);

      double argv[8] = { 0 };
      int argc = 0;
      String raw[8];
      argc = parse_args(line, cmd.fmt, argv, 8, raw);

      // save for MOVE relative parsing
      for (int j = 0; j < 8; j++) last_raw_tokens[j] = raw[j];

      // ---------------- Argument validation ----------------
      if (argc < min_args) {
        serial_printf(
          "Usage: %s\n", cmd.desc);
        serial_printf(
          "---- END %s ERR ----\n\n", cmd.name);
        return;
      }

      // ---------------- ID or Name Resolution ----------------
      if (first_is_id && argc > 0) {
        int id = resolve_id_or_name(raw[0]);
        if ((id < ID_ARM1 || id > ID_BASE) && id != 0) {
          serial_printf(
            "Invalid servo name/id: %s\n", raw[0].c_str());
          serial_printf(
            "---- END %s ERR ----\n\n", cmd.name);
          return;
        }
        argv[0] = id;
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
