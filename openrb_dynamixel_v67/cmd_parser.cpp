#include "cmd_parser.h"
#include "servo_move.h"
#include "servos.h"
#include "color_sensor.h"
#include "ori.h"
#include "color_reader.h"
#include "color_analyzer.h"
#include "run.h"

extern double max_xmm;
extern double max_ymm;
extern double min_ymm;
extern double speed;
extern double max_speed;
extern String version_str;

extern CubeOri ori;
extern CubeColorReader color_reader;
extern ColorAnalyzer color_analyzer;

extern const char runHelp[];

struct CommandEntry {
  const char *name;
  const char *fmt;
  bool (*handler)(int argc, double *argv);
  const char *desc;
};

static CommandEntry command_table[] = {

  // version
  // { "", "", nullptr, "------------------ HELP / VERSION ------------------" },
  { "HELP", "", cmd_help, "HELP - list all available commands" },
  { "VERSION", "", cmd_get_version, "VERSION - show firmware version info" },

  // main robot commands for cube solving
  // { "", "", nullptr, "-------------- MAIN CUBE SOLVING COMMANDS ----------" },
  { "MOVEROBOT", "<moves>", nullptr, "MOVEROBOT <moves> - robot moves (z_plus z_minus z_180 y_plus y_minus y_180 d_plus d_minus d_180)" },
  { "MOVECUBE", "<moves>", nullptr, "MOVECUBE <moves> - cube moves (f+ f- f2 b+ b- b2 r+ r- r2 l+ l- l2 u+ u- u2 d+ d- d2)" },
  { "READCOLORS", "<mode>", nullptr, "READCOLORS <all|bottom|solved> - read cube colors (u=white f=green r=red)" },

  // debug for robot moves
  // { "", "", nullptr, "---------------- DEBUG : ROBOT MOVES ----------------" },
  { "RUN", "%d", cmd_run, runHelp },

  // servos data
  // { "", "", nullptr, "------------------ SERVOS : DATA --------------------" },
  { "READSERVO", "%d", cmd_read_servo, "READSERVO <id> - show servo summary status" },
  { "INFOSERVO", "%d", cmd_servo_info, "INFOSERVO <id> - show full servo status" },
  { "SETMIN", "%d %d", cmd_set_servo_min, "SETMIN <id> <ticks> - set servo minimum ticks" },
  { "SETMAX", "%d %d", cmd_set_servo_max, "SETMAX <id> <ticks> - set servo maximum ticks" },

  // servo leds
  // { "", "", nullptr, "------------------ SERVOS : LEDS --------------------" },
  { "LEDON", "%d", cmd_ledon, "LEDON <id> - turn servo LED on" },
  { "LEDOFF", "%d", cmd_ledoff, "LEDOFF <id> - turn servo LED off" },

  // global servo error clearing
  // { "", "", nullptr, "---------------- SERVOS : ERRORS --------------------" },
  { "REBOOTALL", "", cmd_reboot_servos, "REBOOTALL - reboot all servos" },
  { "SETSTOPALL", "", cmd_set_servo_flag_servos_stop_all, "SETSTOPALL - set global servo error flag" },
  { "CLEARSTOPALL", "", cmd_clear_flag_servos_stop_all, "CLEARSTOPALL - clear global servo error flag" },

  // move servos
  // { "", "", nullptr, "------------------ SERVOS : MOTION ------------------" },
  { "MOVETICKS", "%d %d", cmd_move_ticks, "MOVETICKS <id> <ticks> - move servo to ticks (no smoothing)" },
  { "MOVEDEG", "%d %f", cmd_move_deg, "MOVEDEG <id> <deg> - move servo to degrees (smooth)" },
  { "MOVEPER", "%d %f", cmd_move_per, "MOVEPER <id> <percent> - move servo to percentage (smooth)" },

  // xy arms
  // { "", "", nullptr, "------------------ XY ARMS ---------------------------" },
  { "MOVEYMM", "%f", cmd_move_y, "MOVEYMM <mm> - vertical move (42 to 102)" },
  { "MOVEXMM", "%f", cmd_move_x, "MOVEXMM <mm> - lateral move (-30 to 30)" },
  { "MOVEXYMM", "%f %f", cmd_move_xy, "MOVEXYMM <x_mm> <y_mm> - lateral then vertical move (-25..25, 42..102)" },

  // gripper
  // { "", "", nullptr, "------------------ GRIPPER ---------------------------" },
  { "MOVEGRIPPER", "%f", cmd_move_gripper, "MOVEGRIPPER <percent> - move both grippers (0 to 100)" },
  { "MOVEWRISTVERTDEG", "%f", cmd_move_wrist_vert, "MOVEWRISTVERTDEG <deg> - move wrist relative to vertical (-5 to 185)" },
  { "CLAMP", "", cmd_move_clamp, "CLAMP - clamp gripper" },

  // color sensor
  // { "", "", nullptr, "------------------ COLOR SENSOR ----------------------" },
  { "COLORSENSOR", "%d", cmd_color, "COLORSENSOR <count> - read color <count> times" },
  { "ONECOLOR", "", cmd_read_one_color, "ONECOLOR - read one slot (1..6)" },
  { "ONEFACECOLOR", "", cmd_read_one_face_colors, "ONEFACECOLOR - read colors of the front face" },
  { "GETCOLORDATA", "", cmd_getcolor_data, "GETCOLORDATA - print raw color data" },

  // orientation
  // { "", "", nullptr, "------------------ ORIENTATION -----------------------" },
  { "GETORIDATA", "", cmd_getori_data, "GETORIDATA - print orientation move log" },
  { "CLEARORIDATA", "", cmd_clear_ori_data, "CLEARORIDATA - clear orientation data" },
  { "RESTOREORI", "", cmd_restore_ori, "RESTOREORI - restore cube to original orientation" },
};


static constexpr int COMMAND_COUNT = sizeof(command_table) / sizeof(command_table[0]);

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~-
//                            PARSE HELPERS
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~-

static int parse_args(const String &line, const char *fmt, double *out, int max_args, String *raw_tokens = nullptr) {  // Skip command word
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

bool cmd_get_version(int argc, double *argv) {
  Serial.println();
  Serial.println(version_str);
  Serial.println();
  return true;
}

bool cmd_help(int argc, double *argv) {
  Serial.println();
  Serial.println(get_help_text());
  return true;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~-
//                        GET HELP TEXT
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~-

String get_help_text() {
  String help = "\nCOMMANDS:\n";
  for (int i = 0; i < COMMAND_COUNT; i++) {
    help += "  ";
    if (command_table[i].name == "") help += "\n";
    help += command_table[i].desc;
    help += "\n";
  }
  return help;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~-
//                    PROCESS SERIAL COMMAND (DISPATCHER)
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~-

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

  max_speed = 1.0;
  speed = 1.0;

  for (int i = 0; i < COMMAND_COUNT; i++) {
    if (command_table[i].name == "") continue;
    const CommandEntry &cmd = command_table[i];

    if (U.startsWith(cmd.name)) {

      unsigned long millis_start = millis();
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~--
      // Special handling for MOVEROBOT and MOVECUBE (string commands)
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~--
      if (strcmp(cmd.name, "MOVEROBOT") == 0 || strcmp(cmd.name, "MOVECUBE") == 0) {

        // Extract raw params after command
        int space_idx = line.indexOf(' ');
        if (space_idx < 0) {
          //
          LOG_ERR(MOD_CMD, "error", "missing argument");
          LOG_VAR("command", cmd.name);
          LOG_VAR("result", "fail");
          return;
        }
        String params = line.substring(space_idx + 1);
        params.trim();
        if (params.length() == 0) {
          //
          LOG_ERR(MOD_CMD, "error", "missing argument");
          LOG_VAR("command", cmd.name);
          return;
        }

        String params_ = "\"" + params + "\"";
        LOG_INFO(MOD_CMD, "command_start", cmd.name);
        LOG_VAR("params", params_.c_str());

        bool ok = false;

        if (check_servos_stop_all()) {
          LOG_ERR(MOD_CMD, "error", "servos_stop_flag_on");
          LOG_VAR("command", cmd.name);
          return;
        }

        if (strcmp(cmd.name, "MOVECUBE") == 0) {
          // pass full raw string — ori parses sequence itself

          LOG_INFO(MOD_CMD, "calling_cube_move", params_.c_str());
          ok = ori.cube_move(params);
        } else if (strcmp(cmd.name, "MOVEROBOT") == 0) {
          // pass full raw string — NO splitting
          LOG_INFO(MOD_CMD, "calling_robot_move", params_.c_str());
          ok = ori.robot_move(params);
        }

        LOG_INFO(MOD_CMD, "command_end", cmd.name);
        LOG_VAR("result", ok ? "ok" : "fail");
        unsigned long duration_ms = millis() - millis_start;
        String duration_str = "";
        if (duration_ms <= 999) duration_str = String(duration_ms) + "ms";
        else {
          duration_str = String(duration_ms / 1000UL) + "s";
          duration_str += String(duration_ms % 1000UL) + "ms";
        }
        LOG_VAR("duration", duration_str.c_str());
        return;
      }
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~--
      // Special handling for READCOLORS (string command)
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~--
      if (strcmp(cmd.name, "READCOLORS") == 0) {

        // Extract raw params after command
        int space_idx = line.indexOf(' ');
        if (space_idx < 0) {
          LOG_ERR(MOD_CMD, "error", "missing argument");
          LOG_VAR("command", cmd.name);
          return;
        }
        String params = line.substring(space_idx + 1);
        params.trim();
        if (params.length() == 0) {
          LOG_ERR(MOD_CMD, "error", "missing argument");
          LOG_VAR("command", cmd.name);
          return;
        }

        String params_ = "\"" + params + "\"";
        LOG_INFO(MOD_CMD, "command_start", cmd.name);
        LOG_VAR("params", params_.c_str());

        // Unified logging (same style as MOVEROBOT)
        // serial_printf_verbose("~~~- START READCOLORS params: %s ~~~-", params.c_str());
        params_ = "\"" + params + "\"";
        LOG_INFO(MOD_CMD, "readcolors_start_mode", params_.c_str());

        bool ok = cmd_read_cube_colors(params);
        LOG_INFO(MOD_CMD, "command_end", cmd.name);
        LOG_VAR("result", ok ? "ok" : "fail");
        unsigned long duration_ms = millis() - millis_start;
        String duration_str = "";
        if (duration_ms <= 999) duration_str = String(duration_ms) + "ms";
        else {
          duration_str = String(duration_ms / 1000UL) + "s";
          duration_str += String(duration_ms % 1000UL) + "ms";
        }
        LOG_VAR("duration", duration_str.c_str());

        return;
      }


      // ~~~~~~~~~~~~~~~- Existing numeric-command path ~~~~~~~~~~~~~~~-
      // serial_printf_verbose(
      //  "~~~- START %s params: %s ~~~-", cmd.name, line.c_str());

      int min_args = 0;
      derive_format_info(cmd.fmt, min_args);

      double argv[8] = { 0 };
      int argc = 0;
      String raw[8];
      argc = parse_args(line, cmd.fmt, argv, 8, raw);

      // ~~~~~~~~~~~~~~~- Argument validation ~~~~~~~~~~~~~~~-
      if (argc < min_args) {

        LOG_ERR(MOD_CMD, "invalid_command_usage", cmd.name);
        LOG_VAR("usage", cmd.desc);
        return;
      }

      // ~~~~~~~~~~~~~~~- Execute Command ~~~~~~~~~~~~~~~-
      double p1 = 0;
      if (argc > 0) p1 = (double)argv[0];

      //
      increment_cmd_no();
      LOG_LN();
      LOG_INFO(MOD_CMD, "command_start", cmd.name);
      LOG_VAR("arg", p1);

      // read_print_kinematics_state();
      bool ok = cmd.handler(argc, argv);
      //
      LOG_INFO(MOD_CMD, "command_end", cmd.name);
      LOG_VAR("result", ok ? "ok" : "fail");
      unsigned long duration_ms = millis() - millis_start;
      String duration_str = "";
      if (duration_ms <= 999) duration_str = String(duration_ms) + "ms";
      else {
        duration_str = String(duration_ms / 1000UL) + "s";
        duration_str += String(duration_ms % 1000UL) + "ms";
      }
      LOG_VAR("duration", duration_str.c_str());

      // read_print_kinematics_state();
      return;
    }
  }
  //
  LOG_ERR(MOD_CMD, "error","unknown_command");  //
}
