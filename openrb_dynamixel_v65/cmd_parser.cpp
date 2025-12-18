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
  { "SETMIN", "%d %d", cmd_set_min, "SETMIN <id> <ticks> - set the min ticks" },
  { "SETMAX", "%d %d", cmd_set_max, "SETMAX <id> <ticks> - set the max ticks" },

  { "RUN", "%d", cmd_run, runHelp },

  { "MOVETICKS", "%d %d", cmd_move_ticks, "MOVETICKS <id> <ticks goal> - move one servo to ticks (not smooth)" },
  { "MOVEDEG", "%d %f", cmd_move_deg, "MOVEDEG <id> <deg goal> - move one servo to degree (smooth)" },
  { "MOVEPER", "%d %f", cmd_move_per, "MOVEPER <id> <per goal> - move one servo to percent (smooth)" },

  { "MOVEYMM", "%f", cmd_move_y, "MOVEYMM <float mm> - vertical move (42 to 102)" },
  { "MOVEXMM", "%f", cmd_move_x, "MOVEXMM <float mm> - lateral move (-30 to 30)" },
  { "MOVEXYMM", "%f %f", cmd_move_xy, "MOVEXYMM <float mm> <float mm> - lateral then vertical move (-25 to 25)(42 to 102)" },
  { "MOVEGRIPPER", "%f", cmd_move_gripper, "MOVEGRIPPER <percentage> - move both grips to percentage (0 to 100)" },
  { "MOVEWRISTVERTDEG", "%f", cmd_move_wrist_vert, "MOVEWRISTVERTDEG <deg> - move wrist relative to vertical (-5 to 185)" },
  { "CLAMP", "", cmd_move_clamp, "CLAMP - clamp gripper" },

  { "READSERVO", "%d", cmd_read, "READSERVO <id> - show servo summary status" },
  { "INFOSERVO", "%d", cmd_info, "INFOSERVO <id> - show servo full status" },
  { "REBOOTSERVOS", "", cmd_reboot_servos, "REBOOTSERVOS - reboot servos and reset error flag" },
  { "SETSERVOSSTOPALL", "", cmd_set_flag_servos_stop_all, "SETSERVOSSTOPALL - reboot servos and reset error flag" },
  { "REBOOTSERVOSSTOPALL", "", cmd_reset_flag_servos_stop_all, "REBOOTSERVOSSTOPALL - reboot servos and reset error flag" },

  { "COLORSENSOR", "%d", cmd_color, "COLORSENSOR <count> - read color <count> times" },
  { "ONECOLOR", "", cmd_read_one_color, "ONECOLOR - read one slot 1...6" },
  { "ONEFACECOLOR", "", cmd_read_one_face_colors, "ONEFACECOLOR the colors of the face in front" },

  { "READCOLORS", "<mode>", nullptr, "READCOLORS <all | bottom | solved (u=White, f=Green r=red)> - read cube colors" },

  { "LEDON", "%d", cmd_ledon, "LEDON <id> - turn servo LED on" },
  { "LEDOFF", "%d", cmd_ledoff, "LEDOFF <id> - turn servo LED off" },
  // NEW: string-based move commands using CubeOri
  { "MOVEROBOT", "<moves>", nullptr, "MOVEROBOT <moves> - robot moves space-separated list (z_plus z_minus z_180 y_plus y_minus y_180 d_plus d_minus d_180)" },
  { "MOVECUBE", "<moves>", nullptr, "MOVECUBE <moves> - cube moves space-separated list (f+ f- f2 b+ b- b2 r+ r- r2 l+ l- l2 u+ u- u2 d+ d- d2)" },

  { "GETORIDATA", "", cmd_getori_data, "GETORIDATA - print orientation move log" },
  { "CLEARORIDATA", "", cmd_clear_ori_data, "CLEARORIDATA - reset orientation data" },
  { "RESTOREORI", "", cmd_restore_ori, "RESTOREORI - move cube back in the original orientation" },
  { "GETCOLORDATA", "", cmd_getcolor_data, "GETCOLORDATA - print color data" },

  { "HELP", "", cmd_help, "HELP list of commands" },
};

static constexpr int COMMAND_COUNT = sizeof(command_table) / sizeof(command_table[0]);

// -------------------------------------------------------------------
//                            PARSE HELPERS
// -------------------------------------------------------------------

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

bool cmd_help(int argc, double *argv) {
  Serial.println();
  Serial.println(get_help_text());
  Serial.println();
  return true;
}

// -------------------------------------------------------------------
//                        GET HELP TEXT
// -------------------------------------------------------------------

String get_help_text() {
  String help = "Supported commands:";
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

      unsigned long millis_start = millis();
      // -----------------------------------------------------------
      // Special handling for MOVEROBOT and MOVECUBE (string commands)
      // -----------------------------------------------------------
      if (strcmp(cmd.name, "MOVEROBOT") == 0 || strcmp(cmd.name, "MOVECUBE") == 0) {

        // Extract raw params after command
        int space_idx = line.indexOf(' ');
        if (space_idx < 0) {
          //
          LOG_ERR(MOD_CMD, "missing argument");
          LOG_VAR("command", cmd.name);
          LOG_INFO(MOD_CMD, "command_end");
          LOG_VAR("command", cmd.name);
          LOG_VAR("result", "fail");
          return;
        }
        String params = line.substring(space_idx + 1);
        params.trim();
        if (params.length() == 0) {
          //
          LOG_ERR(MOD_CMD, "missing argument");
          LOG_VAR("command", cmd.name);
          LOG_INFO(MOD_CMD, "command_end");
          LOG_VAR("command", cmd.name);
          LOG_VAR("result", "fail");
          return;
        }

        String params_ = "\"" + params + "\"";
        LOG_INFO(MOD_CMD, "command_start");
        LOG_VAR("command", cmd.name);
        LOG_VAR("params", params_.c_str());

        bool ok = false;

        if (check_servos_stop_all()) {
          LOG_ERR(MOD_CMD, "servos_stop_flag_on");
          LOG_INFO(MOD_CMD, "command_end");
          LOG_VAR("command", cmd.name);
          LOG_VAR("result", ok ? "ok" : "fail");
          return;
        }

        if (strcmp(cmd.name, "MOVECUBE") == 0) {
          // pass full raw string — ori parses sequence itself
          //
          LOG_INFO(MOD_CMD, "calling_cube_move");
          LOG_VAR("moves", params_.c_str());

          ok = ori.cube_move(params);
        } else if (strcmp(cmd.name, "MOVEROBOT") == 0) {
          // pass full raw string — NO splitting
          //
          LOG_INFO(MOD_CMD, "calling_robot_move");
          LOG_VAR("moves", params_.c_str());

          ok = ori.robot_move(params);
        }

        LOG_INFO(MOD_CMD, "command_end");
        LOG_VAR("command", cmd.name);
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
      // -----------------------------------------------------------
      // Special handling for READCOLORS (string command)
      // -----------------------------------------------------------
      if (strcmp(cmd.name, "READCOLORS") == 0) {

        // Extract raw params after command
        int space_idx = line.indexOf(' ');
        if (space_idx < 0) {
          //
          LOG_ERR(MOD_CMD, "missing argument");
          return;
        }
        String params = line.substring(space_idx + 1);
        params.trim();
        if (params.length() == 0) {
          //
          LOG_ERR(MOD_CMD, "missing argument");
          return;
        }

        String params_ = "\"" + params + "\"";
        LOG_INFO(MOD_CMD, "command_start");
        LOG_VAR("command", cmd.name);
        LOG_VAR("params", params_.c_str());

        // Unified logging (same style as MOVEROBOT)
        // serial_printf_verbose("---- START READCOLORS params: %s ----", params.c_str());
        params_ = "\"" + params + "\"";
        LOG_INFO(MOD_CMD, "readcolors start");
        LOG_VAR("mode", params_.c_str());

        bool ok = cmd_read_cube_colors_string(params);

        LOG_INFO(MOD_CMD, "readcolors end");
        LOG_VAR("mode", params_.c_str());
        LOG_VAR("result", ok ? "ok" : "fail");

        LOG_INFO(MOD_CMD, "command_end");
        LOG_VAR("command", cmd.name);
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


      // ---------------- Existing numeric-command path ----------------
      // serial_printf_verbose(
      //  "---- START %s params: %s ----", cmd.name, line.c_str());

      int min_args = 0;
      derive_format_info(cmd.fmt, min_args);

      double argv[8] = { 0 };
      int argc = 0;
      String raw[8];
      argc = parse_args(line, cmd.fmt, argv, 8, raw);

      // ---------------- Argument validation ----------------
      if (argc < min_args) {

        LOG_ERR(MOD_CMD, "invalid_command_usage");
        LOG_VAR("command", cmd.name);
        LOG_VAR("usage", cmd.desc);
        return;
      }

      // ---------------- Execute Command ----------------
      double p1 = 0;
      if (argc > 0) p1 = (double)argv[0];

      //
      increment_cmd_no();
      LOG_LN();
      LOG_INFO(MOD_CMD, "command_start");
      LOG_VAR("command", cmd.name);
      LOG_VAR("arg", p1);

      // read_print_kinematics_state();
      bool ok = cmd.handler(argc, argv);
      //
      LOG_INFO(MOD_CMD, "command_end");
      LOG_VAR("command", cmd.name);
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
  LOG_ERR(MOD_CMD, "unknown command");  //
  LOG_INFO(MOD_CMD, "help text");
  LOG_VAR("text", get_help_text().c_str());
}
