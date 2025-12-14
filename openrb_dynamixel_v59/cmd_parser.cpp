#include "cmd_parser.h"
#include "movement.h"
#include "servos.h"
#include "color_sensor.h"
#include "ori.h"
#include "color_reader.h"
#include "color_analyzer.h"
#include "utils.h"

void print_info(uint8_t id);

extern double max_xmm;
extern double max_ymm;
extern double min_ymm;
extern double speed;

extern CubeOri ori;
extern CubeColorReader color_reader;
extern ColorAnalyzer color_analyzer;

uint32_t start_ms = 0;

const char runHelp[] PROGMEM =
  "RUN <no>\n"
  "      0 pos zero |\n"
  "     11 right down   | 12 left down    | 13 back down   | 14 top down\n"
  "     21 bottom right | 22 bottom right | 23 bottom back\n"
  "     31 cube right   | 32 cube left    | 33 cube back\n"
  "     41 reset right  | 42 reset left   | 143 reset back\n"
  "     60 align";

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

  { "RUN", "%d", cmd_run, runHelp },

  { "MOVETICKS", "%d %d", cmd_move_ticks, "MOVETICKS <id> <ticks goal> - move one servo to ticks (not smooth)" },
  { "MOVEDEG", "%d %f", cmd_move_deg, "MOVEDEG <id> <deg goal> - move one servo to degree (smooth)" },
  { "MOVEPER", "%d %f", cmd_move_per, "MOVEPER <id> <per goal> - move one servo to percent (smooth)" },

  { "MOVEYMM", "%f", cmd_move_y, "MOVEYMM <float mm> - vertical move (42 to 102)" },
  { "MOVEXMM", "%f", cmd_move_x, "MOVEXMM <float mm> - lateral move (-30 to 30)" },
  { "MOVEXYMM", "%f %f", cmd_move_xy, "MOVEXYMM <float mm> <float mm> - lateral then vertical move (-25 to 25)(42 to 102)" },
  { "MOVEGRIPPER", "%f", cmd_move_gripper, "MOVEGRIPPER <percentage> - move both grips to percentage (0 to 100)" },
  { "MOVEWRISTVERTDEG", "%f", cmd_move_wrist_vert, "MOVEWRISTVERTDEG <deg> - move wrist relative to vertical (-5 to 185)" },
  { "GRIPPERCLAMP", "", cmd_move_clamp, "GRIPPERCLAMP - clamp gripper" },

  { "READSERVO", "%d", cmd_read, "READSERVO <id> - show servo summary status" },
  { "INFOSERVO", "%d", cmd_info, "INFOSERVO <id> - show servo full status" },

  { "COLORSENSOR", "%d", cmd_color, "COLORSENSOR <count> - read color <count> times" },
  { "SCAN1COLOR", "", cmd_read_one_color, "SCAN1COLOR - read one slot 1...6" },
  { "SCAN1FACE", "", cmd_read_one_face_colors, "SCAN1FACE the colors of the face in front" },

  { "COLORSCAN", "<mode>", nullptr, "COLORSCAN <all | bottom | solved (u=White, f=Green r=red)> - read cube colors" },

  { "LEDON", "%d", cmd_ledon, "LEDON <id> - turn servo LED on" },
  { "LEDOFF", "%d", cmd_ledoff, "LEDOFF <id> - turn servo LED off" },

  // NEW: string-based move commands using CubeOri
  { "MOVEROBOT", "<moves>", nullptr, "MOVEROBOT <moves> - robot moves space-separated list (y+ y- z+ z- z2 d+ d- d2)" },
  { "MOVECUBE", "<moves>", nullptr, "MOVECUBE <moves> - cube moves space-separated list (f+ f- f2 b+ b- b2 r+ r- r2 l+ l- l2 u+ u- u2 d+ d- d2)" },

  { "GETORIDATA", "", cmd_getori_data, "GETORIDATA - print orientation move log" },
  { "CLEARORIDATA", "", cmd_clear_ori_data, "CLEARORIDATA - reset orientation data" },
  { "RESTOREORI", "", cmd_restore_ori, "RESTOREORI - move cube back in the original orientation" },
  { "GETCOLORDATA", "", cmd_getcolor_data, "GETCOLORDATA - print color data" },

  { "HELP", "", cmd_help, "HELP list of commands" },
};

static constexpr int COMMAND_COUNT = sizeof(command_table) / sizeof(command_table[0]);

char cmd_letter = '?';
int cmd_num = 0;

void increment_cmd_id(char letter) {
  cmd_letter = letter;
  cmd_num++;
}

char get_cmd_id_letter() {
  return cmd_letter;
}

char get_cmd_id_num() {
  return cmd_num;
}

unsigned long get_start_millis() {
  return start_ms;
}

void set_start_millis() {
  start_ms = millis();
}

void rb_make_id(char *out, size_t len) {
  snprintf(out, len, "%c%lu", get_cmd_id_letter(), (unsigned long)get_cmd_id_num());
}

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

  line.trim();
  if (line.length() == 0) return;

  String U = line;
  U.toUpperCase();

  // ------------------------------------------------------------
  // PROTOCOL QUERY (always allowed, no args)
  // ------------------------------------------------------------
  if (U == "PROTO?") {
    increment_cmd_id('p');
    RB_RUN_START('v', "proto?");
    rb_report_protocol();
    RB_RUN_END_OK();
    return;
  }

  char id_letter = 'c';  // default = cube/command

  // ------------------------------------------------------------
  // COMMAND TABLE DISPATCH
  // ------------------------------------------------------------
  for (int i = 0; i < COMMAND_COUNT; i++) {
    const CommandEntry &cmd = command_table[i];

    if (!U.startsWith(cmd.name))
      continue;

    // ==========================================================
    // STRING-BASED COMMANDS
    // ==========================================================
    if (strcmp(cmd.name, "MOVECUBE") == 0 || strcmp(cmd.name, "MOVEROBOT") == 0 || strcmp(cmd.name, "COLORSCAN") == 0) {

      int space_idx = line.indexOf(' ');
      if (space_idx < 0) {
        RB_ERR("CMDROUTER", "missing_argument", "(na)", "cmd=%s", cmd.name);
        return;
      }

      String params = line.substring(space_idx + 1);
      params.trim();
      if (params.length() == 0) {
        RB_ERR("CMDROUTER", "missing_argument", "(na)", "cmd=%s", cmd.name);
        return;
      }

      // ---------------- Command ID ----------------
      char id_leter = 'c';
      if (strcmp(cmd.name, "MOVEROBOT") == 0) id_letter = 'r';
      else if (strcmp(cmd.name, "COLORSCAN") == 0) id_letter = 's';

      increment_cmd_id(id_letter);

      // ---------------- START ----------------
      RB_RUN_START(get_cmd_id_letter(), 
                   params.c_str());

      bool ok = false;

      // ---------------- EXECUTION ----------------
      if (strcmp(cmd.name, "MOVECUBE") == 0) {
        ok = ori.cube_move(params);
      } else if (strcmp(cmd.name, "MOVEROBOT") == 0) {
        ok = ori.robot_move(params);
      } else if (strcmp(cmd.name, "COLORSCAN") == 0) {
        ok = cmd_read_cube_colors_string(params);
      }

      // ---------------- END ----------------
      if (ok) {
        RB_RUN_END_OK();
      } else {
        RB_RUN_END_ERR("execution_failed");
      }
      return;
    }

    // ==========================================================
    // NUMERIC / LEGACY COMMANDS
    // ==========================================================
    int min_args = 0;
    for (const char *p = cmd.fmt; *p; p++) {
      if (*p == '%') min_args++;
    }

    double argv[8] = { 0 };
    String raw[8];
    int argc = parse_args(line, cmd.fmt, argv, 8, raw);

    if (argc < min_args) {
      RB_ERR("CMDROUTER", "invalid_args", "(na)",
             "cmd=%s expected=%d got=%d",
             cmd.name, min_args, argc);
      return;
    }

    // classify numeric commands
    char id_letter = 'v';  // default: verification / misc
    if (strncmp(cmd.name, "MOVE", 4) == 0) id_letter = 'r';
    if (strncmp(cmd.name, "READ", 4) == 0) id_letter = 'v';

    increment_cmd_id(id_letter);

    RB_RUN_START(id_letter, 
                 cmd.name);

    bool ok = cmd.handler(argc, argv);

    if (ok) {
      RB_RUN_END_OK();
    } else {
      RB_RUN_END_ERR("execution_failed");
    }
    return;
  }

  // ------------------------------------------------------------
  // UNKNOWN COMMAND
  // ------------------------------------------------------------
  RB_ERR("CMDROUTER", "unknown_command", "(na)",
         "cmd=%s",
         line.c_str());
  serial_printf("%s", get_help_text().c_str());
}

bool cmd_getcolor_data(int argc, double *argv) {
  String all54 = color_reader.get_cube_colors_string();
  RB_INFO_COLORSCAN("cube_colors", rb_id_,
                    "cube_colors=%s",
                    all54.c_str());
  color_reader.print_face_compact('u');
  color_reader.print_face_compact('r');
  color_reader.print_face_compact('f');
  color_reader.print_face_compact('d');
  color_reader.print_face_compact('l');
  color_reader.print_face_compact('b');
  color_reader.print_cube_colors_string();

  print_colors_detail("get color data");
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

// Always-available rb_id_ (safe fallback)
#ifndef RB_LOCAL_ID
#define RB_LOCAL_ID
static inline const char *rb_get_fallback_id() {
  static char _id[12];
  rb_make_id(_id, sizeof(_id));
  return _id;
}
#define rb_id_ rb_get_fallback_id()
#endif
