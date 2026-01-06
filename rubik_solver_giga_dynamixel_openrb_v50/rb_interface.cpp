#include "rb_interface.h"
#include "logging.h"
#include "ui_touch.h"
#include "ui_cube_view.h"
#include "ui_button.h"
#include "run.h"

/*
  uint32_t id = rb.send_command("READSERVO", "0");
  Serial.printf("Started command id %lu\n", id);
*/

// ============================================================
// Constructor
// ============================================================
RBInterface::RBInterface() {}

// ============================================================
// Begin communication
// ============================================================
bool RBInterface::begin() {
  serial_rb->begin(115200);
  serial_rb->setTimeout(500);

  setFooter("starting rb...", _RUNNING_NOSTOP);

  // Flush startup noise
  delay(300);
  while (serial_rb->available()) serial_rb->read();

  // --- ACTIVE PROBE ---
  uint32_t start = millis();
  serial_rb->println("VERSION");  // or VERSION / PING

  while (millis() - start < timeout_ms) {
    if (serial_rb->available()) {
      String line = serial_rb->readStringUntil('\n');
      line.trim();
      if (line.length()) {
        return true;  // ✅ RB responded
      }
    }
  }

  return false;  // ❌ timed out, no RB
}

// ============================================================
// Send command (blocking until command_start)
// ============================================================
uint32_t RBInterface::send_command(const String& command,
                                   const String& params) {
  waiting_for_start_ = true;

  String line = command;
  if (!params.isEmpty()) {
    line += " ";
    line += params;
  }

  serial_rb->println(line);

  unsigned long t0 = millis();
  while (waiting_for_start_ && millis() - t0 < 3000) {
    poll();
  }
  if (waiting_for_start_) {
    LOG_ERR("[RB] command %s %s timed out\n", command, params);
    return -1;  // command never received TODO-> TO IMPLEMENT use this}
  }
  current_cmd_id_++;
  return current_cmd_id_;
}

bool RBInterface::send_stop_command() {
  waiting_for_end_ = true;
  int stopCmdId = send_command("STOP", "");
  if (stopCmdId < 0) return false;

  unsigned long t0 = millis();
  while (waiting_for_end_ && millis() - t0 < 3000) {
    poll();
  }
  if (waiting_for_end_) {
    LOG_ERR("[RB] stop timed out, no end command received\n");
    return false;  // command never received TODO-> TO IMPLEMENT use this
  }
  return true;
}

// ============================================================
// Poll serial (call from loop)
// ============================================================
void RBInterface::poll() {
  while (serial_rb->available()) {
    String line = serial_rb->readStringUntil('\n');
    line.trim();
    if (!line.isEmpty())
      handle_line(line);
  }
}

// ============================================================
// Handle one protocol line
// ============================================================
void RBInterface::handle_line(const String& line) {

  // ---------------- ERROR ----------------
  if (line.startsWith("[!] ERR")) {
    // [!] ERR MODULE (id) rest...
    int m_start = 7;
    int m_end = line.indexOf(' ', m_start);
    String module = line.substring(m_start, m_end);

    uint32_t id = parse_id(line);
    String payload = line.substring(line.indexOf(')', m_end) + 1);
    payload.trim();

    if (error_cb_) error_cb_(module, id, payload);
    return;
  }

  // ---------------- CMD ----------------
  if (line.startsWith("CMD (")) {
    uint32_t id = parse_id(line);

    if (line.indexOf("command_start=") >= 0) {
      current_cmd_id_ = id;
      waiting_for_start_ = false;
      return;
    }

    if (line.indexOf("command_end=") >= 0) {
      String result, duration;

      int r = line.indexOf("result=");
      if (r >= 0) {
        int e = line.indexOf(' ', r);
        result = line.substring(r + 7, e < 0 ? line.length() : e);
      }

      int d = line.indexOf("duration=");
      if (d >= 0)
        duration = line.substring(d + 9);

      if (command_end_cb_)
        command_end_cb_(result, duration);
      waiting_for_end_ = false;
      return;
    }
  }

  // ---------------- INFO ----------------
  // MODULE (id) info=...
  int p = line.indexOf('(');
  int q = line.indexOf(')', p);
  if (p > 0 && q > p) {
    String module = line.substring(0, p);
    module.trim();

    uint32_t id = line.substring(p + 1, q).toInt();
    String payload = line.substring(q + 1);
    payload.trim();

    if (info_cb_) info_cb_(module, id, payload);
  }
}

// ============================================================
// Parse "(id)" token
// ============================================================
uint32_t RBInterface::parse_id(const String& line) {
  int p = line.indexOf('(');
  int q = line.indexOf(')', p);
  if (p < 0 || q < 0) return 0;
  return line.substring(p + 1, q).toInt();
}

// ============================================================
// Callback setters
// ============================================================
void RBInterface::on_command_end(command_end_cb_t cb) {
  command_end_cb_ = cb;
}

void RBInterface::on_info(info_cb_t cb) {
  info_cb_ = cb;
}

void RBInterface::on_error(error_cb_t cb) {
  error_cb_ = cb;
}

RBInterface rb;

// ------------------------------------------------------------------------------------------------------------------------------------------------
// RB wrappers

struct rb_cmd_state_t {
  bool finished_bool = false;
  bool ok_bool = false;
  String command_name;
  String command_params;
  String last_error;

  String color_string_54;
  String orientation;

  // NEW
  String version_string;

  // COLORSCAN
  String color_string_curr;  // full 54 when available, or partial
  char color_one_color_char;
  String color_read_map;  // f6_r6_b6_...
  int color_read_step = -1;

  // MOVES
  int cube_move_step = -1;
  int robot_move_step = -1;
};

struct rb_servo_status_t {
  int servo_id = -1;
  String servo_name;

  int pos_ticks = 0;
  float pos_deg = 0.0f;
  float pos_per = 0.0f;

  int current_ma = 0;
  int temp_c = 0;

  int min_ticks = 0;
  int zero_ticks = 0;
  int max_ticks = 0;

  int cmd_id = -1;  // command that produced this snapshot
};

static std::map<int, rb_servo_status_t> last_servo_status_by_id;
static uint32_t last_servo_status_cmd = 0;

static std::map<uint32_t, rb_cmd_state_t> cmd_states;
static uint32_t last_finished_cmd_id = 0;
static String last_color_string_54 = "";
static String last_orientation = "";
static String last_cube_solution = "";

// ================= GLOBAL LAST-SEEN STATE =================
static char last_color_one_color_char = '.';
static uint32_t last_color_one_color_cmd = 0;

static String last_color_string_curr;
static uint32_t last_color_string_curr_cmd = 0;

static int last_color_read_step = -1;
static uint32_t last_color_read_step_cmd = 0;

static int last_cube_move_step = -1;
static uint32_t last_cube_move_step_cmd = 0;

static int last_robot_move_step = -1;
static uint32_t last_robot_move_step_cmd = 0;

static String last_color_read_map;
static uint32_t last_color_read_map_cmd = 0;

void set_last_color_string_54(String a_color_string) {
  last_color_string_54 = a_color_string;
}
void set_last_orientation(String a_ori) {
  last_orientation = a_ori;
}

static void rb_command_end_cb(const String& result, const String& duration) {
  LOG_PRINTF("[RB CMD] done %s (%s)\n", result.c_str(), duration.c_str());

  uint32_t id = rb.get_current_cmd_id();  // see note below

  auto& st = cmd_states[id];
  st.finished_bool = true;
  st.ok_bool = (result == "ok");
  last_finished_cmd_id = id;

  // --------------------------------------------------
  // PRUNE OLD COMMAND STATES (keep last 20)
  // --------------------------------------------------
  const size_t MAX_CMD_HISTORY = 20;

  if (cmd_states.size() > MAX_CMD_HISTORY) {
    auto oldest = cmd_states.begin();  // std::map is ordered by key
    cmd_states.erase(oldest);
  }
}

static void rb_error_cb(const String& module, uint32_t id, const String& payload) {
  LOG_PRINTF("[RB ERR] %s (%lu) %s\n", module.c_str(), id, payload.c_str());

  auto& st = cmd_states[id];
  if (!st.last_error.isEmpty())
    st.last_error += " | ";
  st.last_error += module + ": " + payload;

  addErrorLine("[RB ERR] " + st.last_error);
}

static void rb_info_cb(const String& mod,
                       uint32_t id,
                       const String& msg) {
  LOG_PRINTF("[RB INFO] %s (%lu) %s\n",
             mod.c_str(),
             (unsigned long)id,
             msg.c_str());

  auto& st = cmd_states[id];

  // ------------------------------------------------------------
  // SERVOS info=servo_status
  // ------------------------------------------------------------
  if (mod == "SERVOS" && msg.startsWith("info=servo_status")) {

    rb_servo_status_t st_servo;
    st_servo.cmd_id = id;

    auto extract_int = [&](const char* key, int& out) {
      int p = msg.indexOf(key);
      if (p < 0) return;
      p += strlen(key);
      int e = msg.indexOf(' ', p);
      out = msg.substring(p, e < 0 ? msg.length() : e).toInt();
    };

    auto extract_float = [&](const char* key, float& out) {
      int p = msg.indexOf(key);
      if (p < 0) return;
      p += strlen(key);
      int e = msg.indexOf(' ', p);
      out = msg.substring(p, e < 0 ? msg.length() : e).toFloat();
    };

    auto extract_string = [&](const char* key, String& out) {
      int p = msg.indexOf(key);
      if (p < 0) return;
      p += strlen(key);
      int e = msg.indexOf(' ', p);
      out = msg.substring(p, e < 0 ? msg.length() : e);
    };

    extract_int("servo_id=", st_servo.servo_id);
    extract_string("servo_name=", st_servo.servo_name);

    extract_int("pos_ticks=", st_servo.pos_ticks);
    extract_float("pos_deg=", st_servo.pos_deg);
    extract_float("pos_per=", st_servo.pos_per);

    extract_int("current_ma=", st_servo.current_ma);
    extract_int("temp_c=", st_servo.temp_c);

    extract_int("min_ticks=", st_servo.min_ticks);
    extract_int("zero_ticks=", st_servo.zero_ticks);
    extract_int("max_ticks=", st_servo.max_ticks);

    if (st_servo.servo_id >= 0) {
      last_servo_status_by_id[st_servo.servo_id] = st_servo;
      last_servo_status_cmd = id;
    }

    return;
  }

  // ------------------------------------------------------------
  // COLORSCAN color_string_curr / color_string_curr_face
  // ------------------------------------------------------------
  if (msg.indexOf("color_string_curr") >= 0) {
    int eq = msg.indexOf('=');
    if (eq > 0) {
      String value = msg.substring(eq + 1);
      value.trim();

      st.color_string_curr = value;
      last_color_string_curr = value;
      last_color_string_curr_cmd = id;
      if (send_cube_view_bool) {
        ui_cube_view_set_colors(value);
        // update buttons
        buttons_set_color_string(value.c_str());
      }
    }
    return;
  }

  // ------------------------------------------------------------
  // COLORSCAN color_string_curr / color_string_curr_face
  // info=read_one_color color=G
  // ------------------------------------------------------------
  if (msg.indexOf("read_one_color") >= 0) {
    int eq = msg.indexOf('color=');
    if (eq > 0) {
      String value = msg.substring(eq + 1);
      value.trim();

      st.color_one_color_char = value.charAt(0);
      last_color_one_color_char = st.color_one_color_char;
      last_color_one_color_cmd = id;
      // update buttons
      buttons_set_one_color_string(last_color_one_color_char);
    }
    return;
  }

  // ------------------------------------------------------------
  // COLORSCAN color_read_step
  // ------------------------------------------------------------
  const char* k_color_step = "color_read_step=";
  if (msg.startsWith(k_color_step)) {
    int step = msg.substring(strlen(k_color_step)).toInt();

    st.color_read_step = step;
    last_color_read_step = step;
    last_color_read_step_cmd = id;

    // TODO-> TO TEST
    if (send_readcolors_progress_bool) {
      ui_moves_progress_set_index(step);
    }
    //
    return;
  }

  // ------------------------------------------------------------
  // COLORSCAN color_read_start_with_map
  // ------------------------------------------------------------
  const char* k_color_map = "color_read_start_with_map=";
  if (msg.startsWith(k_color_map)) {
    String map = msg.substring(strlen(k_color_map));
    map.trim();

    st.color_read_map = map;
    last_color_read_map = map;
    last_color_read_map_cmd = id;

    // TODO-> TO TEST
    if (send_readcolors_progress_bool) {
      //TODO-> OPTIONAL TO IMPLEMENT actual colors for the z+ y+ etc map

      // below are most likely the colors, they will be trimmed by the ui code if too many
      ui_moves_progress_set_map(map, "GRBOOBRGWYYW");
      ui_moves_progress_set_index(0);
    }
    return;
  }

  // ------------------------------------------------------------
  // robot_move_step
  // ------------------------------------------------------------
  const char* k_robot_step = "robot_move_step=";
  if (msg.startsWith(k_robot_step)) {
    int step = msg.substring(strlen(k_robot_step)).toInt();

    st.robot_move_step = step;
    last_robot_move_step = step;
    last_robot_move_step_cmd = id;

    // TODO-> TO TEST
    if (send_move_robot_progress_bool) {
      ui_moves_progress_set_index(step);
    }
    return;
  }

  // ------------------------------------------------------------
  // CUBEMOVE cube_move_step
  // ------------------------------------------------------------
  int cms = msg.indexOf("cube_move_step=");
  if (cms >= 0) {
    int start = cms + strlen("cube_move_step=");
    int end = msg.indexOf(' ', start);
    int step = msg.substring(start, end < 0 ? msg.length() : end).toInt();

    st.cube_move_step = step;
    last_cube_move_step = step;
    last_cube_move_step_cmd = id;

    // TODO-> TO TEST
    if (send_move_cube_progress_bool) {
      ui_moves_progress_set_index(step);
    }
    return;
  }

  // ------------------------------------------------------------
  // version=v83_|_built_dec_30_2025_14_10_39_|_protocol_v1
  // ------------------------------------------------------------
  const char* k_version_prefix = "version=";
  if (msg.startsWith(k_version_prefix)) {
    String value = msg.substring(strlen(k_version_prefix));
    value.trim();
    st.version_string = value;
    return;
  }

  // ------------------------------------------------------------
  // cube_color_string_54=......................................
  // ------------------------------------------------------------
  const char* k_color_prefix = "cube_color_string_54=";
  if (msg.startsWith(k_color_prefix)) {
    String value = msg.substring(strlen(k_color_prefix));
    value.trim();

    st.color_string_54 = value;
    last_color_string_54 = value;
    if (send_cube_view_bool) {
      ui_cube_view_set_colors(value);
      // update buttons
      buttons_set_color_string(value.c_str());
    }
    return;
  }

  // ------------------------------------------------------------
  // orientation=u->u_r->r_f->f_d->d_l->l_b->b
  // ------------------------------------------------------------
  const char* k_ori_prefix = "orientation=";
  if (msg.startsWith(k_ori_prefix)) {
    String value = msg.substring(strlen(k_ori_prefix));
    value.trim();

    st.orientation = value;
    last_orientation = value;
    // TODO-> TO TEST
    if (send_orientation_data_bool) {
      buttons_set_text_ori(value.c_str());
    }
    return;
  }
}

String getLastColorString() {
  return last_color_string_54;
}
String getLastCubeSolution() {
  return last_cube_solution;
}
void setLastCubeSolution(String a_solution) {
  last_cube_solution = a_solution;
}
String getLastOrientation() {
  return last_orientation;
}
String getColorStringForCmd(int cmd_id) {
  auto it = cmd_states.find(cmd_id);
  if (it == cmd_states.end())
    return "";
  return it->second.color_string_54;
}
String getOrientationForCmd(int cmd_id) {
  auto it = cmd_states.find(cmd_id);
  if (it == cmd_states.end())
    return "";
  return it->second.orientation;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------

void init_rb_wrappers() {
  rb.on_command_end(rb_command_end_cb);
  rb.on_error(rb_error_cb);
  rb.on_info(rb_info_cb);
}

bool runCommand(const String& command,
                const String& params,
                int* cmdId) {

  LOG_PRINTF("running command {%s} params {%s}\n", command.c_str(), params.c_str());

  uint32_t id = rb.send_command(command, params);

  if (cmdId)
    *cmdId = (int)id;

  rb_cmd_state_t st;
  st.command_name = command;
  st.command_params = params;
  cmd_states[id] = st;

  unsigned long t0 = millis();
  const unsigned long timeout_ms = 3333;

  while (!cmd_states[id].finished_bool) {
    rb.poll();
    if (millis() - t0 > timeout_ms) {
      cmd_states[id].last_error = "timeout waiting for command_end";
      LOG_ERR("[RB] command start {%s} timeout\n", command.c_str());
      return false;
    }
    delay(1);
  }

  return cmd_states[id].ok_bool;
}

bool checkServosStatus() {
  int cmd_id = -1;
  return runCommand("READSERVO", "0", &cmd_id);
}

String getLastError(int cmdId) {
  auto it = cmd_states.find(cmdId);
  if (it == cmd_states.end())
    return "";
  return it->second.last_error;
}

bool getLastResult(int cmdId) {
  auto it = cmd_states.find(cmdId);
  if (it == cmd_states.end())
    return false;
  return it->second.ok_bool;
}

String getLastCommandName(int cmdId) {
  auto it = cmd_states.find(cmdId);
  if (it == cmd_states.end())
    return "";
  return it->second.command_name;
}

String getLastCommandParams(int cmdId) {
  auto it = cmd_states.find(cmdId);
  if (it == cmd_states.end())
    return "";
  return it->second.command_params;
}

String getRbInterfaceVersion() {
  int cmd_id = -1;

  uint32_t id = rb.send_command("VERSION", "");
  cmd_id = (int)id;

  rb_cmd_state_t st;
  st.command_name = "VERSION";
  cmd_states[id] = st;

  unsigned long t0 = millis();
  const unsigned long timeout_ms = 3333;

  while (!cmd_states[id].finished_bool) {
    rb.poll();
    if (millis() - t0 > timeout_ms) {
      LOG_ERR("[RB] rb version command timeout\n");
      LOG_ERR("[RB] no rb version\n");
      return "err";
    }
    delay(5);
  }

  if (!cmd_states[id].ok_bool) {
    return "err";
  }

  String v = cmd_states[id].version_string;
  v.replace("_", " ");

  if (v.isEmpty()) {
    return "rb interface version unknown";
  }
  return "rb interface version " + v;
}

String getLastColorStringCurr(int* cmd_id) {
  if (cmd_id) *cmd_id = last_color_string_curr_cmd;
  return last_color_string_curr;
}
char getLastColorOneColor(int* cmd_id) {
  if (cmd_id) *cmd_id = last_color_one_color_cmd;
  return last_color_one_color_char;
}
int getLastColorReadStep(int* cmd_id) {
  if (cmd_id) *cmd_id = last_color_read_step_cmd;
  return last_color_read_step;
}

int getLastCubeMoveStep(int* cmd_id) {
  if (cmd_id) *cmd_id = last_cube_move_step_cmd;
  return last_cube_move_step;
}

int getLastRobotMoveStep(int* cmd_id) {
  if (cmd_id) *cmd_id = last_robot_move_step_cmd;
  return last_robot_move_step;
}

String getLastColorReadMap(int* cmd_id) {
  if (cmd_id) *cmd_id = last_color_read_map_cmd;
  return last_color_read_map;
}

bool getLastServoStatus(int servo_id, rb_servo_status_t& out) {
  auto it = last_servo_status_by_id.find(servo_id);
  if (it == last_servo_status_by_id.end())
    return false;

  out = it->second;
  return true;
}

std::map<int, rb_servo_status_t> getAllLastServoStatus(int* cmd_id) {
  if (cmd_id) *cmd_id = last_servo_status_cmd;
  return last_servo_status_by_id;
}

/*
Servo 11 (arm1): pos=75.50 deg, min=-72.0 deg, max=73.0 deg, I=0 mA, T=25 C
Servo 12 (arm2): pos=-59.50 deg, min=-72.0 deg, max=125.0 deg, I=0 mA, T=25 C
Servo 13 (wrist): pos=76.46 deg, min=-41.8 deg, max=318.2 deg, I=0 mA, T=25 C
Servo 14 (grip1): pos=39.29 deg, min=-31.4 deg, max=54.9 deg, I=0 mA, T=25 C
Servo 15 (grip2): pos=133.51 deg, min=-124.1 deg, max=77.3 deg, I=0 mA, T=25 C
Servo 17 (base): pos=0.00 deg, min=-180.0 deg, max=180.0 deg, I=0 mA, T=23 C

Servo 16 (unknown): NO DATA

*/

String getLastServoStatusStr(int servo_id) {

  auto ticks_to_deg = [](const rb_servo_status_t& s, int ticks) -> float {
    if (s.max_ticks == s.min_ticks) return 0.0f;
    float span_ticks = (float)(s.max_ticks - s.min_ticks);
    float span_deg = 360.0f;  // generic safe span
    return (ticks - s.zero_ticks) * (span_deg / span_ticks);
  };

  auto format_servo = [&](const rb_servo_status_t& s) -> String {
    String out;
    out.reserve(128);

    out += "Servo ";
    out += String(s.servo_id);
    out += " (";
    out += s.servo_name;
    out += "): ";

    // ---- NO DATA CASE ----
    if (s.cmd_id == -1) {
      out += "NO DATA";
      return out;
    }

    float min_deg = ticks_to_deg(s, s.min_ticks);
    float max_deg = ticks_to_deg(s, s.max_ticks);

    out += "pos=";
    out += String(s.pos_deg, 2);
    out += " deg, ";

    out += "min=";
    out += String(min_deg, 1);
    out += " deg, ";

    out += "max=";
    out += String(max_deg, 1);
    out += " deg, ";

    out += "I=";
    out += String(s.current_ma);
    out += " mA, ";

    out += "T=";
    out += String(s.temp_c);
    out += " C";

    return out;
  };

  // ------------------------------------------------------------
  // ALL SERVOS (servo_id == 0)
  // ------------------------------------------------------------
  if (servo_id == 0) {
    if (last_servo_status_by_id.empty()) {
      return "no servo info";
    }

    String out;
    out.reserve(320);

    bool first = true;
    for (const auto& it : last_servo_status_by_id) {
      if (!first) out += "\n";
      first = false;

      out += format_servo(it.second);
    }

    return out;
  }

  // ------------------------------------------------------------
  // SINGLE SERVO
  // ------------------------------------------------------------
  rb_servo_status_t s;
  if (!getLastServoStatus(servo_id, s)) {
    return "no servo info";
  }

  return format_servo(s);
}

/* --------------------------------------------------SERIAL EXAMPLES------------------------------------------------------------
static CommandEntry command_table[] = {

  // version
  { "", "", nullptr, "------------- MAIN CUBE SOLVING COMMANDS ----------" },
  { "HELP", "", cmd_help, "HELP - list all available commands" },
  { "VERSION", "", cmd_get_version, "VERSION - show firmware version info" },
  { "MOVEROBOT", "<moves>", nullptr, "MOVEROBOT <moves> - robot moves (z_plus z_minus z_180 y_plus y_minus y_180 d_plus d_minus d_180)" },
  { "MOVECUBE", "<moves>", nullptr, "MOVECUBE <moves> - cube moves (f+ f- f2 b+ b- b2 r+ r- r2 l+ l- l2 u+ u- u2 d+ d- d2)" },
  { "READCOLORS", "<mode>", nullptr, "READCOLORS <all|bottom|solved|centers> - read cube colors (u=white f=green r=red)" },
  { "DETECTCUBE", "", cmd_detect_cube, "DETECTCUBE - detect if a cube is in the base" },
  { "DETECTORI", "", cmd_detect_ori, "DETECTORI - detect orientation from center colors" },
  { "CHECKORI", "", cmd_check_ori, "CHECKORI - reads front face and confirms orientation matches ori" },
  { "RESTOREORI", "", cmd_restore_ori, "RESTOREORI - restore cube to original orientation" },
  { "GETCOLORDATA", "", cmd_getcolor_data, "GETCOLORDATA - print raw color data" },
  { "GETORIDATA", "", cmd_getori_data, "GETORIDATA - print orientation move log" },
  { "CLEARORIDATA", "", cmd_clear_ori_data, "CLEARORIDATA - clear orientation data" },
  { "", "", nullptr, "---------------- DEBUG COMMANDS  ----------------" },
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
  // { "", "", nullptr, "-------------------------------------------------------------------" },
};

----------------------------------------------------------------------------

[!] ERR CMD (0) error=unknown_command

CMD (1) command_start=GETCOLORDATA arg=0.00
        RUN (1) cube_color_string_54=......................................................
        RUN (1) orientation=u->u_r->r_f->f_d->d_l->l_b->b
        COLORCHECK (1) info=color_analyzer_data
[!] ERR COLORCHECK (1) error=color_string_not_valid log=center color not detected for face U
[!] ERR COLORCHECK (1) error=color_string_not_fixable
CMD (1) command_end=GETCOLORDATA result=ok duration=25ms

CMD (2) command_start=READSERVO arg=0.00
        SERVOS (2) info=servo_status servo_id=11 servo_name=arm1 pos_ticks=1189 pos_deg=75.50 pos_per=82.00 current_ma=0 temp_c=25 min_ticks=1000 zero_ticks=2048 max_ticks=2050
        SERVOS (2) info=servo_status servo_id=12 servo_name=arm2 pos_ticks=1371 pos_deg=-59.50 pos_per=19.03 current_ma=0 temp_c=25 min_ticks=1000 zero_ticks=2048 max_ticks=2950
        SERVOS (2) info=servo_status servo_id=13 servo_name=wrist pos_ticks=1345 pos_deg=76.46 pos_per=32.84 current_ma=0 temp_c=25 min_ticks=0 zero_ticks=475 max_ticks=4095
        SERVOS (2) info=servo_status servo_id=14 servo_name=grip1 pos_ticks=2495 pos_deg=39.29 pos_per=65.39 current_ma=0 temp_c=25 min_ticks=2170 zero_ticks=2048 max_ticks=2667
        SERVOS (2) info=servo_status servo_id=15 servo_name=grip2 pos_ticks=529 pos_deg=133.51 pos_per=65.74 current_ma=0 temp_c=25 min_ticks=345 zero_ticks=2048 max_ticks=882
        SERVOS (2) info=servo_status servo_id=17 servo_name=base pos_ticks=2735 pos_deg=0.00 pos_per=66.79 current_ma=0 temp_c=23 min_ticks=0 zero_ticks=2735 max_ticks=4095
        SERVOMOVE (2) info=kinematics_state
        SERVOMOVE (2) info=xy_arms x_mm=0.46 y_mm=31.56 a1_deg=75.59 a2_deg=-59.50
        SERVOMOVE (2) info=wrist w_deg=76.46 w_horiz_r_deg=77.92 w_horiz_l_deg=257.92 w_vert_deg=165.92
        SERVOMOVE (2) info=grip_and_base g1_per=65.39 g2_per=65.74 base_deg=0.00
CMD (2) command_end=READSERVO result=ok duration=198ms

CMD (3) command_start=ONECOLOR arg=1.00
         (3) info=read_one_color color=G
CMD (3) command_end=ONECOLOR result=ok duration=5s179ms

CMD (4) command_start=RUN arg=0.00
        RUN (4) run_zero_start=wrist_is_horiz
CMD (4) command_end=RUN result=ok duration=4s844ms

CMD (5) command_start=INFOSERVO arg=0.00
[!] ERR RUN (5) ping_failed=err func=print_servo_info
CMD (5) command_end=INFOSERVO result=ok duration=8ms

CMD (6) command_start=INFOSERVO arg=17.00
        RUN (6) info=checking_if_servo_is_ok
        RUN (6) info=servo_ok servo_id=17 servo_name=base ok=1
        RUN (6) info=infoservo id=17
        RUN (6) info=operating_mode op_mode=3
        RUN (6) info=drive_mode drive_mode=0 profile_type=VELOCITY
        RUN (6) info=profile_velocity profile_vel=0 rpm=0.00 ticks_per_sec=1.00
        RUN (6) info=profile_acceleration profile_accel=0
        RUN (6) info=position_limits min_ticks=0 max_ticks=4095
        RUN (6) info=position_span span_deg=359.91
        RUN (6) info=present_position pos_ticks=2734
CMD (6) command_end=INFOSERVO result=ok duration=80ms

CMD (1) command_start=VERSION arg=0.00
CMD (1) command_start=version
CMD (1) version=v83_|_built_dec_30_2025_14_10_39_|_protocol_v1
CMD (1) command_end=version
CMD (1) command_end=VERSION result=ok duration=13ms

CMD (0) command_start=READCOLORS params="all"
CMD (0) readcolors_start_mode="all"
        RUN (0) info=full
        COLORSCAN (0) info=orientation_cleared
        COLORSCAN (0) color_scan_start=full total_steps=14
        COLORSCAN (0) color_read_start_with_map=f6_r6_b6_l6_l3_b3_r3_f3_d6_u3_u6_d3
        COLORSCAN (0) color_read_start_with_step_count=14
        COLORSCAN (0) color_read_step=0
        COLORSCAN (0) read_face=f slot=1 color=R
        COLORSCAN (0) color_string_curr_face=f color=f=R........
        COLORSCAN (0) read_face=f slot=4 color=R
        COLORSCAN (0) color_string_curr_face=f color=f=R..R.....
        COLORSCAN (0) read_face=f slot=5 color=R
        COLORSCAN (0) color_string_curr_face=f color=f=R..RR....
        COLORSCAN (0) read_face=f slot=6 color=R
        COLORSCAN (0) color_string_curr_face=f color=f=R..RRR...
        COLORSCAN (0) read_face=f slot=3 color=R
        COLORSCAN (0) color_string_curr_face=f color=f=R.RRRR...


        CMD (6) robot_move_step=2 for_cube_move_face=f for_cube_move_face_qt=1
        ROBOTMOVE (6) info=robot_move_start move=d_plus
        RUN (6) base_->start=center ->rel move=right ->goal=right
        CUBEMOVE (6) info=cube_move_progress cube_move_step=2 total=2 face=f suffix=-
CMD (6) robot_move_step=0 for_cube_move_face=f for_cube_move_face_qt=-1
        ROBOTMOVE (6) info=robot_move_start move=d_minus
        RUN (6) base_->start=right ->rel move=left ->goal=center
CMD (6) command_end=MOVECUBE result=ok duration=41s656ms

CMD (7) command_start=RESTOREORI arg=0.00
        RUN (7) restore_ori_based_on_moves_history=u->l_r->f_f->d_d->r_l->b_b->u
        CUBEORI (7) info=orientation_restore_start
        CUBEORI (7) oerientation_restore_solution_found_with_moves_count=2
        ROBOTMOVE (7) info=robot_move_start move=z_plus
        ROBOTMOVE (7) info=robot_move_start move=y_minus
        RUN (7) base_->start=center ->rel move=right ->goal=right
        RUN (7) info=return_to_pos_zero
        RUN (7) info=ori_restored_to_identity
        RUN (7) info=ori_orientation orientation=u->u r->r f->f d->d l->l b->b
        RUN (7) info=ori_move_log move_log=
CMD (7) command_end=RESTOREORI result=ok duration=26s421ms

CMD (8) command_start=CHECKORI arg=0.00
CMD (8) color_showing_in_front=G
CMD (8) corresponding_face_showing_in_front=f
CMD (8) face_that_should_be_showing_per_ori=f
CMD (8) front_face_matches_ori__robot=f ori=f
CMD (8) command_end=CHECKORI result=ok duration=3s638ms

CMD (9) command_start=RUN arg=0.00
        RUN (9) run_zero_start=wrist_is_horiz
CMD (9) command_end=RUN result=ok duration=3s190ms


Teensy interface

setup start
READY ram_init_ms=44

HELP teensy 4.1 solver v2
version=teensy_4_1_v2
COMMANDS:
FINDSOLUTION cube=<54 chars>
format cube=URFDLB... or cube=WYROGB...
examples
FINDSOLUTION cube=BUFUUDFBLURRFRLRFBDRFUFRBBFRDDFDDBULLLLDLBRLUURDBBLUFD
FINDSOLUTION cube=BWGWWYGBOWRRGRORGBYRGWGRBBGRYYGYYBWOOOOYOBROWWRYBBOWGY

setup end

starting solve...
solve done
SOLUTION result=found solution=B2 L' F L' D' F2 L' B U D2 B R2 D R2 B2 U' R2 F2 L2 B2 U'  move_count=22 time_ms=1109

-------------------------------------------------------------------------------------------------------------------------------------------------*/
