#include "rb_interface.h"
#include "logging.h"
#include "ui_touch.h"

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
bool RBInterface::begin(unsigned long baud, uint32_t timeout_ms) {
  serial_->begin(baud);
  serial_->setTimeout(timeout_ms);

  // Flush startup noise
  delay(300);
  while (serial_->available()) serial_->read();

  // Disable verbose output
  serial_->println("VERBOSEOFF");
  delay(100);
  while (serial_->available()) serial_->read();

  return true;
}

// ============================================================
// Send command (blocking until command_start)
// ============================================================
uint32_t RBInterface::send_command(const String& command,
                                   const String& params) {
  waiting_for_start_ = true;
  current_cmd_id_ = 0;

  String line = command;
  if (!params.isEmpty()) {
    line += " ";
    line += params;
  }

  serial_->println(line);

  unsigned long t0 = millis();
  while (waiting_for_start_ && millis() - t0 < 3000) {
    poll();
  }

  return current_cmd_id_;
}

// ============================================================
// Poll serial (call from loop)
// ============================================================
void RBInterface::poll() {
  while (serial_->available()) {
    String line = serial_->readStringUntil('\n');
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

/* --------------------------------------------------------------------------------------------------------------------------------------------------
  DETECTORI - detect orientation from center colors
  CHECKORI - reads front face and confirms orientation matches ori
  RESTOREORI - restore cube to original orientation
  GETCOLORDATA - print raw color data
  GETORIDATA - print orientation move log
  CLEARORIDATA - clear orientation data
  
---------------- DEBUG COMMANDS  ----------------
  RUN <no>
      0 pos zero |     11 right down   | 12 left down    | 13 back down   | 14 top down
     21 bottom right | 22 bottom right | 23 bottom back
     31 cube right   | 32 cube left    | 33 cube back
     60 align
  READSERVO <id> - show servo summary status
  INFOSERVO <id> - show full servo status
  SETMIN <id> <ticks> - set servo minimum ticks
  SETMAX <id> <ticks> - set servo maximum ticks
  LEDON <id> - turn servo LED on
  LEDOFF <id> - turn servo LED off
  REBOOTALL - reboot all servos
  SETSTOPALL - set global servo error flag
  CLEARSTOPALL - clear global servo error flag
  MOVETICKS <id> <ticks> - move servo to ticks (no smoothing)
  MOVEDEG <id> <deg> - move servo to degrees (smooth)
  MOVEPER <id> <percent> - move servo to percentage (smooth)
  MOVEYMM <mm> - vertical move (42 to 102)
  MOVEXMM <mm> - lateral move (-30 to 30)
  MOVEXYMM <x_mm> <y_mm> - lateral then vertical move (-25..25, 42..102)
  MOVEGRIPPER <percent> - move both grippers (0 to 100)
  MOVEWRISTVERTDEG <deg> - move wrist relative to vertical (-5 to 185)
  CLAMP - clamp gripper
  COLORSENSOR <count> - read color <count> times
  ONECOLOR - read one slot (1..6)
  ONEFACECOLOR - read colors of the front face

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

-------------------------------------------------------------------------------------------------------------------------------------------------*/
