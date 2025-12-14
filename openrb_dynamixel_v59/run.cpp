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

extern uint32_t start_ms;

// ---- y poses
#define Y_CENTER 76  // was 73
#define Y_ALIGN 68
#define Y_MID 97
#define Y_UP 112
#define Y_DOWN 33
#define Y_ROTATE_BASE 93
#define Y_ABOVE_DROP 93  // was 86
#define Y_DROP 83        // was 89

// ---- y read color poses
#define Y_C_TOP 65
#define Y_C_MID 48

// ---- x poses
#define X_CENTER 1.0  // was -2

// ---- x read color poses
#define X_C_LEFT -14.0
#define X_C_RIGHT 20.0
#define X_C_CENTER 2  // was 4.0

// ---- g poses
#define G_OPEN 28
#define G_WIDE_OPEN -1
#define G_CLOSE 100
#define G_SOFT_CLOSE 60
#define G_ALIGN_LEFT 100
#define G_ALIGN_RIGHT 100

// ---- b poses
#define B_CENTER 0
#define B_RIGHT 90
#define B_LEFT -90
#define B_BACK -180
#define B_ERR 3

#define B_TOL 3

// -------------------------------------------------------------------
//                      COMMAND TABLE
// -------------------------------------------------------------------

// RUN command constants (placed next to help text)
static constexpr int RUN_ZERO = 0;

static constexpr int RUN_RIGHT_DOWN = 11;
static constexpr int RUN_LEFT_DOWN = 12;
static constexpr int RUN_BACK_DOWN = 13;
static constexpr int RUN_TOP_DOWN = 14;

static constexpr int RUN_DOWN_RIGHT = 21;
static constexpr int RUN_DOWN_LEFT = 22;
static constexpr int RUN_DOWN_BACK = 23;

static constexpr int RUN_CUBE_RIGHT = 31;
static constexpr int RUN_CUBE_LEFT = 32;
static constexpr int RUN_CUBE_BACK = 33;

static constexpr int RUN_RESET_RIGHT = 41;
static constexpr int RUN_RESET_LEFT = 42;
static constexpr int RUN_RESET_BACK = 43;

static constexpr int RUN_ALIGN_CUBE = 60;

// -------------------------------------------------------------------
//                       COMMAND HANDLERS (LOGIC ONLY)
// -------------------------------------------------------------------

// ------------------------------------------------------------
// GETORI
// ------------------------------------------------------------
bool cmd_getori_data(int argc, double *argv) {

  RB_RUN_START('o', "ori_report");

  // ------------------------------------------------------------
  // Orientation string (compact)
  // ------------------------------------------------------------
  String ori_str = ori.get_orientation_string();

  RB_INFO_CUBEORI("orientation", "orientation=%s", ori_str.c_str());

  // ------------------------------------------------------------
  // Orientation diagram (verbose / human)
  // ------------------------------------------------------------
  RB_INFO_CUBEORI("orientation_diagram", "diagram_start", "");

  ori.print_orientation_string();

  RB_INFO_CUBEORI("orientation_diagram", "diagram_end", "");

  // ------------------------------------------------------------
  // Orientation move log
  // ------------------------------------------------------------
  String move_log = ori.get_move_log();

  RB_INFO_CUBEORI("move_log",
                  "move_log=%s",
                  move_log.length() ? move_log.c_str() : "(empty)");

  // ------------------------------------------------------------
  // Associated cube colors snapshot
  // ------------------------------------------------------------
  RB_INFO_CUBEORI("cube_colors",
                  "cube_colors=%s",
                  color_reader.get_cube_colors_string().c_str());

  // ------------------------------------------------------------
  // Command end (duration auto-computed)
  // ------------------------------------------------------------
  RB_RUN_END_OK();

  return true;
}


// ------------------------------------------------------------
// RESETORI
// ------------------------------------------------------------
bool cmd_clear_ori_data(int argc, double *argv) {

  RB_RUN_START('o', "ori_clear");

  // ------------------------------------------------------------
  // Clear orientation + move log
  // ------------------------------------------------------------
  ori.clear_orientation_data();
  ori.clear_move_log();

  RB_INFO_CUBEORI("ori_reset", "status=cleared", "");

  // ------------------------------------------------------------
  // Report current orientation (should be identity)
  // ------------------------------------------------------------
  String ori_str = ori.get_orientation_string();

  RB_INFO_CUBEORI("orientation", "orientation=%s", ori_str.c_str());

  // ------------------------------------------------------------
  // Report move log (should be empty)
  // ------------------------------------------------------------
  String move_log = ori.get_move_log();

  RB_INFO_CUBEORI("move_log", "moves=%s", move_log.length() ? move_log.c_str() : "(empty)");

  // ------------------------------------------------------------
  // End command
  // ------------------------------------------------------------
  RB_RUN_END_OK();

  return true;
}


bool cmd_restore_ori(int argc, double *argv) {

  // ------------------------------------------------------------
  // Restore logical cube orientation
  // ------------------------------------------------------------
  if (!ori.restore_cube_orientation()) {

    RB_ERR_CUBEORI("restore_failed", "(na)", "");

    String ori_str = ori.get_orientation_string();
    RB_INFO_CUBEORI("orientation", "(na)",
                    "orientation=%s",
                    ori_str.c_str());

    String log = ori.get_move_log();
    RB_INFO_CUBEORI("move_log", "(na)",
                    "log=%s",
                    log.c_str());

    return false;
  }

  // ------------------------------------------------------------
  // Physically settle cube after restore
  // ------------------------------------------------------------
  if (!cmdMoveGripperPer(G_OPEN)) {
    RB_ERR_ROBOTMOVE("gripper_open_failed", "(na)", "");
    return false;
  }

  if (!cmdMoveYmm(Y_DOWN)) {
    RB_ERR_ROBOTMOVE("move_y_failed", "(na)",
                     "y_mm=%.1f",
                     (double)Y_DOWN);
    return false;
  }

  if (!cmdMoveGripperPer(G_SOFT_CLOSE)) {
    RB_ERR_ROBOTMOVE("gripper_soft_close_failed", "(na)", "");
    return false;
  }

  // ------------------------------------------------------------
  // Logical state cleanup
  // ------------------------------------------------------------
  ori.clear_move_log();

  // ------------------------------------------------------------
  // Report final state
  // ------------------------------------------------------------
  RB_INFO_CUBEORI("restored", "(na)",
                  "state=identity");

  String ori_str = ori.get_orientation_string();
  RB_INFO_CUBEORI("orientation", "(na)",
                  "orientation=%s",
                  ori_str.c_str());

  String log = ori.get_move_log();
  RB_INFO_CUBEORI("move_log", "(na)",
                  "log=%s",
                  log.c_str());

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

bool cmd_help(int argc, double *argv) {
  Serial.println();
  Serial.println(get_help_text());
  Serial.println();
  return true;
}

// -------------------------------------------------------------------
// INFOSERVO <id> : print key control-table data for one servo
// -------------------------------------------------------------------
// -------------------------------------------------------------------
// INFOSERVO <id> : print key control-table data for one servo
// -------------------------------------------------------------------
void print_info(uint8_t id) {

  RB_INFO_SERVO("servo_info_start", "(na)",
                "servo_id=%d",
                id);

  // ------------------------------------------------------------
  // Presence check
  // ------------------------------------------------------------
  if (!dxl.ping(id)) {
    RB_ERR_SERVO("servo_not_found", "(na)",
                 "servo_id=%d",
                 id);
    return;
  }

  // ------------------------------------------------------------
  // Health check
  // ------------------------------------------------------------
  bool ok = servo_ok(id, true);

  RB_INFO_SERVO("health_check", "(na)",
                "servo_id=%d ok=%s",
                id,
                ok ? "true" : "false");

  if (!ok) {

    RB_INFO_SERVO("reset_attempt", "(na)",
                  "servo_id=%d",
                  id);

    reset_servo(id);

    ok = servo_ok(id, true);

    RB_INFO_SERVO("health_recheck", "(na)",
                  "servo_id=%d ok=%s",
                  id,
                  ok ? "true" : "false");

    if (!ok) {
      RB_ERR_SERVO("reset_failed", "(na)",
                   "servo_id=%d",
                   id);
      return;
    }
  }

  // ------------------------------------------------------------
  // Read control-table data
  // ------------------------------------------------------------
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

  // ------------------------------------------------------------
  // Report summary (single structured line)
  // ------------------------------------------------------------
  RB_INFO_SERVO("servo_status", "(na)",
                "servo_id=%d "
                "op_mode=%d "
                "drive_mode=%d "
                "profile=%s "
                "profile_vel=%d "
                "rpm=%.3f "
                "ticks_per_s=%.1f "
                "profile_accel=%d "
                "pos_min=%d "
                "pos_max=%d "
                "span_deg=%.1f "
                "pos_present=%d",
                id,
                op,
                drv,
                (drv & 0x01) ? "time" : "velocity",
                pv,
                rpm,
                tps,
                pa,
                minL,
                maxL,
                spanDeg,
                pos);

  RB_INFO_SERVO("servo_info_end", "(na)",
                "servo_id=%d",
                id);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

bool cmd_move_xy(int argc, double *argv) {
  double goal_xmm = argv[0];
  if (goal_xmm < -max_xmm || goal_xmm > max_xmm) {
    RB_ERR_ROBOTMOVE("invalid_x_mm",
                     "x_mm=%.2f min=%.2f max=%.2f",
                     goal_xmm, -max_xmm, max_xmm);
    return false;
  }

  double goal_ymm = argv[1];
  if (goal_ymm < min_ymm || goal_ymm > max_ymm) {
    RB_ERR_ROBOTMOVE("invalid_y_mm",
                     "y_mm=%.2f min=%.2f max=%.2f",
                     goal_ymm, min_ymm, max_ymm);
    return false;
  }

  if (!cmdMoveXmm(goal_xmm)) return false;
  if (!cmdMoveYmm(goal_ymm)) return false;
  return true;
}


bool cmd_move_deg(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) {
    RB_ERR_SERVO("servo_not_found",
                 "id=%d", id);
    return false;
  }

  double goal_deg = argv[1];
  if (goal_deg < -185.0 || goal_deg > 360.0) {
    RB_ERR_SERVO("invalid_deg",
                 "id=%d deg=%.2f min=-185.0 max=360.0",
                 id, goal_deg);
    return false;
  }

  if (!cmdMoveServoDeg((uint8_t)id, goal_deg)) return false;
  return true;
}

bool cmd_move_ticks(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) {
    RB_ERR_SERVO("servo_not_found",
                 "id=%d", id);
    return false;
  }

  int goal_ticks = (int)argv[1];
  if (!safeSetGoalPosition(id, goal_ticks)) {
    RB_ERR_SERVO("move_failed_ticks",
                 "id=%d ticks=%d",
                 id, goal_ticks);
    return false;
  }

  return true;
}

bool cmd_move_per(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) {
    RB_ERR_SERVO("servo_not_found",
                 "id=%d", id);
    return false;
  }

  double goal_per = argv[1];
  if (goal_per < -15.0 || goal_per > 115.0) {
    RB_ERR_SERVO("invalid_percentage",
                 "id=%d per=%.2f min=-15.0 max=115.0",
                 id, goal_per);
    return false;
  }

  double goal_deg = per2deg(id, goal_per);
  if (!cmdMoveServoDeg((uint8_t)id, goal_deg)) {
    RB_ERR_SERVO("move_failed_per",
                 "id=%d per=%.2f deg=%.2f",
                 id, goal_per, goal_deg);
    return false;
  }

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
    RB_ERR("movey.invalid_range",
           "y_mm=%.2f min_mm=%.2f max_mm=%.2f",
           goal_mm, min_ymm, max_ymm);
    return false;
  }

  serial_printf_verbose("cmd_move_y: y=%.2fmm\n", goal_mm);
  if (!cmdMoveYmm(goal_mm)) return false;
  return true;
}

bool cmd_move_x(int argc, double *argv) {
  double goal_mm = argv[0];
  if (goal_mm < -max_xmm || goal_mm > max_xmm) {
    RB_ERR("cmd.move_x.invalid_range",
           "x_mm=%.2f min=%.2f max=%.2f",
           goal_mm, -max_xmm, max_xmm);
    return false;
  }

  if (!cmdMoveXmm(goal_mm)) return false;
  return true;
}

bool cmd_move_clamp(int argc, double *argv) {
  if (!dxl.ping(ID_GRIP1) || !dxl.ping(ID_GRIP2)) {
    RB_ERR("cmd.clamp.servo_not_found",
           "grip1=%d grip2=%d",
           dxl.ping(ID_GRIP1), dxl.ping(ID_GRIP2));
    return false;
  }

  dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 0);
  bool ok = cmdMoveGripperClamp();
  dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 1);

  if (!ok) {
    RB_ERR("cmd.clamp.move_failed", "");
    return false;
  }

  return true;
}

bool cmd_move_gripper(int argc, double *argv) {
  if (!dxl.ping(ID_GRIP1) || !dxl.ping(ID_GRIP2)) {
    RB_ERR("cmd.gripper.servo_not_found",
           "grip1=%d grip2=%d",
           dxl.ping(ID_GRIP1), dxl.ping(ID_GRIP2));
    return false;
  }

  double goal_per = argv[0];

  if (goal_per < -5.0 || goal_per > 115.0) {
    RB_ERR("cmd.gripper.invalid_percentage",
           "per=%.2f min=-5.0 max=115.0",
           goal_per);
    return false;
  }

  if (!cmdMoveGripperPer(goal_per)) return false;
  return true;
}

bool cmd_move_wrist_vert(int argc, double *argv) {
  if (!dxl.ping(ID_WRIST) || !dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2)) {
    RB_ERR("cmd.wrist.servo_not_found",
           "wrist=%d arm1=%d arm2=%d",
           dxl.ping(ID_WRIST),
           dxl.ping(ID_ARM1),
           dxl.ping(ID_ARM2));
    return false;
  }

  double goal_deg = argv[0];

  if (goal_deg < -95.0 || goal_deg > 95.0) {
    RB_ERR("cmd.wrist.invalid_deg",
           "deg=%.2f min=-95 max=95",
           goal_deg);
    return false;
  }

  if (!cmdMoveWristDegVertical(goal_deg)) return false;
  return true;
}

bool cmd_color(int argc, double *argv) {
  int read_count = 1;

  if (argc == 1) {
    read_count = (int)argv[0];
  }
  if (read_count > 10) read_count = 10;
  if (read_count < 1) read_count = 1;

  for (int i = 0; i < read_count; i++) {
    String crrColor = read_color();
    if (crrColor.length() == 0) {
      RB_ERR("cmd_color_read_failed",
             "iteration=%d", i);
      return false;
    }
    delay(555);
  }

  return true;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

bool prepBaseForRotation(double nextBaseMoveRelative) {
  if (nextBaseMoveRelative == B_CENTER) return true;

  // serial_printf_verbose("***** start prep base for rotation %.2f\n", nextBaseMoveRelative);

  double b_pos = getPos_deg(ID_BASE);

  // serial_printf_verbose("***** before prep base at %.2f\n", b_pos);

  bool isBaseCenter = (b_pos > B_CENTER - B_TOL && b_pos < B_CENTER + B_TOL);
  bool isBaseRight = (b_pos > B_RIGHT - B_TOL && b_pos < B_RIGHT + B_TOL);
  bool isBaseLeft = (b_pos > B_LEFT - B_TOL && b_pos < B_LEFT + B_TOL);
  bool isBaseBack = (b_pos > B_BACK - B_TOL && b_pos < B_BACK + B_TOL);

  // serial_printf_verbose("***** base at center : %s\n", isBaseCenter ? "yes" : "no");
  // serial_printf_verbose("***** base at right : %s\n", isBaseRight ? "yes" : "no");
  // serial_printf_verbose("***** base at left : %s\n", isBaseLeft ? "yes" : "no");
  // serial_printf_verbose("***** base at back : %s\n", isBaseBack ? "yes" : "no");

  // move one pos to right
  if (nextBaseMoveRelative == B_RIGHT) {
    if (isBaseCenter) return true;
    if (isBaseLeft) return true;
    if (isBaseBack) return true;
  }
  // move one pos to left
  if (nextBaseMoveRelative == B_LEFT) {
    if (isBaseCenter) return true;
    if (isBaseRight) return true;
    if (isBaseLeft) return true;
  }
  if (nextBaseMoveRelative == B_BACK) {
    if (isBaseCenter) return true;
    if (isBaseRight) return true;
  }

  // serial_printf_verbose("***** prep base for rotation move to center\n");

  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!isGripperOpen(G_OPEN)) {
    if (!cmdMoveGripperPer(G_OPEN)) return false;
  }
  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;
  if (!cmdMoveGripperClamp()) return false;
  if (!cmdMoveYmm(Y_ROTATE_BASE)) return false;
  if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) return false;
  if (!lowerCube()) return false;

  // serial_printf_verbose("***** move to center done\n");

  // serial_printf_verbose("***** after prep base at %.2f\n", getPos_deg(ID_BASE));

  return true;
}

/*
#define B_RIGHT 90
#define B_LEFT -90
#define B_BACK -180
*/

// below are relative moves
bool rotateBaseRelative(double baseMoveRelative, bool gripperOn = false) {
  if (!dxl.ping(ID_BASE)) return false;

  // serial_printf_verbose("***** start rotate base relative with %.2f\n", baseMoveRelative);
  // serial_printf_verbose("***** base before prepared pos is %.2f\n", getPos_deg(ID_BASE));

  if (baseMoveRelative == B_CENTER) return true;  // no move

  if (!prepBaseForRotation(baseMoveRelative)) return false;

  double b_pos = getPos_deg(ID_BASE);

  // serial_printf_verbose("***** base after prep pos is %.2f\n", b_pos);

  bool isBaseCenter = (b_pos > B_CENTER - B_TOL && b_pos < B_CENTER + B_TOL);
  bool isBaseRight = (b_pos > B_RIGHT - B_TOL && b_pos < B_RIGHT + B_TOL);
  bool isBaseLeft = (b_pos > B_LEFT - B_TOL && b_pos < B_LEFT + B_TOL);
  bool isBaseBack = (b_pos > B_BACK - B_TOL && b_pos < B_BACK + B_TOL);

  // serial_printf_verbose("***** base at center : %s\n", isBaseCenter ? "yes" : "no");
  // serial_printf_verbose("***** base at right : %s\n", isBaseRight ? "yes" : "no");
  // serial_printf_verbose("***** base at left : %s\n", isBaseLeft ? "yes" : "no");
  // serial_printf_verbose("***** base at back : %s\n", isBaseBack ? "yes" : "no");

  double baseNextMove = B_CENTER;

  // serial_printf_verbose("***** rotate base relative with %.2f\n", baseMoveRelative);

  // move from center
  if (isBaseCenter) {
    if (baseMoveRelative == B_RIGHT) baseNextMove = B_RIGHT;
    if (baseMoveRelative == B_LEFT) baseNextMove = B_LEFT;
    if (baseMoveRelative == B_BACK) baseNextMove = B_BACK;
  }
  // move from right
  else if (isBaseRight) {
    if (baseMoveRelative == B_RIGHT) return false;
    if (baseMoveRelative == B_LEFT) baseNextMove = B_CENTER;
    if (baseMoveRelative == B_BACK) baseNextMove = B_LEFT;
  }
  // move from left
  else if (isBaseLeft) {
    if (baseMoveRelative == B_RIGHT) baseNextMove = B_CENTER;
    if (baseMoveRelative == B_LEFT) baseNextMove = B_BACK;
    if (baseMoveRelative == B_BACK) return false;
  }
  // move from back
  else if (isBaseBack) {
    if (baseMoveRelative == B_RIGHT) baseNextMove = B_LEFT;
    if (baseMoveRelative == B_LEFT) return false;
    if (baseMoveRelative == B_BACK) baseNextMove = B_CENTER;
  }
  // serial_printf_verbose("***** rotate base to next move %.2f\n", baseNextMove);

  if (!gripperOn) {
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveYmm(Y_ROTATE_BASE)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
  }
  if (gripperOn) {
    double adjFw = 0;
    double adjBk = 0;
    if (baseNextMove > b_pos) {
      adjFw = B_ERR + 1;
      adjBk = B_ERR;
    } else {
      adjFw = -B_ERR - 1;
      adjBk = -B_ERR;
    }

    // move past the target and the final move after will reset it
    if (!cmdMoveServoDeg(ID_BASE, baseNextMove + adjFw)) return false;
    if (!cmdMoveServoDeg(ID_BASE, baseNextMove - adjBk)) return false;
  }
  return cmdMoveServoDeg(ID_BASE, baseNextMove);
}

bool alignCube() {
  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;

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

char crrColorChar = '.';

bool cmd_read_one_color(int argc, double *argv) {
  if (argc < 1) return false;

  double d_slot = (double)argv[0];
  int slot = (int)d_slot;

  // read one color for slot
  crrColorChar = '.';
  int prev_speed = speed;

  while (1) {
    if (!cmdMoveGripperPer(G_WIDE_OPEN)) break;
    if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) break;

    speed = 0.25;

    if (slot == 1) {
      if (!cmdMoveYmm(Y_C_TOP)) break;
      if (!cmdMoveXmm(X_C_LEFT)) break;
    }
    if (slot == 2) {
      if (!cmdMoveYmm(Y_C_TOP)) break;
      if (!cmdMoveXmm(X_C_CENTER)) break;
    }
    if (slot == 3) {
      if (!cmdMoveYmm(Y_C_TOP)) break;
      if (!cmdMoveXmm(X_C_RIGHT)) break;
    }
    if (slot == 4) {
      if (!cmdMoveYmm(Y_C_MID)) break;
      if (!cmdMoveXmm(X_C_LEFT)) break;
    }
    if (slot == 5) {
      if (!cmdMoveYmm(Y_C_MID)) break;
      if (!cmdMoveXmm(X_C_CENTER)) break;
    }
    if (slot == 6) {
      if (!cmdMoveYmm(Y_C_MID)) break;
      if (!cmdMoveXmm(X_C_RIGHT)) break;
    }
    if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) break;

    crrColorChar = read_color().charAt(0);
    // serial_printf_verbose("      ---C%d=%c\n", slot, crrColorChar);

    speed = prev_speed;
    return true;
  }

  speed = prev_speed;
  return true;
}

bool cmd_read_one_face_colors(int argc, double *argv) {

  // desired read order (hardware-friendly)
  const int read_order[] = { 1, 2, 3, 6, 5, 4 };
  const int N = sizeof(read_order) / sizeof(read_order[0]);

  // temp storage for face positions 1..6
  char face[7];
  for (int i = 1; i <= 6; i++) face[i] = '.';

  RB_INFO("color_face_read_start",
          "order=123654");

  for (int i = 0; i < N; i++) {
    int slot_i = read_order[i];
    double slot = (double)slot_i;

    crrColorChar = '.';
    bool ok = cmd_read_one_color(1, &slot);

    if (!ok || crrColorChar == '.') {
      RB_ERR("color_face_read_slot_failed",
             "slot=%d", slot_i);
      face[slot_i] = '.';
      continue;
    }

    face[slot_i] = crrColorChar;

    RB_INFO("color_face_read_slot",
            "slot=%d color=%c",
            slot_i, crrColorChar);
  }

  // build final string in canonical 1..6 order
  String face_colors;
  face_colors.reserve(6);
  for (int i = 1; i <= 6; i++) {
    face_colors += face[i];
  }

  RB_INFO("color_face_read_end",
          "face_colors=%s",
          face_colors.c_str());

  return true;
}


bool cmd_run(int argc, double *argv) {

  // ------------------------------------------------------------
  // Validate arguments
  // ------------------------------------------------------------
  if (argc < 1) {
    RB_ERR_ROBOTMOVE("missing_argument", "(na)",
                     "expected=run_no");
    return false;
  }

  speed = 1.0;
  int run_no = (int)argv[0];

  RB_INFO_ROBOTMOVE("run_start", "(na)",
                    "run_no=%d",
                    run_no);

  // ------------------------------------------------------------
  // RUN_ZERO — standby / neutral position
  // ------------------------------------------------------------
  if (run_no == RUN_ZERO) {

    if (!prepBaseForRotation(B_LEFT)) {
      RB_ERR_ROBOTMOVE("prep_base_failed", "(na)", "dir=left");
      return false;
    }

    if (!prepBaseForRotation(B_RIGHT)) {
      RB_ERR_ROBOTMOVE("prep_base_failed", "(na)", "dir=right");
      return false;
    }

    if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) {
      RB_ERR_SERVO("base_center_failed", "(na)", "");
      return false;
    }

    if (!isGripperOpen(G_WIDE_OPEN)) {
      if (!cmdMoveGripperPer(G_WIDE_OPEN)) {
        RB_ERR_ROBOTMOVE("gripper_open_failed", "(na)", "");
        return false;
      }
    }

    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!rotateBaseRelative(B_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;
    if (!cmdMoveGripperPer(G_OPEN)) return false;
    if (!cmdMoveYmm(Y_DOWN)) return false;
    if (!cmdMoveGripperPer(G_SOFT_CLOSE)) return false;

    RB_INFO_ROBOTMOVE("run_complete", "(na)",
                      "run_no=%d",
                      run_no);
    return true;
  }

  // ------------------------------------------------------------
  // Face-to-down rotations (11–14)
  // ------------------------------------------------------------
  if (run_no == RUN_RIGHT_DOWN || run_no == RUN_LEFT_DOWN || run_no == RUN_BACK_DOWN || run_no == RUN_TOP_DOWN) {

    RB_INFO_ROBOTMOVE("cube_face_to_down", "(na)",
                      "run_no=%d",
                      run_no);

    if (!cmdMoveGripperPer(G_OPEN)) return false;

    if (!cmdMoveYmm(Y_CENTER)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;

    // wrist orientation differs
    if (run_no == RUN_LEFT_DOWN)
      if (!cmdMoveWristDegVertical(W_VERT)) return false;
    if (run_no == RUN_TOP_DOWN)
      if (!cmdMoveWristDegVertical(W_HORIZ_LEFT)) return false;
    if (run_no == RUN_RIGHT_DOWN || run_no == RUN_BACK_DOWN)
      if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;

    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveGripperClamp()) return false;
    if (!liftCube()) return false;
    if (!cmdMoveYmm(Y_UP)) return false;

    if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) return false;

    // rotate cube vertically
    if (run_no == RUN_RIGHT_DOWN || run_no == RUN_BACK_DOWN)
      if (!cmdMoveWristDegVertical(W_VERT)) return false;
    if (run_no == RUN_LEFT_DOWN || run_no == RUN_TOP_DOWN)
      if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;

    if (!lowerCube()) return false;
    if (!cmdMoveGripperPer(G_OPEN)) return false;

    if (!cmdMoveYmm(Y_CENTER + 3)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;

    RB_INFO_ROBOTMOVE("run_complete", "(na)",
                      "run_no=%d",
                      run_no);
    return true;
  }

  // ------------------------------------------------------------
  // Bottom-layer rotations (21–23)
  // ------------------------------------------------------------
  if (run_no == RUN_DOWN_RIGHT || run_no == RUN_DOWN_LEFT || run_no == RUN_DOWN_BACK) {

    RB_INFO_ROBOTMOVE("bottom_layer_rotate", "(na)",
                      "run_no=%d",
                      run_no);

    if (!cmdMoveGripperPer(G_OPEN)) return false;

    if (run_no == RUN_DOWN_RIGHT)
      if (!prepBaseForRotation(B_RIGHT)) return false;
    if (run_no == RUN_DOWN_LEFT)
      if (!prepBaseForRotation(B_LEFT)) return false;
    if (run_no == RUN_DOWN_BACK)
      if (!prepBaseForRotation(B_BACK)) return false;

    if (!cmdMoveYmm(Y_MID)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;
    if (!cmdMoveGripperClamp()) return false;

    if (run_no == RUN_DOWN_RIGHT)
      if (!rotateBaseRelative(B_RIGHT, true)) return false;
    if (run_no == RUN_DOWN_LEFT)
      if (!rotateBaseRelative(B_LEFT, true)) return false;
    if (run_no == RUN_DOWN_BACK)
      if (!rotateBaseRelative(B_BACK, true)) return false;

    if (!cmdMoveGripperPer(G_OPEN)) return false;
    if (!cmdMoveYmm(Y_CENTER)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;

    RB_INFO_ROBOTMOVE("run_complete", "(na)",
                      "run_no=%d",
                      run_no);
    return true;
  }

  // ------------------------------------------------------------
  // Full cube rotations (31–33)
  // ------------------------------------------------------------
  if (run_no == RUN_CUBE_RIGHT) return rotateBaseRelative(B_RIGHT);
  if (run_no == RUN_CUBE_LEFT) return rotateBaseRelative(B_LEFT);
  if (run_no == RUN_CUBE_BACK) return rotateBaseRelative(B_BACK);

  // ------------------------------------------------------------
  // Base reset (41–43)
  // ------------------------------------------------------------
  if (run_no == RUN_RESET_RIGHT) return prepBaseForRotation(B_RIGHT);
  if (run_no == RUN_RESET_LEFT) return prepBaseForRotation(B_LEFT);
  if (run_no == RUN_RESET_BACK) return prepBaseForRotation(B_BACK);

  // ------------------------------------------------------------
  // Align cube
  // ------------------------------------------------------------
  if (run_no == RUN_ALIGN_CUBE) {
    return alignCube();
  }

  // ------------------------------------------------------------
  // Unknown run number
  // ------------------------------------------------------------
  RB_ERR_ROBOTMOVE("invalid_run_no", "(na)",
                   "run_no=%d",
                   run_no);
  return false;
}

bool robot_move_callback(const String &mv) {

  // NOTE: no INFO here on purpose (very chatty path)

  if (mv == "z_plus") {
    static double arg = RUN_RIGHT_DOWN;
    return cmd_run(1, &arg);
  }
  if (mv == "z_minus") {
    static double arg = RUN_LEFT_DOWN;
    return cmd_run(1, &arg);
  }
  if (mv == "z_180") {
    static double arg = RUN_TOP_DOWN;
    return cmd_run(1, &arg);
  }
  if (mv == "y_plus") {
    static double arg = RUN_CUBE_LEFT;
    return cmd_run(1, &arg);
  }
  if (mv == "y_minus") {
    static double arg = RUN_CUBE_RIGHT;
    return cmd_run(1, &arg);
  }
  if (mv == "y_180") {
    static double arg = RUN_CUBE_BACK;
    return cmd_run(1, &arg);
  }
  if (mv == "d_minus") {
    static double arg = RUN_DOWN_LEFT;
    return cmd_run(1, &arg);
  }
  if (mv == "d_plus") {
    static double arg = RUN_DOWN_RIGHT;
    return cmd_run(1, &arg);
  }
  if (mv == "d_180") {
    static double arg = RUN_DOWN_BACK;
    return cmd_run(1, &arg);
  }

  // ---- ERROR ----
  RB_ERR_ROBOTMOVE("robot_move_invalid",
                   "move=%s", mv.c_str());

  return false;
}

//---------------------------------------------------------------------------
void print_colors_detail(char *txt) {
  String all54 = color_reader.get_cube_colors_string();

  RB_INFO_COLORCHECK("check_start", "(na)",
                     "context=%s", txt);

  RB_INFO_COLORCHECK("check", "(na)",
                     "cube_colors=%s", all54.c_str());

  color_analyzer.set_colors(all54);

  bool valid_colors = color_analyzer.is_color_string_valid_bool();

  RB_INFO_COLORCHECK("check", "(na)",
                     "valid=%d", valid_colors ? 1 : 0);

  if (!valid_colors) {
    RB_ERR_COLORCHECK("check_colors_invalid", "(na)",
                      "reason=%s",
                      color_analyzer.get_string_check_log().c_str());

    if (color_analyzer.is_string_fixable_bool()) {
      String fixed;
      if (color_analyzer.try_fix_color_string(fixed)) {
        RB_INFO_COLORCHECK("check_fixable", "(na)",
                           "fixed_string=%s", fixed.c_str());
      } else {
        RB_ERR_COLORCHECK("check_fix_failed", "(na)",
                          "attempted=1");
      }
    } else {
      RB_ERR_COLORCHECK("check_not_fixable", "(na)",
                        "attempted=0");
    }
  } else {
    RB_INFO_COLORCHECK("check_stage_eval", "(na)", "");

    for (int s = 0; s < color_analyzer.get_stage_count(); s++) {
      const char *state = "none";
      if (color_analyzer.is_stage_done_bool(s)) state = "done";
      else if (color_analyzer.is_stage_partial_bool(s)) state = "partial";

      RB_INFO_COLORCHECK("check_stage", "(na)",
                         "stage_id=%d name=%s state=%s",
                         s,
                         color_analyzer.get_stage_name(s),
                         state);
    }
  }

  RB_INFO_COLORCHECK("check_end", "(na)", "context=%s", txt);
}

// Your callback implementation
char read_one_color_cb(int slot) {
  double arg = (double)slot;

  if (!cmd_read_one_color(1, &arg)) {
    RB_ERR("RUN cmd=read_color_slot",
           "slot=%d", slot);
    return '.';
  }

  return crrColorChar;
}

bool cmd_read_cube_colors_string(const String &mode_in) {
  String mode = mode_in;
  mode.toLowerCase();

  bool do_bottom = false;
  bool do_full = false;
  bool do_solved = false;

  if (mode == "all") {
    do_full = true;
  } else if (mode == "bottom") {
    do_bottom = true;
  } else if (mode == "solved") {
    do_solved = true;
  } else {
    RB_ERR_COLORSCAN("invalid_mode",
                     "mode=%s", mode.c_str());
    return false;
  }

  // ---- BEFORE ----
  String before = color_reader.get_cube_colors_string();
  RB_INFO_COLORSCAN("colors_before_scan", "(na)",
                    "cube_colors=%s",
                    before.c_str());

  print_colors_detail((char *)"before_read");

  ori.clear_orientation_data();
  ori.clear_move_log();

  bool ok = false;

  if (do_full) {
    RB_INFO_COLORSCAN("scan_start", "(na)",
                      "mode=%s",
                      "full");
    ok = color_reader.read_cube_full();
  } else if (do_solved) {
    RB_INFO_COLORSCAN("scan_start", "(na)",
                      "mode=%s",
                      "solved");
    color_reader.fill_solved_cube();
    ok = true;
  } else if (do_bottom) {
    RB_INFO_COLORSCAN("scan_start", "(na)",
                      "mode=%s",
                      "bottom");
    ok = color_reader.read_cube_bottom();
  }

  if (!ok) {
    RB_ERR_COLORSCAN("scan_failed", "(na)", "");
    return false;
  }

  // ---- AFTER ----
  String after = color_reader.get_cube_colors_string();

  if (!ori.restore_cube_orientation()) {
    RB_ERR_CUBEORI("ori_restore_failed", "(na)", "");
    return false;
  }

  RB_INFO_COLORSCAN("colors_after_scan", "(na)",
                    "cube_colors=%s",
                    before.c_str());

  print_colors_detail((char *)"after_read");

  static double arg = 0;
  return cmd_run(1, &arg);
}
