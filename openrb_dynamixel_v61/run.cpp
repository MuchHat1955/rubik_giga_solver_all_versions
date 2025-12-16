#include "cmd_parser.h"
#include "movement.h"
#include "servos.h"
#include "color_sensor.h"
#include "ori.h"
#include "color_reader.h"
#include "color_analyzer.h"
#include "run.h"

void print_info(uint8_t id);

extern double max_xmm;
extern double max_ymm;
extern double min_ymm;
extern double speed;

extern CubeOri ori;
extern CubeColorReader color_reader;
extern ColorAnalyzer color_analyzer;


// -------------------------------------------------------------------
//                      COMMAND TABLE
// -------------------------------------------------------------------

const char runHelp[] PROGMEM =
  "RUN <no>\n"
  "      0 pos zero |"
  "     11 right down   | 12 left down    | 13 back down   | 14 top down\n"
  "     21 bottom right | 22 bottom right | 23 bottom back\n"
  "     31 cube right   | 32 cube left    | 33 cube back\n"
  "     41 reset right  | 42 reset left   | 143 reset back\n"
  "     60 align";

bool cmd_run(int argc, double *argv) {
  if (argc < 1) {
    LOG_ERR(MOD_RUN, "missing run argument");
    return false;
  }

  speed = 1.0;
  int run_no = (int)argv[0];

  switch (run_no) {
    case RUN_ZERO: return cmd_run_zero();
    case RUN_RIGHT_DOWN: return cmd_run_right_down();
    case RUN_LEFT_DOWN: return cmd_run_left_down();
    case RUN_TOP_DOWN: return cmd_run_top_down();
    case RUN_BACK_DOWN: return cmd_run_back_down();

    case RUN_DOWN_RIGHT:
    case RUN_DOWN_LEFT:
    case RUN_DOWN_BACK: return cmd_run_down_layer(run_no);

    case RUN_CUBE_RIGHT: return cmd_run_cube_right();
    case RUN_CUBE_LEFT: return cmd_run_cube_left();
    case RUN_CUBE_BACK: return cmd_run_cube_back();

    case RUN_RESET_RIGHT: return cmd_run_reset_right();
    case RUN_RESET_LEFT: return cmd_run_reset_left();
    case RUN_RESET_BACK: return cmd_run_reset_back();

    case RUN_ALIGN_CUBE: return cmd_run_align_cube();
  }

  speed = 1.0;
  return false;
}

bool robot_move_callback(const String &mv) {
  LOG_INFO(MOD_ROBOTMOVE, "robot move start");
  LOG_VAR("move", mv.c_str());

  if (mv == "z_plus") return cmd_run_right_down();
  if (mv == "z_minus") return cmd_run_left_down();
  if (mv == "z_180") return cmd_run_top_down();

  if (mv == "y_plus") return cmd_run_cube_left();
  if (mv == "y_minus") return cmd_run_cube_right();
  if (mv == "y_180") return cmd_run_cube_back();

  if (mv == "d_minus") return cmd_run_down_layer(RUN_DOWN_LEFT);
  if (mv == "d_plus") return cmd_run_down_layer(RUN_DOWN_RIGHT);
  if (mv == "d_180") return cmd_run_down_layer(RUN_DOWN_BACK);

  LOG_ERR(MOD_CMD, "invalid robot move");
  LOG_VAR("move", mv.c_str());
  return false;
}

// ------------------------------------------------------------
// RUN_ZERO
// ------------------------------------------------------------
bool cmd_run_zero() {
  if (!prepBaseForRotation(B_LEFT)) return false;
  if (!prepBaseForRotation(B_RIGHT)) return false;
  if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) return false;
  if (!isGripperOpen(G_WIDE_OPEN)) {
    if (!cmdMoveGripperPer(G_WIDE_OPEN)) return false;
  }
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!rotateBaseRelative(B_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;
  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveYmm(Y_DOWN)) return false;
  if (!cmdMoveGripperPer(G_SOFT_CLOSE)) return false;
  return true;
}

// ------------------------------------------------------------
// RUN_RIGHT_DOWN
// ------------------------------------------------------------
bool cmd_run_right_down() {
  if (!cmdMoveGripperPer(G_OPEN)) return false;

  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveGripperClamp()) return false;
  if (!liftCube()) return false;
  if (!cmdMoveYmm(Y_UP)) return false;
  if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) return false;

  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_VERT)) return false;

  if (!lowerCube()) return false;
  if (!cmdMoveGripperPer(G_OPEN)) return false;

  if (!cmdMoveYmm(Y_CENTER + 3)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;

  return true;
}

// ------------------------------------------------------------
// RUN_LEFT_DOWN
// ------------------------------------------------------------
bool cmd_run_left_down() {
  if (!cmdMoveGripperPer(G_OPEN)) return false;

  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_CENTER + 3)) return false;
  if (!cmdMoveWristDegVertical(W_VERT)) return false;
  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveGripperClamp()) return false;
  if (!liftCube()) return false;
  if (!cmdMoveYmm(Y_UP)) return false;
  if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;

  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;

  if (!lowerCube()) return false;
  if (!cmdMoveGripperPer(G_OPEN)) return false;

  if (!cmdMoveYmm(Y_CENTER + 3)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;

  return true;
}

// ------------------------------------------------------------
// RUN_TOP_DOWN
// ------------------------------------------------------------
bool cmd_run_top_down() {
  if (!cmdMoveGripperPer(G_OPEN)) return false;

  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_CENTER + 3)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_LEFT)) return false;
  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveGripperClamp()) return false;
  if (!liftCube()) return false;
  if (!cmdMoveYmm(Y_UP)) return false;
  if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;

  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;

  if (!lowerCube()) return false;
  if (!cmdMoveGripperPer(G_OPEN)) return false;

  if (!cmdMoveYmm(Y_CENTER + 3)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;

  return true;
}

// ------------------------------------------------------------
// RUN_BACK_DOWN
// ------------------------------------------------------------
bool cmd_run_back_down() {
  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!rotateBaseRelative(B_LEFT)) return false;

  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveGripperClamp()) return false;
  if (!liftCube()) return false;
  if (!cmdMoveYmm(Y_UP)) return false;
  if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;

  if (!cmdMoveWristDegVertical(W_VERT)) return false;

  if (!lowerCube()) return false;
  if (!cmdMoveGripperPer(G_OPEN)) return false;

  if (!cmdMoveYmm(Y_CENTER + 3)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;

  return true;
}

// ------------------------------------------------------------
// RUN_DOWN_RIGHT / LEFT / BACK
// ------------------------------------------------------------
bool cmd_run_down_layer(int run_no) {
  if (!cmdMoveGripperPer(G_OPEN)) return false;

  if (run_no == RUN_DOWN_RIGHT && !prepBaseForRotation(B_RIGHT)) return false;
  if (run_no == RUN_DOWN_LEFT && !prepBaseForRotation(B_LEFT)) return false;
  if (run_no == RUN_DOWN_BACK && !prepBaseForRotation(B_BACK)) return false;

  if (!cmdMoveYmm(Y_MID)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;
  if (!cmdMoveYmm(Y_MID)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveGripperClamp()) return false;

  if (run_no == RUN_DOWN_RIGHT && !rotateBaseRelative(B_RIGHT, true)) return false;
  if (run_no == RUN_DOWN_LEFT && !rotateBaseRelative(B_LEFT, true)) return false;
  if (run_no == RUN_DOWN_BACK && !rotateBaseRelative(B_BACK, true)) return false;

  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveYmm(Y_CENTER)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;

  return true;
}

// ------------------------------------------------------------
// RUN_CUBE_*
// ------------------------------------------------------------
bool cmd_run_cube_right() {
  return rotateBaseRelative(B_RIGHT);
}
bool cmd_run_cube_left() {
  return rotateBaseRelative(B_LEFT);
}
bool cmd_run_cube_back() {
  return rotateBaseRelative(B_BACK);
}

// ------------------------------------------------------------
// RUN_RESET_*
// ------------------------------------------------------------
bool cmd_run_reset_right() {
  return prepBaseForRotation(B_RIGHT);
}
bool cmd_run_reset_left() {
  return prepBaseForRotation(B_LEFT);
}
bool cmd_run_reset_back() {
  return prepBaseForRotation(B_BACK);
}

// ------------------------------------------------------------
// RUN_ALIGN_CUBE
// ------------------------------------------------------------
bool cmd_run_align_cube() {
  return alignCube();
}

// ------------------------------------------------------------
// GETORI
// ------------------------------------------------------------
bool cmd_getori_data(int argc, double *argv) {
  String s = ori.get_orientation_string();
  //
  LOG_INFO(MOD_CMD, "ori orientation");
  LOG_VAR("orientation", s.c_str());

  ori.print_orientation_string();
  String log = ori.get_move_log();  //
  LOG_INFO(MOD_CMD, "ori move log");
  LOG_VAR("move_log", log.c_str());

  print_colors_detail("get ori data");

  return true;
}

// ------------------------------------------------------------
// RESETORI
// ------------------------------------------------------------
bool cmd_clear_ori_data(int argc, double *argv) {
  ori.clear_orientation_data();
  ori.clear_move_log();  //
  LOG_INFO(MOD_CMD, "ori reset");

  String s = ori.get_orientation_string();  //
  LOG_INFO(MOD_CMD, "ori orientation");
  LOG_VAR("orientation", s.c_str());

  String log = ori.get_move_log();  //
  LOG_INFO(MOD_CMD, "ori move log");
  LOG_VAR("move_log", log.c_str());

  return true;
}

bool cmd_restore_ori(int argc, double *argv) {
  if (!ori.restore_cube_orientation()) {
    //
    LOG_ERR(MOD_CMD, "failed to restore orientation");
    String s = ori.get_orientation_string();
    //
    LOG_INFO(MOD_CMD, "ori orientation");
    LOG_VAR("orientation", s.c_str());

    String ori_log = ori.get_move_log();
    //
    LOG_INFO(MOD_CMD, "ori move log");
    LOG_VAR("move_log", ori_log.c_str());

    return false;
  }

  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveYmm(Y_DOWN)) return false;
  if (!cmdMoveGripperPer(G_SOFT_CLOSE)) return false;
  // At this point ori_ is already identity.  // Only clear the move log.
  ori.clear_move_log();  //
  LOG_INFO(MOD_CMD, "ori restored to identity");

  String s = ori.get_orientation_string();  //
  LOG_INFO(MOD_CMD, "ori orientation");
  LOG_VAR("orientation", s.c_str());

  String log = ori.get_move_log();  //
  LOG_INFO(MOD_CMD, "ori move log");
  LOG_VAR("move_log", log.c_str());
  return true;
}

bool cmd_move_xy(int argc, double *argv) {
  double goal_xmm = argv[0];
  if (goal_xmm < -max_xmm || goal_xmm > max_xmm) {
    //
    LOG_ERR(MOD_CMD, "invalid x mm");
    LOG_VAR("goal_x_mm", goal_xmm);
    LOG_VAR("min_x_mm", -max_xmm);
    LOG_VAR("max_x_mm", max_xmm);
    return false;
  }
  double goal_ymm = argv[1];
  if (goal_ymm < min_ymm || goal_ymm > max_ymm) {
    //
    LOG_ERR(MOD_CMD, "invalid y mm");
    LOG_VAR("goal_y_mm", goal_ymm);
    LOG_VAR("min_y_mm", min_ymm);
    LOG_VAR("max_y_mm", max_ymm);
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
    //
    LOG_ERR(MOD_CMD, "invalid servo deg");
    LOG_VAR("goal_deg", goal_deg);
    return false;
  }
  int goal_ticks = per2ticks(id, goal_deg);  // serial_printf_verbose("cmd_move_deg: id=%d deg=%d", id, goal_deg);

  if (!cmdMoveServoDeg((uint8_t)id, goal_deg)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_move_ticks(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  int goal_ticks = (int)argv[1];  // serial_printf_verbose("cmd_move_ticks: id=%d ticks=%d", id, goal_ticks);

  if (!safeSetGoalPosition(id, goal_ticks)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_move_per(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  double goal_per = argv[1];
  if (goal_per < -15.0 || goal_per > 115.0) {

    //
    LOG_ERR(MOD_CMD, "invalid servo percentage");
    LOG_VAR("goal_per", goal_per);
    return false;
  }

  double goal_deg = per2deg(id, goal_per);  // serial_printf_verbose("cmd_move_per: id=%d per=%d deg=%d", id, goal_per, goal_deg);

  if (!cmdMoveServoDeg((uint8_t)id, goal_deg)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_set_min(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  int t = (int)argv[1];

  if (auto *s = find_servo(id)) {
    // serial_printf_verbose("cmd_set_min: id=%d ticks=%d", id, t);
    // s->set_min_ticks(t);
    return true;
  }
  return false;
}

bool cmd_set_max(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl.ping(id)) return false;

  int t = (int)argv[1];

  if (auto *s = find_servo(id)) {
    // serial_printf_verbose("cmd_set_max: id=%d ticks=%d", id, t);
    s->set_max_ticks(t);
    return true;
  }
  return false;
}

bool cmd_move_y(int argc, double *argv) {
  double goal_mm = argv[0];
  if (goal_mm < min_ymm || goal_mm > max_ymm) {
    //
    LOG_ERR(MOD_CMD, "invalid y mm");
    LOG_VAR("goal_y_mm", goal_mm);
    //
    return false;
  }
  // serial_printf_verbose("cmd_move_y: y=%.2fmm", goal_mm);
  if (!cmdMoveYmm(goal_mm)) return false;
  return true;
}

bool cmd_move_x(int argc, double *argv) {
  double goal_mm = argv[0];
  if (goal_mm < -max_xmm || goal_mm > max_xmm) {

    //
    LOG_ERR(MOD_CMD, "invalid x mm");
    LOG_VAR("goal_x_mm", goal_mm);
    return false;
  }
  // serial_printf_verbose("cmd_move_x: x=%.2fmm", goal_mm);
  if (!cmdMoveXmm(goal_mm)) return false;
  return true;
}

bool cmd_move_clamp(int argc, double *argv) {
  if (!dxl.ping(ID_GRIP1) || !dxl.ping(ID_GRIP2)) return false;
  // serial_printf_verbose("cmd_move_clamp");  // turn off torque off base before clamp
  dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 0);
  bool ok = cmdMoveGripperClamp();  // turn off torque on base after clamp
  dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 1);
  print_servo_status(ID_GRIP1);
  print_servo_status(ID_GRIP2);
  return ok;
}

bool cmd_move_gripper(int argc, double *argv) {
  if (!dxl.ping(ID_GRIP1) || !dxl.ping(ID_GRIP2)) return false;

  double goal_deg = argv[0];

  if (goal_deg < -5.0 || goal_deg > 115.0) {

    //
    LOG_ERR(MOD_CMD, "invalid gripper percentage");
    LOG_VAR("goal_per", goal_deg);
    return false;
  }
  // serial_printf_verbose("cmd_move_gripper: deg=%.2f", goal_deg);
  if (!cmdMoveGripperPer(goal_deg)) return false;
  print_servo_status(ID_GRIP1);
  print_servo_status(ID_GRIP2);
  return true;
}

bool cmd_move_wrist_vert(int argc, double *argv) {
  if (!dxl.ping(ID_WRIST) || !dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2)) return false;

  double goal_deg = argv[0];

  if (goal_deg < -95 || goal_deg > 95) {

    //
    LOG_ERR(MOD_CMD, "invalid wrist degrees");
    LOG_VAR("goal_deg", goal_deg);
    return false;
  }
  // serial_printf_verbose("cmd_move_wrist: deg=%.2f", goal_deg);

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
  // serial_printf_verbose("cmd_color: count=%d", read_count);

  for (int i = 0; i < read_count; i++) {
    //
    LOG_INFO(MOD_CMD, "calling read color");
    LOG_VAR("iteration", i);
    String crrColor = read_color();
    // serial_printf_verbose("COLOR READSERVO clr=%s", crrColor.c_str());
    delay(555);
  }

  return true;
}

bool prepBaseForRotation(double nextBaseMoveRelative) {
  if (nextBaseMoveRelative == B_CENTER) return true;
  // // serial_printf_verbose("***** start prep base for rotation %.2f", nextBaseMoveRelative);

  double b_pos = getPos_deg(ID_BASE);
  // // serial_printf_verbose("***** before prep base at %.2f", b_pos);

  bool isBaseCenter = (b_pos > B_CENTER - B_TOL && b_pos < B_CENTER + B_TOL);
  bool isBaseRight = (b_pos > B_RIGHT - B_TOL && b_pos < B_RIGHT + B_TOL);
  bool isBaseLeft = (b_pos > B_LEFT - B_TOL && b_pos < B_LEFT + B_TOL);
  bool isBaseBack = (b_pos > B_BACK - B_TOL && b_pos < B_BACK + B_TOL);
  // // serial_printf_verbose("***** base at center : %s", isBaseCenter ? "yes" : "no");  // // serial_printf_verbose("***** base at right : %s", isBaseRight ? "yes" : "no");  // // serial_printf_verbose("***** base at left : %s", isBaseLeft ? "yes" : "no");  // // serial_printf_verbose("***** base at back : %s", isBaseBack ? "yes" : "no");
  // move one pos to right
  if (nextBaseMoveRelative == B_RIGHT) {
    if (isBaseCenter) return true;
    if (isBaseLeft) return true;
    if (isBaseBack) return true;
  }  // move one pos to left
  if (nextBaseMoveRelative == B_LEFT) {
    if (isBaseCenter) return true;
    if (isBaseRight) return true;
    if (isBaseLeft) return true;
  }
  if (nextBaseMoveRelative == B_BACK) {
    if (isBaseCenter) return true;
    if (isBaseRight) return true;
  }
  // // serial_printf_verbose("***** prep base for rotation move to center");

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
  // // serial_printf_verbose("***** move to center done");
  // // serial_printf_verbose("***** after prep base at %.2f", getPos_deg(ID_BASE));

  return true;
}

/*
#define B_RIGHT 90
#define B_LEFT -90
#define B_BACK -180
*/

// below are relative moves
bool rotateBaseRelative(double baseMoveRelative, bool gripperOn) {
  if (!dxl.ping(ID_BASE)) return false;
  // // serial_printf_verbose("***** start rotate base relative with %.2f", baseMoveRelative);  // // serial_printf_verbose("***** base before prepared pos is %.2f", getPos_deg(ID_BASE));

  if (baseMoveRelative == B_CENTER) return true;  // no move

  if (!prepBaseForRotation(baseMoveRelative)) return false;

  double b_pos = getPos_deg(ID_BASE);
  // // serial_printf_verbose("***** base after prep pos is %.2f", b_pos);

  bool isBaseCenter = (b_pos > B_CENTER - B_TOL && b_pos < B_CENTER + B_TOL);
  bool isBaseRight = (b_pos > B_RIGHT - B_TOL && b_pos < B_RIGHT + B_TOL);
  bool isBaseLeft = (b_pos > B_LEFT - B_TOL && b_pos < B_LEFT + B_TOL);
  bool isBaseBack = (b_pos > B_BACK - B_TOL && b_pos < B_BACK + B_TOL);
  // // serial_printf_verbose("***** base at center : %s", isBaseCenter ? "yes" : "no");  // // serial_printf_verbose("***** base at right : %s", isBaseRight ? "yes" : "no");  // // serial_printf_verbose("***** base at left : %s", isBaseLeft ? "yes" : "no");  // // serial_printf_verbose("***** base at back : %s", isBaseBack ? "yes" : "no");

  double baseNextMove = B_CENTER;
  // // serial_printf_verbose("***** rotate base relative with %.2f", baseMoveRelative);
  // move from center
  if (isBaseCenter) {
    if (baseMoveRelative == B_RIGHT) baseNextMove = B_RIGHT;
    if (baseMoveRelative == B_LEFT) baseNextMove = B_LEFT;
    if (baseMoveRelative == B_BACK) baseNextMove = B_BACK;
  }  // move from right
  else if (isBaseRight) {
    if (baseMoveRelative == B_RIGHT) return false;
    if (baseMoveRelative == B_LEFT) baseNextMove = B_CENTER;
    if (baseMoveRelative == B_BACK) baseNextMove = B_LEFT;
  }  // move from left
  else if (isBaseLeft) {
    if (baseMoveRelative == B_RIGHT) baseNextMove = B_CENTER;
    if (baseMoveRelative == B_LEFT) baseNextMove = B_BACK;
    if (baseMoveRelative == B_BACK) return false;
  }  // move from back
  else if (isBaseBack) {
    if (baseMoveRelative == B_RIGHT) baseNextMove = B_LEFT;
    if (baseMoveRelative == B_LEFT) return false;
    if (baseMoveRelative == B_BACK) baseNextMove = B_CENTER;
  }  // // serial_printf_verbose("***** rotate base to next move %.2f", baseNextMove);

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
    // // serial_printf_verbose("      ---C%d=%c", slot, crrColorChar);

    speed = prev_speed;
    return true;
  }

  speed = prev_speed;
  return true;
}

bool cmd_read_one_face_colors(int argc, double *argv) {  // desired read order
  int readOrder[] = { 1, 2, 3, 6, 5, 4 };
  const int N = sizeof(readOrder) / sizeof(readOrder[0]);
  // temp storage for face positions 1..6
  char face[7];
  for (int i = 1; i <= 6; i++) face[i] = '.';

  for (int idx = 0; idx < N; idx++) {
    int pos = readOrder[idx];
    double slot = (double)pos;

    crrColorChar = '.';
    bool ok = cmd_read_one_color(1, &slot);

    char result = ok ? crrColorChar : '.';
    face[pos] = result;

    //
    LOG_INFO(MOD_CMD, "read slot");
    LOG_VAR("slot", pos);
    LOG_VAR("color", result);
  }
  // build final string in normal 123456 order
  String faceColors = "";
  for (int i = 1; i <= 6; i++) {
    faceColors += face[i];
  }  //
  LOG_INFO(MOD_CMD, "face colors");
  LOG_VAR("colors", faceColors.c_str());
  return true;
}

//---------------------------------------------------------------------------

void print_colors_detail(char *txt) {
  String all54 = color_reader.get_cube_colors_string();
  //
  LOG_INFO(MOD_CMD, "color analyzer start");
  LOG_VAR("context", txt);
  //
  LOG_INFO(MOD_CMD, "cube colors");
  LOG_VAR("cube_colors", all54.c_str());

  color_analyzer.set_colors(all54);
  bool valid_colors = color_analyzer.is_color_string_valid_bool();
  //
  LOG_INFO(MOD_CMD, "color validity");
  LOG_VAR("valid", valid_colors);
  if (!valid_colors) {
    //
    LOG_INFO(MOD_CMD, "color analyzer log");
    LOG_VAR("log", color_analyzer.get_string_check_log().c_str());

    if (color_analyzer.is_string_fixable_bool()) {
      String fixed;
      if (color_analyzer.try_fix_color_string(fixed)) {
        LOG_INFO(MOD_CMD, "color string is fixable");
        LOG_INFO(MOD_CMD, "color string corrected");
        LOG_VAR("corrected", fixed.c_str());
      } else {
        LOG_ERR(MOD_CMD, "color string fix failed");
      }
    } else {
      //
      LOG_ERR(MOD_CMD, "color string not fixable");
    }
  } else {
    // show the stage//
    LOG_INFO(MOD_CMD, "solving stage");
    for (int s = 0; s < color_analyzer.get_stage_count(); s++) {
      String state = "...";
      if (color_analyzer.is_stage_done_bool(s)) state = "done";
      else if (color_analyzer.is_stage_partial_bool(s)) state = "partial";
      //
      LOG_VAR("stage", s);
      //
      LOG_VAR("name", color_analyzer.get_stage_name(s));
      //
      LOG_VAR("state", state.c_str());
    }
  }  // ~~~~~~~~~~~~~~~~~~~~~~~~  //
  LOG_INFO(MOD_CMD, "color analyzer end");
  LOG_VAR("context", txt);
}

// Your callback implementation
char read_one_color_cb(int slot) {
  double arg = (double)slot;
  if (!cmd_read_one_color(1, &arg)) return '.';
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
    // ~~~~~~~~~~~~~~~~~~~~~~~~
    //
    LOG_ERR(MOD_CMD, "invalid mode");
    LOG_VAR("mode", mode.c_str());
    return false;
  }
  // Before read
  String before = color_reader.get_cube_colors_string();  // ~~~~~~~~~~~~~~~~~~~~~~~~  //
  LOG_INFO(MOD_CMD, "before read cube colors");
  LOG_VAR("cube_colors", before.c_str());
  color_reader.print_cube_colors_string();
  print_colors_detail("before read cube colors");

  ori.clear_orientation_data();
  ori.clear_move_log();

  bool ok = false;

  if (do_full) {  //
    LOG_INFO(MOD_CMD, "full");
    ok = color_reader.read_cube_full();
  } else if (do_solved) {  //
    LOG_INFO(MOD_CMD, "solved");
    color_reader.fill_solved_cube();
    ok = true;
  } else if (do_bottom) {  //
    LOG_INFO(MOD_CMD, "bottom");
    ok = color_reader.read_cube_bottom();
  }

  if (!ok) {
    //
    LOG_ERR(MOD_CMD, "failed");
    return false;
  }
  // After read
  String after = color_reader.get_cube_colors_string();
  ori.restore_cube_orientation();  //
  LOG_INFO(MOD_CMD, "after read orientation");
  LOG_VAR("orientation", ori.get_orientation_string().c_str());
  ori.print_orientation_string();
  //
  LOG_INFO(MOD_CMD, "after read cube colors");
  LOG_VAR("cube_colors", after.c_str());
  print_colors_detail("after read cube colors");

  static double arg = 0;
  return cmd_run(1, &arg);
}

bool cmd_getcolor_data(int argc, double *argv) {
  String all54 = color_reader.get_cube_colors_string();
  //
  LOG_INFO(MOD_CMD, "cube colors");
  LOG_VAR("cube_colors", all54.c_str());

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

// -------------------------------------------------------------------
// INFOSERVO <id> : print key control-table data for one servo
// -------------------------------------------------------------------
void print_info(uint8_t id) {
  if (!dxl.ping(id)) {
    //
    LOG_ERR(MOD_CMD, "servo not found");
    LOG_VAR("id", id);
    return;
  }

  bool _ok = servo_ok(id, true);  //
  LOG_INFO(MOD_CMD, "servo ok");
  LOG_VAR("id", id);
  LOG_VAR("ok", _ok);
  if (!_ok) {  //
    LOG_INFO(MOD_CMD, "servo resetting");
    LOG_VAR("id", id);
    reset_servo(id);  //
    LOG_INFO(MOD_CMD, "servo ok");
    LOG_VAR("id", id);
    LOG_VAR("ok", _ok);
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
  //
  LOG_INFO(MOD_CMD, "infoservo");
  LOG_VAR("id", id);
  // ~~~~~~~~~~~~~~~~~~~~~~~~  //
  LOG_INFO(MOD_CMD, "operating mode");
  LOG_VAR("op_mode", op);
  // ~~~~~~~~~~~~~~~~~~~~~~~~  //
  LOG_INFO(MOD_CMD, "drive mode");
  LOG_VAR("drive_mode", drv);
  LOG_VAR("profile_type", (drv & 0x01) ? "TIME" : "VELOCITY");
  // ~~~~~~~~~~~~~~~~~~~~~~~~  //
  LOG_INFO(MOD_CMD, "profile velocity");
  LOG_VAR("profile_vel", pv);
  LOG_VAR("rpm", rpm);
  LOG_VAR("ticks_per_sec", tps);
  // ~~~~~~~~~~~~~~~~~~~~~~~~  //
  LOG_INFO(MOD_CMD, "profile acceleration");
  LOG_VAR("profile_accel", pa);
  // ~~~~~~~~~~~~~~~~~~~~~~~~  //
  LOG_INFO(MOD_CMD, "position limits");
  LOG_VAR("min_ticks", minL);
  LOG_VAR("max_ticks", maxL);
  // ~~~~~~~~~~~~~~~~~~~~~~~~  //
  LOG_INFO(MOD_CMD, "position span");
  LOG_VAR("span_deg", spanDeg);
  // ~~~~~~~~~~~~~~~~~~~~~~~~  //
  LOG_INFO(MOD_CMD, "present position");
  LOG_VAR("pos_ticks", pos);
}