#include "cmd_parser.h"
#include "servo_move.h"
#include "servos.h"
#include "color_sensor.h"
#include "ori.h"
#include "color_reader.h"
#include "color_analyzer.h"
#include "run.h"

bool print_servo_info(uint8_t id);

extern double max_xmm;
extern double max_ymm;
extern double min_ymm;
extern double speed;
extern double max_speed;

extern double MAX_PER_CLAMP_GRIP;

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
    LOG_ERR(MOD_RUN, "error", "missing run argument");
    return false;
  }

  speed = 1.0;
  int run_no = (int)argv[0];

  set_torque_all_servos(true);
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
  LOG_ERR(MOD_RUN, "error", "invalid run argument");
  LOG_VAR("run_no", run_no);
  return false;
}

bool cmd_reboot_servos(int argc, double *argv) {
  LOG_INFO(MOD_SERVOS, "info", "reboot servos start");
  return reboot_all_servos();
}
bool cmd_set_servo_flag_servos_stop_all(int argc, double *argv) {
  LOG_INFO(MOD_SERVOS, "info", "set stop all start");
  set_flag_servos_stop_all();
  return true;
}
bool cmd_clear_flag_servos_stop_all(int argc, double *argv) {
  LOG_INFO(MOD_SERVOS, "info", "reset stop all start");
  return clear_flag_servos_stop_all();
}
bool cmd_check_servos(int argc, double *argv) {
  LOG_INFO(MOD_SERVOS, "info", "check all servos start");
  return check_all_servos_if_ok();
}

bool robot_move_callback(const String &mv) {
  LOG_INFO(MOD_ROBOTMOVE, "info", "robot move start");
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

  LOG_ERR(MOD_RUN, "error", "invalid robot move");
  LOG_VAR("move", mv.c_str());
  return false;
}

// ------------------------------------------------------------
// RUN_ZERO
// ------------------------------------------------------------
bool cmd_run_zero() {

  if (!dxl_ping_cached(ID_ARM1) || !dxl_ping_cached(ID_ARM2)) return false;
  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  kin.solve_x_y_from_a1_a2(a1_deg, a2_deg);

  double prev_speed = speed;

  // ============================================================
  // case 3 - wrist is exactly horiz
  if (isWristHoriz()) {
    // 0 - do nothing
    LOG_INFO(MOD_RUN, "run_zero_start", "wrist is horiz");
  }

  // ============================================================
  // case 4 - wrist is exactly vert
  else if (isWristVert()) {
    // 0 - do nothing
    LOG_INFO(MOD_RUN, "run_zero_start", "wrist is vert");
  }

  // ============================================================
  // case 1 - wrist is near horiz
  else if (isWristNearHoriz()) {
    LOG_INFO(MOD_RUN, "run_zero_start", "wrist is near horiz");
    // 1 - soften the base
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 0);
    // 2 - soften the gripper
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 0);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 0);
    // 3 - open the gripper to wide open
    max_speed = 0.15;
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 1);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 1);
    if (!isGripperOpen(G_WIDE_OPEN)) {
      if (!cmdMoveGripperPer(G_WIDE_OPEN)) return false;
    }
    // 4 - soften the gripper again
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 0);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 0);
    // 5 - move high
    max_speed = 0.35;
    if (!cmdMoveYmm(Y_UP)) return false;
    // 6 - fix the gripper again
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 1);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 1);
    if (!isGripperOpen(G_WIDE_OPEN))
      if (!cmdMoveGripperPer(G_WIDE_OPEN)) return false;
    // 7 - soften the wrist
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_WRIST, 0);
    if (!cmdMoveYmm(Y_UP)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    // 8 - fix the wrist
    max_speed = 0.35;
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_WRIST, 1);
    if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;
    // 9 - fix the base
    max_speed = 0.65;
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 1);
    if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) return false;
    speed = prev_speed;
    max_speed = 1.0;
    // 10 - regular run 0 from now
    set_torque_all_servos(true);

    // ============================================================
    // case 2 - wrist is near vert
  } else if (isWristNearVert()) {
    LOG_INFO(MOD_RUN, "run_zero_start", "wrist is near vert");
    // 1 - soften the base
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 0);
    // 2 - soften the gripper
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 0);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 0);
    // 3 - open the gripper to wide open
    max_speed = 0.15;
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 1);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 1);
    if (!isGripperOpen(G_WIDE_OPEN)) {
      if (!cmdMoveGripperPer(G_WIDE_OPEN)) return false;
    }
    // 4 - soften the gripper gain
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 0);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 0);
    // 5 - move to center
    max_speed = 0.35;
    if (!cmdMoveYmm(Y_CENTER)) return false;
    // 6 - fix the gripper again
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 1);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 1);
    if (!isGripperOpen(G_WIDE_OPEN))
      if (!cmdMoveGripperPer(G_WIDE_OPEN)) return false;
    // 7 - soften the wrist
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_WRIST, 0);
    if (!cmdMoveYmm(Y_UP)) return false;
    if (!cmdMoveXmm(X_CENTER)) return false;
    // 8 - fix the wrist
    max_speed = 0.35;
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_WRIST, 1);
    if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;
    // 9 - fix the base
    max_speed = 0.65;
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 1);
    if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) return false;
    speed = prev_speed;
    max_speed = 1.0;
    // 10 - regular run 0 from now
    set_torque_all_servos(true);
  }

  // ============================================================
  // case 5 - wrist is in between (fixable only by hand)
  else {
    LOG_ERR(MOD_RUN, "run_zero_failed", "wrist is not horiz or vert");
    set_torque_all_servos(true);
    flash_led_all_servos(6);
    // needs to be hand adjusted
    return false;
  }

  // ============================================================
  // normal run 0 to land y low

  // 1- open grip
  if (!isGripperOpen(G_WIDE_OPEN)) {
    if (!cmdMoveGripperPer(G_WIDE_OPEN)) return false;
  }
  // 2- move wrist horiz
  if (!isWristHoriz()) {
    if (!prepArmsForWristRotationGripOpen()) return false;
    if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;
  }
  // 3 - fix the base
  if (!prepBaseForRotation(B_LEFT)) return false;
  if (!prepBaseForRotation(B_RIGHT)) return false;
  if (!cmdMoveServoDeg(ID_BASE, B_CENTER)) return false;
  // 3 - move down
  if (!cmdMoveYmm(Y_DOWN)) return false;
  // 4 - soft close
  if (!cmdMoveGripperPer(G_SOFT_CLOSE)) return false;
  return true;
}

// ------------------------------------------------------------
// RUN_RIGHT_DOWN
// ------------------------------------------------------------
bool cmd_run_right_down() {
  if (!cmdMoveGripperPer(G_OPEN)) return false;

  if (!prepArmsForWristRotationGripOpen()) return false;
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

  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_CENTER + 3)) return false;
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

  if (!prepArmsForWristRotationGripOpen()) return false;
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
  LOG_INFO(MOD_RUN, "info", "ori orientation");
  LOG_VAR("orientation", s.c_str());

  String log = ori.get_move_log();  //
  LOG_INFO(MOD_RUN, "info", "ori move log");
  LOG_VAR("move_log", log.c_str());

  // print_colors_analyzer_detail();

  return true;
}

// ------------------------------------------------------------
// RESETORI
// ------------------------------------------------------------
bool cmd_clear_ori_data(int argc, double *argv) {
  ori.clear_orientation_data();
  ori.clear_move_log();  //
  LOG_INFO(MOD_RUN, "info", "ori reset");

  String s = ori.get_orientation_string();  //
  LOG_INFO(MOD_RUN, "info", "ori orientation");
  LOG_VAR("orientation", s.c_str());

  String log = ori.get_move_log();  //
  LOG_INFO(MOD_RUN, "info", "ori move log");
  LOG_VAR("move_log", log.c_str());

  return true;
}

bool cmd_restore_ori(int argc, double *argv) {
  return cmd_restore_ori_run();
}

bool cmd_restore_ori_run() {
  LOG_INFO(MOD_RUN, "restore ori based on moves history", ori.get_orientation_string());
  bool ok = ori.restore_cube_orientation();
  if (!ok) {
    LOG_ERR(MOD_RUN, "error", "failed to restore orientation");
    LOG_VAR("orientation", ori.get_orientation_string());
  }
  LOG_INFO(MOD_RUN, "info", "return to pos zero");

  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveYmm(Y_DOWN)) return false;
  if (!cmdMoveGripperPer(G_SOFT_CLOSE)) return false;

  // At this point ori_ is already identity.  // Only clear the move log.
  ori.clear_move_log();  //
  LOG_INFO(MOD_RUN, "info", "ori restored to identity");

  String s = ori.get_orientation_string();  //
  LOG_INFO(MOD_RUN, "info", "ori orientation");
  LOG_VAR("orientation", s.c_str());

  String log = ori.get_move_log();  //
  LOG_INFO(MOD_RUN, "info", "ori move log");
  LOG_VAR("move_log", log.c_str());
  return true;
}

bool cmd_move_xy(int argc, double *argv) {
  double goal_xmm = argv[0];
  if (goal_xmm < -max_xmm || goal_xmm > max_xmm) {
    //
    LOG_ERR(MOD_RUN, "error", "invalid x mm");
    LOG_VAR("goal_x_mm", goal_xmm);
    LOG_VAR("min_x_mm", -max_xmm);
    LOG_VAR("max_x_mm", max_xmm);
    return false;
  }
  double goal_ymm = argv[1];
  if (goal_ymm < min_ymm || goal_ymm > max_ymm) {
    //
    LOG_ERR(MOD_RUN, "error", "invalid y mm");
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
  if (!dxl_ping_cached(id)) return false;

  double goal_deg = (double)argv[1];

  if (goal_deg < -185.0 || goal_deg > 360.0) {
    //
    LOG_ERR(MOD_RUN, "error", "invalid servo deg");
    LOG_VAR("goal_deg", goal_deg);
    return false;
  }

  if (!cmdMoveServoDeg((uint8_t)id, goal_deg)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_move_ticks(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl_ping_cached(id)) return false;

  int goal_ticks = (int)argv[1];  // serial_printf_verbose("cmd_move_ticks: id=%d ticks=%d", id, goal_ticks);

  if (!safeSetGoalPosition(id, goal_ticks)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_move_per(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl_ping_cached(id)) return false;

  double goal_per = argv[1];
  if (goal_per < -15.0 || goal_per > 115.0) {

    //
    LOG_ERR(MOD_RUN, "error", "invalid servo percentage");
    LOG_VAR("goal_per", goal_per);
    return false;
  }

  double goal_deg = per2deg(id, goal_per);  // serial_printf_verbose("cmd_move_per: id=%d per=%d deg=%d", id, goal_per, goal_deg);

  if (!cmdMoveServoDeg((uint8_t)id, goal_deg)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_set_servo_min(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl_ping_cached(id)) return false;

  int t = (int)argv[1];

  if (auto *s = find_servo(id)) {
    // serial_printf_verbose("cmd_set_servo_min: id=%d ticks=%d", id, t);
    s->set_min_ticks(t);
    return true;
  }
  return false;
}

bool cmd_set_servo_max(int argc, double *argv) {
  int id = (int)argv[0];
  if (!dxl_ping_cached(id)) return false;

  int t = (int)argv[1];

  if (auto *s = find_servo(id)) {
    // serial_printf_verbose("cmd_set_servo_max: id=%d ticks=%d", id, t);
    s->set_max_ticks(t);
    return true;
  }
  return false;
}

bool cmd_move_y(int argc, double *argv) {
  double goal_mm = argv[0];
  if (goal_mm < min_ymm || goal_mm > max_ymm) {
    //
    LOG_ERR(MOD_RUN, "error", "invalid y mm");
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
    LOG_ERR(MOD_RUN, "error", "invalid x mm");
    LOG_VAR("goal_x_mm", goal_mm);
    return false;
  }
  // serial_printf_verbose("cmd_move_x: x=%.2fmm", goal_mm);
  if (!cmdMoveXmm(goal_mm)) return false;
  return true;
}

bool cmd_move_clamp(int argc, double *argv) {
  if (!dxl_ping_cached(ID_GRIP1) || !dxl_ping_cached(ID_GRIP2)) return false;
  // serial_printf_verbose("cmd_move_clamp");  // turn off torque off base before clamp
  dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 0);
  bool ok = cmdMoveGripperClamp();  // turn off torque on base after clamp
  dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 1);
  print_servo_status(ID_GRIP1);
  print_servo_status(ID_GRIP2);
  return ok;
}

bool cmd_move_gripper(int argc, double *argv) {
  if (!dxl_ping_cached(ID_GRIP1) || !dxl_ping_cached(ID_GRIP2)) return false;

  double goal_deg = argv[0];

  if (goal_deg < -5.0 || goal_deg > 115.0) {

    //
    LOG_ERR(MOD_RUN, "error", "invalid gripper percentage");
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
  if (!dxl_ping_cached(ID_WRIST) || !dxl_ping_cached(ID_ARM1) || !dxl_ping_cached(ID_ARM2)) return false;

  double goal_deg = argv[0];

  if (goal_deg < -95 || goal_deg > 95) {

    //
    LOG_ERR(MOD_RUN, "error", "invalid wrist degrees");
    LOG_VAR("goal_deg", goal_deg);
    return false;
  }
  // serial_printf_verbose("cmd_move_wrist: deg=%.2f", goal_deg);

  if (!cmdMoveWristDegVertical(goal_deg)) return false;
  // print_servo_status(ID_WRIST);
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
    LOG_INFO(MOD_COLORSENSOR, "info", "calling read color");
    LOG_VAR("iteration", i);
    char crrColor = read_color();
    LOG_INFO(MOD_COLORSENSOR, "info", "read_sensor");
    LOG_VAR("color", crrColor);
    delay(555);
  }

  return true;
}

bool prepArmsForWristRotationGripOpen() {
  if (!dxl_ping_cached(ID_ARM1) || !dxl_ping_cached(ID_ARM2)) return false;

  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  kin.solve_x_y_from_a1_a2(a1_deg, a2_deg);

  double y_mm = kin.getYmm();
  double x_mm = kin.getXmm();

  double target = Y_UP;

  bool y_ok = (y_mm > (target - 1)) && (y_mm < (target + 1));
  bool x_ok = (x_mm > (X_CENTER - 1)) && (x_mm < (X_CENTER + 1));

  //Serial.print("\nprepare arms y_mm=");
  //Serial.println(y_mm);

  // ✅ Always fix Y first and RETURN
  if (!y_ok) {
    return cmdMoveYmm(Y_CENTER) && cmdMoveXmm(X_CENTER);
  }

  // ✅ Only touch X once Y is correct
  if (!x_ok) {
    return cmdMoveXmm(X_CENTER);
  }
  return true;
}

bool prepArmsForWristRotationGripClosed() {
  if (!dxl_ping_cached(ID_ARM1) || !dxl_ping_cached(ID_ARM2)) return false;

  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  kin.solve_x_y_from_a1_a2(a1_deg, a2_deg);

  double y_mm = kin.getYmm();
  double x_mm = kin.getXmm();

  double target = Y_CENTER;

  bool y_ok = (y_mm > (target - 1)) && (y_mm < (target + 1));
  bool x_ok = (x_mm > (X_CENTER - 1)) && (x_mm < (X_CENTER + 1));

  //Serial.print("\nprepare arms y_mm=");
  //Serial.println(y_mm);

  // ✅ Always fix Y first and RETURN
  if (!y_ok) {
    return cmdMoveYmm(Y_CENTER) && cmdMoveXmm(X_CENTER);
  }

  // ✅ Only touch X once Y is correct
  if (!x_ok) {
    return cmdMoveXmm(X_CENTER);
  }
  return true;
}

bool isWristHoriz() {
  double goal_deg = kin.getWdeg_for_horizontal_right();
  double w_deg = getPos_deg(ID_WRIST);

  double diff = fabs(goal_deg - w_deg);
  if (diff < W_TOL) return true;

  goal_deg = kin.getWdeg_for_horizontal_left();
  diff = fabs(goal_deg - w_deg);
  return diff < W_TOL;
}
bool isWristVert() {
  double goal_deg = kin.getWdeg_for_vertical();
  double w_deg = getPos_deg(ID_WRIST);

  double diff = fabs(goal_deg - w_deg);
  return diff < W_TOL;
}
bool isWristNearHoriz() {
  double goal_deg = kin.getWdeg_for_horizontal_right();
  double w_deg = getPos_deg(ID_WRIST);

  double diff = fabs(goal_deg - w_deg);
  if (diff < 3 * W_TOL) return true;

  goal_deg = kin.getWdeg_for_horizontal_left();
  diff = fabs(goal_deg - w_deg);
  return diff < 3 * W_TOL;
}
bool isWristNearVert() {
  double goal_deg = kin.getWdeg_for_vertical();
  double w_deg = getPos_deg(ID_WRIST);

  double diff = fabs(goal_deg - w_deg);
  return diff < 3 * W_TOL;
}

bool prepBaseForRotation(double nextBaseMoveRelative) {
  if (nextBaseMoveRelative == B_CENTER) return true;
  // // serial_printf_verbose("***** start prep base for rotation %.2f", nextBaseMoveRelative);

  if (!cmdMoveGripperPer(G_WIDE_OPEN)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;

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

  if (!cmdMoveGripperPer(G_WIDE_OPEN)) return false;
  if (!prepArmsForWristRotationGripOpen()) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;
  if (!cmdMoveGripperClamp()) return false;
  if (!isYmmAbove(Y_ROTATE_BASE))
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
  if (!dxl_ping_cached(ID_BASE)) return false;
  // // serial_printf_verbose("***** start rotate base relative with %.2f", baseMoveRelative);  // // serial_printf_verbose("***** base before prepared pos is %.2f", getPos_deg(ID_BASE));

  if (baseMoveRelative == B_CENTER) return true;  // no move

  if (!gripperOn)
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
    if (!isYmmAbove(Y_ROTATE_BASE))
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
  if (!prepArmsForWristRotationGripOpen()) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;

  if (!cmdMoveServoPer(ID_GRIP2, G_ALIGN_LEFT)) return false;
  if (!cmdMoveServoDeg(ID_GRIP2, G_OPEN)) return false;
  if (!cmdMoveServoDeg(ID_GRIP1, G_ALIGN_RIGHT)) return false;
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
  max_speed = 0.35;
  if (!cmdMoveYmm(Y_ABOVE_DROP)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  max_speed = 0.20;
  if (!cmdMoveYmm(Y_DROP)) return false;
  if (!cmdMoveGripperPer(G_OPEN)) return false;
  max_speed = 1.0;
  return true;
}

bool liftCube() {
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_ABOVE_DROP)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  return true;
}

char crrColorChar = '.';

// face index: u r f d l b
static const char k_faces[] = { 'u', 'r', 'f', 'd', 'l', 'b' };

// [face][slot] → char
// slots indexed 1..6 (0 unused)
static char last_color[6][7];

static int face_to_index(char face) {
  face = tolower(face);
  for (int i = 0; i < 6; i++) {
    if (k_faces[i] == face) return i;
  }
  return -1;
}

bool is_valid_face(char face) {
  return face_to_index(face) >= 0;
}

bool is_valid_slot(int slot) {
  return slot >= 1 && slot <= 6;
}

void clear_last_color_reads() {
  for (int f = 0; f < 6; f++) {
    for (int s = 1; s <= 6; s++) {
      last_color[f][s] = '.';
    }
  }
}

char get_last_color_read(char face, int slot) {
  int fi = face_to_index(face);
  if (fi < 0 || !is_valid_slot(slot)) return '.';

  return last_color[fi][slot];
}

bool cmd_read_one_color(int argc, double *argv) {
  if (argc < 1) return false;

  double d_slot = (double)argv[0];
  int slot = (int)d_slot;

  char c = cmd_read_one_color_run(slot);
  LOG_INFO(MOD_COLORSENSOR, "info", "read_one_color");
  LOG_VAR("color", c);

  if (c == '.') return false;
  return true;
}

char cmd_read_one_color_run(int slot) {
  int prev_speed = speed;

  do {
    if (!cmdMoveGripperPer(G_WIDE_OPEN)) break;
    if (!isWristHoriz()) {
      if (!prepArmsForWristRotationGripOpen()) return false;
    }
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

    speed = prev_speed;
    return read_color();
  } while (false);

  speed = prev_speed;
  return '.';
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

    char crrColorChar = cmd_read_one_color_run(slot);

    face[pos] = crrColorChar;
    //
    LOG_INFO(MOD_COLORSENSOR, "info", "read slot");
    LOG_VAR("slot", pos);
    LOG_VAR("color", crrColorChar);
  }
  // build final string in normal 123456 order
  String faceColors = "";
  for (int i = 1; i <= 6; i++) {
    faceColors += face[i];
  }  //
  LOG_INFO(MOD_RUN, "info", "face colors");
  LOG_VAR("colors", faceColors.c_str());
  return true;
}

//---------------------------------------------------------------------------

void print_colors_analyzer_detail() {
  LOG_INFO(MOD_COLORCHECK, "info", "color analyzer data");
  bool valid_colors = color_analyzer.is_color_string_valid_bool();

  if (!valid_colors) {
    LOG_ERR(MOD_COLORCHECK, "error", "color string not valid");
    LOG_VAR("log", color_analyzer.get_string_check_log().c_str());
    if (color_analyzer.is_string_fixable_bool()) {
      String fixed;
      if (color_analyzer.try_fix_color_string(fixed)) {
        LOG_INFO(MOD_COLORCHECK, "info", "color string can be corrected");
        LOG_VAR("corrected looks like", fixed.c_str());
      } else {
        LOG_ERR(MOD_COLORCHECK, "error", "color string fix failed");
      }
    } else {
      LOG_ERR(MOD_COLORCHECK, "error", "color string not fixable");
    }
  } else {
    // show the stage//
    LOG_INFO(MOD_COLORCHECK, "info", "color string is valid");
    for (int s = 0; s < color_analyzer.get_stage_count(); s++) {
      String state = "(...)";
      if (color_analyzer.is_stage_done_bool(s)) state = "(done)";
      else if (color_analyzer.is_stage_partial_bool(s)) state = "(partial)";
      LOG_INFO(MOD_COLORCHECK, color_analyzer.get_stage_name(s), state);
    }
  }
}

// Your callback implementation
char read_one_color_cb(int slot) {
  char c = cmd_read_one_color_run(slot);
  return c;
}

bool cmd_read_cube_colors(const String &mode_in) {
  String mode = mode_in;
  mode.toLowerCase();

  bool do_bottom = false;
  bool do_full = false;
  bool do_solved = false;
  bool do_centers = false;

  if (mode == "all") {
    do_full = true;
  } else if (mode == "bottom") {
    do_bottom = true;
  } else if (mode == "centers") {
    do_centers = true;
  } else if (mode == "solved") {
    do_solved = true;
  } else {
    // ~~~~~~~~~~~~~~~~~~~~~~~~
    //
    LOG_ERR(MOD_RUN, "error", "invalid mode");
    LOG_VAR("mode", mode.c_str());
    return false;
  }
  // needed to restore after read
  ori.clear_orientation_data();
  ori.clear_move_log();
  bool ok = false;

  if (do_full) {  //
    LOG_INFO(MOD_RUN, "info", "full");
    color_reader.clear_colors();
    ok = color_reader.read_cube_full();
    if (!ok) {
      LOG_ERR(MOD_RUN, "error", "read full cube failed");
      color_reader.clear_colors();
    }
  } else if (do_solved) {  //
    LOG_INFO(MOD_RUN, "info", "solved");
    color_reader.clear_colors();
    color_reader.fill_solved_cube();
    ok = true;
  } else if (do_bottom) {  //
    LOG_INFO(MOD_RUN, "info", "bottom");
    // no clear colors, update in place
    ok = color_reader.read_cube_bottom();
    if (ok) {
      if (!color_reader.fill_top_2_layers_as_solved()) {
        LOG_ERR(MOD_RUN, "error", "update top 2 layers failed");
        color_reader.clear_colors();
        ok = false;
      }
    } else {
      LOG_ERR(MOD_RUN, "error", "read bottom cube failed");
      color_reader.clear_colors();
    }
  } else if (do_centers) {  //
    LOG_INFO(MOD_RUN, "info", "centers");
    // no clear colors, update in place
    ok = color_reader.read_cube_centers();
    if (!ok) {
      LOG_ERR(MOD_RUN, "error", "read cube centers failed");
      color_reader.clear_colors();
    }
  }
  // restore ori based on moves history
  ori.clear_orientation_data();
  ori.clear_move_log();
  if (ok) {
    if (!ori.restore_cube_orientation()) {
      LOG_ERR(MOD_RUN, "error", "could not restore ori");
      color_reader.clear_colors();
      ok = false;
    }
  }
  if (ok) {
    if (!update_ori_from_color_54(color_reader.get_justread_color_string_54())) {
      LOG_ERR(MOD_RUN, "error", "could not update ori from colors");
      color_reader.clear_colors();
      ok = false;
    }
  }
  if (!ok) {
    LOG_ERR(MOD_RUN, "error", "failed");
    return false;
  }
  ok = color_analyzer.set_colors(colors_just_read);
  color_reader.clear_colors();
  if (!ok) {
    LOG_ERR(MOD_RUN, "color analyzer set colors failed", colors_just_read);
    color_analyzer.clear_colors();
    return false;
  }

  // After read
  LOG_INFO(MOD_RUN, "color_reader_should_be_clear_string_54", color_reader.get_justread_color_string_54());
  LOG_INFO(MOD_RUN, "color_analyze_color_string_54", color_analyzer.get_standard_color_string_54());
  // LOG_INFO(MOD_RUN, "color_analyze_string_faces", color_analyzer.get_color_string_faces());
  LOG_INFO(MOD_RUN, "color_analyzer_is_valid", color_analyzer.is_color_string_valid_bool());
  LOG_INFO(MOD_RUN, "color_analyzer_is_fixable", color_analyzer.is_string_fixable_bool());
  LOG_INFO(MOD_RUN, "ori_orientation", ori.get_orientation_string());
  return true;
}

bool cmd_getcolor_data(int argc, double *argv) {
  LOG_INFO(MOD_RUN, "cube_color_string_54", color_analyzer.get_standard_color_string_54().c_str());
  LOG_INFO(MOD_RUN, "orientation", ori.get_orientation_string());
  print_colors_analyzer_detail();
  return true;
}

bool cmd_read_servo(int argc, double *argv) {
  print_servo_status((argc > 0) ? (int)argv[0] : 0);
  return true;
}

bool cmd_servo_info(int argc, double *argv) {
  print_servo_info((uint8_t)argv[0]);
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

bool cmd_detect_cube(int argc, double *argv) {
  if (!dxl_ping_cached(ID_GRIP1) || !dxl_ping_cached(ID_GRIP2))
    return false;

  // bring the grip in position
  LOG_INFO(MOD_SERVO_MOVE, "bring grip in position", Y_CENTER);
  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!prepArmsForWristRotationGripOpen()) return false;
  if (!cmdMoveWristDegVertical(W_HORIZ_RIGHT)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_CENTER)) return false;

  const uint16_t PWM_REG = 124;
  const double STEP = 0.8;

  bool touched1 = false;
  bool touched2 = false;

  if (!cmdMoveGripperPer(G_NEARCLOSE)) return false;
  double per1 = getPos_per(ID_GRIP1);
  double per2 = getPos_per(ID_GRIP2);

  LOG_INFO(MOD_SERVO_MOVE, "test grip from grip1 per", per1);
  LOG_VAR("grip 2 per", per2);
  for (int step = 0; step < 133; step++) {

    dxl.setGoalPosition(ID_GRIP1, per2ticks(ID_GRIP1, per1));
    dxl.setGoalPosition(ID_GRIP2, per2ticks(ID_GRIP2, per2));

    unsigned long until = millis() + 10;
    while (millis() < until) {
      int16_t pwm1 = readReg16(ID_GRIP1, PWM_REG);
      int16_t pwm2 = readReg16(ID_GRIP2, PWM_REG);

      if (isGripAtTouch(pwm1)) touched1 = true;
      if (isGripAtTouch(pwm2)) touched2 = true;

      if (touched1 && touched2) {
        if (touched1) {
          LOG_INFO(MOD_SERVO_MOVE, "grip1 touched pmw1", pwm1);
          LOG_VAR("grip1 pos", getPos_per(ID_GRIP1));
        }
        if (touched2) {
          LOG_INFO(MOD_SERVO_MOVE, "grip2 touched pmw2", pwm2);
          LOG_VAR("grip2 pos", getPos_per(ID_GRIP2));
        }
        break;
      } else if (touched1 || touched2) {
        if (touched1) {
          LOG_INFO(MOD_SERVO_MOVE, "grip1 touched pmw1", pwm1);
          LOG_VAR("grip1 pos", getPos_per(ID_GRIP1));
        }
        if (touched2) {
          LOG_INFO(MOD_SERVO_MOVE, "grip2 touched pmw2", pwm2);
          LOG_VAR("grip2 pos", getPos_per(ID_GRIP2));
        }
        break;
      }
      delay(2);
    }
    delay(25);

    // If either side touches early → cube IS present
    if (touched1 || touched2) {
      if (!cmdMoveGripperPer(G_OPEN)) return false;
      if (!cmdMoveXmm(X_CENTER)) return false;
      if (!cmdMoveYmm(Y_DOWN)) return false;

      if (touched1) LOG_INFO(MOD_SERVO_MOVE, "cube_detection_info", "(touch grip1)");
      if (touched2) LOG_INFO(MOD_SERVO_MOVE, "cube_detection_info", "(touch grip2)");
      LOG_INFO(MOD_SERVO_MOVE, "cube_detected", "yes");
      return true;
    }

    // Advance closing
    per1 += STEP;
    per2 += STEP;

    if (per1 > MAX_PER_CLAMP_GRIP) per1 = MAX_PER_CLAMP_GRIP;
    if (per2 > MAX_PER_CLAMP_GRIP) per2 = MAX_PER_CLAMP_GRIP;

    // Fully closed without load → no cube
    if (per1 >= MAX_PER_CLAMP_GRIP && per2 >= MAX_PER_CLAMP_GRIP) {
      if (!cmdMoveGripperPer(G_OPEN)) return false;
      if (!cmdMoveXmm(X_CENTER)) return false;
      if (!cmdMoveYmm(Y_DOWN)) return false;
      LOG_INFO(MOD_SERVO_MOVE, "cube_detection_info", "(free close)");
      LOG_INFO(MOD_SERVO_MOVE, "cube_detected", "no");
      return false;
    }
  }

  // Safety fallback: no touch detected
  if (!cmdMoveGripperPer(G_OPEN)) return false;
  if (!cmdMoveXmm(X_CENTER)) return false;
  if (!cmdMoveYmm(Y_DOWN)) return false;
  LOG_INFO(MOD_SERVO_MOVE, "cube_detection_info", "(timeout)");
  LOG_INFO(MOD_SERVO_MOVE, "cube_detected", "no");
  return false;
}

// -------------------------------------------------------------------
// INFOSERVO <id> : print key control-table data for one servo
// -------------------------------------------------------------------
bool print_servo_info(uint8_t id) {
  if (!dxl_ping_cached(id)) {
    //
    LOG_ERR(MOD_RUN, "error", "servo not found");
    LOG_VAR("id", id);
    return false;
  }
  LOG_INFO(MOD_RUN, "info", "checking_if_servo_is_ok");
  bool _ok = servo_ok(id);  //
  LOG_INFO(MOD_RUN, "info", "servo ok");
  LOG_VAR("servo_id", id);
  LOG_VAR("servo_name", id2name(id));
  LOG_VAR("ok", _ok);

  if (!_ok) {  //
    LOG_ERR(MOD_RUN, "error", "servo_not_ok");
    LOG_VAR("id", id);
    LOG_VAR("servo_name", id2name(id));
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
  LOG_INFO(MOD_RUN, "info", "infoservo");
  LOG_VAR("id", id);

  LOG_INFO(MOD_RUN, "info", "operating mode");
  LOG_VAR("op_mode", op);

  LOG_INFO(MOD_RUN, "info", "drive mode");
  LOG_VAR("drive_mode", drv);
  LOG_VAR("profile_type", (drv & 0x01) ? "TIME" : "VELOCITY");

  LOG_INFO(MOD_RUN, "info", "profile velocity");
  LOG_VAR("profile_vel", pv);
  LOG_VAR("rpm", rpm);
  LOG_VAR("ticks_per_sec", tps);

  LOG_INFO(MOD_RUN, "info", "profile acceleration");
  LOG_VAR("profile_accel", pa);

  LOG_INFO(MOD_RUN, "info", "position limits");
  LOG_VAR("min_ticks", minL);
  LOG_VAR("max_ticks", maxL);

  LOG_INFO(MOD_RUN, "info", "position span");
  LOG_VAR("span_deg", spanDeg);

  LOG_INFO(MOD_RUN, "info", "present position");
  LOG_VAR("pos_ticks", pos);

  return true;
}

bool cmd_check_ori(int argc, double *argv) {
  return bool cmd_check_ori_run();
}

bool cmd_check_ori_run() {

  char robot_face_front = ori.cube_face_to_robot_face('f');
  if (!is_valid_face(robot_face_front)) {
    LOG_ERR(MOD_RUN, "ori does not have orientation", robot_face_front);
    return false;
  }

  char just_read_robot_color_front = cmd_read_one_color_run(6);
  if (!is_valid_color(just_read_robot_color_front)) {
    LOG_ERR(MOD_RUN, "invalid_robot_center_color", just_read_robot_color_front);
    return false;
  }
  char just_read_robot_face_front = color_to_face(just_read_robot_color_front);
  if (just_read_robot_face_front == robot_face_front) {
    LOG_INFO(MOD_CMD, "ori data matched color read on front", just_read_robot_face_front);
    return true;
  }
  LOG_ERR(MOD_CMD, "ori data matched does not match color read on front", just_read_robot_face_front);
  LOG_VAR("ori expects", robot_face_front);
  return false;
}

bool cmd_detect_ori(int argc, double *argv) {
  LOG_INFO(MOD_RUN, "info", "color read for centers will update ori")
  if (!color_reader.read_cube_centers()) return false;
  return true;
}