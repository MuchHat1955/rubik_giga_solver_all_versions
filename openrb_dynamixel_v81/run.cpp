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
  LOG_INFO(MOD_ROBOTMOVE, "info", "robot_move_start");
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

  // sanity check: both arm servos must respond
  RUN_PING(ID_ARM1);
  RUN_PING(ID_ARM2);

  // compute current XY from joint angles
  double a1_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double a2_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  kin.solve_x_y_from_a1_a2(a1_deg, a2_deg);

  double prev_speed = speed;

  // ============================================================
  // case 3 - wrist is exactly horiz
  if (isWristHoriz()) {
    // do nothing
    RUN_CMD(cmdMoveXmm(X_CENTER), "center x");
    LOG_INFO(MOD_RUN, "run_zero_start", "wrist is horiz");
  }

  // ============================================================
  // case 4 - wrist is exactly vert
  else if (isWristVert()) {
    // do nothing
    RUN_CMD(cmdMoveXmm(X_CENTER), "center x");
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

    // 3 - open the gripper wide
    max_speed = 0.15;
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 1);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 1);
    if (!isGripperOpen(G_WIDE_OPEN)) {
      RUN_CMD(cmdMoveGripperPer(G_WIDE_OPEN), "open gripper wide");
    }

    // 4 - soften the gripper again
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 0);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 0);

    // 5 - move high
    max_speed = 0.35;
    RUN_CMD(cmdMoveYmm(Y_UP), "move y up");

    // 6 - re-enable gripper torque and re-open
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 1);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 1);
    if (!isGripperOpen(G_WIDE_OPEN)) {
      RUN_CMD(cmdMoveGripperPer(G_WIDE_OPEN), "re-open gripper wide");
    }

    // 7 - soften the wrist
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_WRIST, 0);
    RUN_CMD(cmdMoveYmm(Y_UP), "ensure y up");
    RUN_CMD(cmdMoveXmm(X_CENTER), "center x");

    // 8 - fix the wrist
    max_speed = 0.35;
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_WRIST, 1);
    RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "set wrist horizontal");

    // 9 - fix the base
    max_speed = 0.65;
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 1);
    RUN_CMD(cmdMoveServoDeg(ID_BASE, B_CENTER), "center base");

    // restore speed limits
    speed = prev_speed;
    max_speed = 1.0;

    // 10 - normal torque from now on
    set_torque_all_servos(true);
  }

  // ============================================================
  // case 2 - wrist is near vert
  else if (isWristNearVert()) {
    LOG_INFO(MOD_RUN, "run_zero_start", "wrist is near vert");

    // 1 - soften the base
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 0);

    // 2 - soften the gripper
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 0);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 0);

    // 3 - open the gripper wide
    max_speed = 0.15;
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 1);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 1);
    if (!isGripperOpen(G_WIDE_OPEN)) {
      RUN_CMD(cmdMoveGripperPer(G_WIDE_OPEN), "open gripper wide");
    }

    // 4 - soften the gripper again
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 0);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 0);

    // 5 - move to center
    max_speed = 0.35;
    RUN_CMD(cmdMoveYmm(Y_CENTER), "move y to center");

    // 6 - re-enable gripper torque and re-open
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP1, 1);
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_GRIP2, 1);
    if (!isGripperOpen(G_WIDE_OPEN)) {
      RUN_CMD(cmdMoveGripperPer(G_WIDE_OPEN), "re-open gripper wide");
    }

    // 7 - soften the wrist
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_WRIST, 0);
    RUN_CMD(cmdMoveYmm(Y_UP), "move y up");
    RUN_CMD(cmdMoveXmm(X_CENTER), "center x");

    // 8 - fix the wrist
    max_speed = 0.35;
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_WRIST, 1);
    RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "set wrist horizontal");

    // 9 - fix the base
    max_speed = 0.65;
    dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 1);
    RUN_CMD(cmdMoveServoDeg(ID_BASE, B_CENTER), "center base");

    // restore speed limits
    speed = prev_speed;
    max_speed = 1.0;

    // 10 - normal torque from now on
    set_torque_all_servos(true);
  }

  // ============================================================
  // case 5 - wrist is in between (manual intervention required)
  else {
    LOG_ERR(MOD_RUN, "run_zero_failed", "wrist is not horiz or vert");
    set_torque_all_servos(true);
    flash_led_all_servos(6);
    return false;
  }

  // ============================================================
  // normal run 0 to land y low

  // 1 - open grip if needed
  if (!isGripperOpen(G_WIDE_OPEN)) {
    RUN_CMD(cmdMoveGripperPer(G_WIDE_OPEN), "open gripper wide");
  }

  // 2 - ensure wrist horizontal
  if (!isWristHoriz()) {
    RUN_CMD(cmdMoveYmm(Y_CENTER), "move y to center");
    RUN_CMD(cmdMoveXmm(X_CENTER), "center x");
    RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "set wrist horizontal");
  }

  // 4 - move down
  RUN_CMD(cmdMoveYmm(Y_DOWN), "move y down");

  // 5 - soft close
  RUN_CMD(cmdMoveGripperPer(G_SOFT_CLOSE), "soft close gripper");

  return true;
}

// ------------------------------------------------------------
// RUN_RIGHT_DOWN
// ------------------------------------------------------------
bool cmd_run_right_down() {

  // open grip before positioning
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");

  // move to center and align wrist for pickup
  RUN_CMD(cmdMoveYmm(Y_CENTER), "move y to center");
  RUN_CMD(cmdMoveXmm(X_CENTER), "move x to center");
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "set wrist horizontal right");

  // re-center and grip cube
  RUN_CMD(cmdMoveYmm(Y_CENTER), "move y to center");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveGripperClamp(), "clamp gripper");

  // lift cube and move up for rotation
  RUN_CMD(liftCube(), "lift cube");
  RUN_CMD(cmdMoveYmm(Y_UP), "move to y up");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveServoDeg(ID_BASE, B_CENTER), "reset base to center");

  // re-position and rotate wrist vertical for right-down move
  RUN_CMD(cmdMoveYmm(Y_UP), "ensure y at up position");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveWristDegVertical(W_VERT), "rotate wrist vertical");

  // lower cube and release
  RUN_CMD(lowerCube(), "lower cube");
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");

  // restore position and wrist orientation
  RUN_CMD(cmdMoveYmm(Y_CENTER + 3), "move y above center (post-release)");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "reset wrist horizontal right");

  return true;
}

// ------------------------------------------------------------
// RUN_LEFT_DOWN
// ------------------------------------------------------------
bool cmd_run_left_down() {

  // open grip before positioning
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");

  // move to center and align vertically for pickup
  RUN_CMD(cmdMoveXmm(X_CENTER), "move x to center");
  RUN_CMD(cmdMoveYmm(Y_CENTER + 3), "move y above center");
  RUN_CMD(cmdMoveXmm(X_CENTER), "move x to center");
  RUN_CMD(cmdMoveWristDegVertical(W_VERT), "set wrist vertical");

  // adjust to center and grip cube
  RUN_CMD(cmdMoveYmm(Y_CENTER), "move y to center");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveGripperClamp(), "clamp gripper");

  // lift cube and move up for rotation
  RUN_CMD(liftCube(), "lift cube");
  RUN_CMD(cmdMoveYmm(Y_UP), "move to y up");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveServoDeg(ID_BASE, B_CENTER), "reset base to center");

  // re-position and rotate wrist to left-down orientation
  RUN_CMD(cmdMoveYmm(Y_UP), "ensure y at up position");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "rotate wrist horizontal right");

  // lower cube and release
  RUN_CMD(lowerCube(), "lower cube");
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");

  // restore position and wrist orientation
  RUN_CMD(cmdMoveYmm(Y_CENTER + 3), "move y above center (post-release)");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "reset wrist horizontal right");

  return true;
}

// ------------------------------------------------------------
// RUN_TOP_DOWN
// ------------------------------------------------------------
bool cmd_run_top_down() {

  // open grip
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");

  // move to center and reset the wrist horiz
  RUN_CMD(cmdMoveXmm(X_CENTER), "move x to center");
  RUN_CMD(cmdMoveYmm(Y_CENTER + 3), "move y above center");
  RUN_CMD(cmdMoveXmm(X_CENTER), "move x to center");
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_LEFT), "move wrist 180 ahead of grip");

  // radjust to center and grip cube
  RUN_CMD(cmdMoveYmm(Y_CENTER), "re-set y");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveGripperClamp(), "clamp gripper");

  // move up ahead of rotate
  RUN_CMD(liftCube(), "lift cube");
  RUN_CMD(cmdMoveYmm(Y_UP), "move to y up");
  RUN_CMD(cmdMoveServoDeg(ID_BASE, B_CENTER), "reset base for future moves");  // reset the base just in case

  // re-adjust to center than do the final rotate
  RUN_CMD(cmdMoveYmm(Y_UP), "move to y up");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x for rotate");
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "rotate wrist back 180 to rotate the cube down");

  // lower cube and open to drop in place
  RUN_CMD(lowerCube(), "lower cube");
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");

  // fix the gripper back to original horiz
  RUN_CMD(cmdMoveYmm(Y_CENTER + 3), "move y above center (post-release)");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "reset wrist horizontal right");

  return true;
}


// ------------------------------------------------------------
// RUN_BACK_DOWN
// ------------------------------------------------------------
bool cmd_run_back_down() {

  // open grip before reposition
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");

  // rotate base to the left to prepare back-down move
  RUN_CMD(rotateBaseRelative(B_LEFT, false), "rotate base left");

  // move to center and set wrist horizontal
  RUN_CMD(cmdMoveYmm(Y_CENTER), "move y to center");
  RUN_CMD(cmdMoveXmm(X_CENTER), "move x to center");
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "set wrist horizontal right");

  // re-center and grip cube
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveGripperClamp(), "clamp gripper");

  // lift cube and move up for rotation
  RUN_CMD(liftCube(), "lift cube");
  RUN_CMD(cmdMoveYmm(Y_UP), "move to y up");
  RUN_CMD(cmdMoveServoDeg(ID_BASE, B_CENTER), "reset base to center");

  // re-center before vertical wrist move
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");

  // rotate wrist vertical to place cube back-down
  RUN_CMD(cmdMoveWristDegVertical(W_VERT), "rotate wrist vertical");

  // lower cube and release
  RUN_CMD(lowerCube(), "lower cube");
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");

  // restore position and wrist orientation
  RUN_CMD(cmdMoveYmm(Y_CENTER + 3), "move y above center (post-release)");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "reset wrist horizontal right");

  return true;
}

// ------------------------------------------------------------
// RUN_DOWN_RIGHT / LEFT / BACK
// ------------------------------------------------------------
bool cmd_run_down_layer(int run_no) {

  // open grip before positioning
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");

  // move to mid height and align gripper
  RUN_CMD(cmdMoveYmm(Y_MID), "move y to mid");
  RUN_CMD(cmdMoveXmm(X_CENTER), "move x to center");
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "set wrist horizontal right");

  // re-adjust and grip cube
  RUN_CMD(cmdMoveYmm(Y_MID), "re-set y to mid");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveGripperClamp(), "clamp gripper");

  // rotate base depending on target side
  if (run_no == RUN_DOWN_RIGHT)
    RUN_CMD(rotateBaseRelative(B_RIGHT, true), "rotate base right (down layer)");

  if (run_no == RUN_DOWN_LEFT)
    RUN_CMD(rotateBaseRelative(B_LEFT, true), "rotate base left (down layer)");

  if (run_no == RUN_DOWN_BACK)
    RUN_CMD(rotateBaseRelative(B_BACK, true), "rotate base back (down layer)");

  // release and return to center
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");
  RUN_CMD(cmdMoveYmm(Y_CENTER), "move y to center");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");

  return true;
}

// ------------------------------------------------------------
// RUN_CUBE_*
// ------------------------------------------------------------
bool cmd_run_cube_right() {
  return rotateBaseRelative(B_RIGHT, false);
}
bool cmd_run_cube_left() {
  return rotateBaseRelative(B_LEFT, false);
}
bool cmd_run_cube_back() {
  return rotateBaseRelative(B_BACK, false);
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
  RUN_PING(id);

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
  RUN_PING(id);

  int goal_ticks = (int)argv[1];  // serial_printf_verbose("cmd_move_ticks: id=%d ticks=%d", id, goal_ticks);

  if (!safeSetGoalPosition(id, goal_ticks)) return false;
  print_servo_status(id);
  return true;
}

bool cmd_move_per(int argc, double *argv) {
  int id = (int)argv[0];
  RUN_PING(id);

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
  RUN_PING(id);

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
  RUN_PING(id);

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
  RUN_PING(ID_GRIP1);
  RUN_PING(ID_GRIP2);

  dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 0);
  bool ok = cmdMoveGripperClamp();  // turn off torque on base after clamp
  dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, ID_BASE, 1);
  print_servo_status(ID_GRIP1);
  print_servo_status(ID_GRIP2);
  return ok;
}

bool cmd_move_gripper(int argc, double *argv) {
  RUN_PING(ID_GRIP1);
  RUN_PING(ID_GRIP2);

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
  RUN_PING(ID_WRIST);
  RUN_PING(ID_ARM1);
  RUN_PING(ID_ARM2);

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

//TODO run 0 messes the ori

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
bool prepBaseForRotation(bool withCube) {
  if (withCube) {
    RUN_CMD(cmdMoveXmm(X_CENTER), "center");
    return true;
  }
  RUN_CMD(cmdMoveGripperPer(G_WIDE_OPEN), "grip wide open");
  if (!isYmmAbove(Y_ROTATE_BASE)) {
    RUN_CMD(cmdMoveYmm(Y_ROTATE_BASE), "y up");
  }
  RUN_CMD(cmdMoveXmm(X_CENTER), "center");
  return true;
}

/*
#define B_RIGHT 90
#define B_LEFT -90
#define B_BACK -180
*/

#define I_CENTER 0
#define I_RIGHT 1
#define I_LEFT -1
#define I_BACK -2

int basePos_deg2i(double d_pose) {
  double d_i_pose = d_pose / 90.0;
  if (d_i_pose > 0) d_i_pose += 0.5;
  else d_i_pose -= 0.5;
  return (int)d_i_pose;
}
String basePos_i2name(int iPose) {
  if (iPose == I_CENTER) return "center";
  if (iPose == I_RIGHT) return "right";
  if (iPose == I_LEFT) return "left";
  if (iPose == I_BACK) return "back";
  return "err";
}
double basePos_i2deg(int i_pose) {
  double d_i_pose = (double)i_pose * 90.0;
  return d_i_pose;
}

// below are relative moves
bool rotateBaseRelative(double base_rel_deg, bool gripperOn) {
  RUN_PING(ID_BASE);
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "wrist horiz");
  RUN_CMD(prepBaseForRotation(gripperOn), "prep base");
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "wrist horiz");

  double base_crr_deg = getPos_deg(ID_BASE);
  int base_crr_i = basePos_deg2i(base_crr_deg);
  int base_rel_i = basePos_deg2i(base_rel_deg);
  LOG_INFO(MOD_RUN, "base ->start", basePos_i2name(base_crr_i));
  LOG_VAR("->rel move", basePos_i2name(base_rel_i));

  /*
  #define I_CENTER 0
  #define I_RIGHT 1
  #define I_LEFT -1
  #define I_BACK -2
*/

  int base_goal_i = base_crr_i + base_rel_i;  // between -4 and 3

  if (base_goal_i == I_BACK - 1) base_goal_i = I_RIGHT;
  if (base_goal_i <= I_BACK - 2) base_goal_i = I_CENTER;

  if (base_goal_i == I_RIGHT + 1) base_goal_i = I_BACK;
  if (base_goal_i >= I_RIGHT + 2) base_goal_i = I_LEFT;

  double base_goal_deg = basePos_i2deg(base_goal_i);

  LOG_VAR("->goal", basePos_i2name(base_goal_i));

  double d_err = fabs(base_crr_deg - base_goal_deg);
  if (d_err < B_TOL) {
    LOG_INFO(MOD_RUN, "base err", d_err);
    return true;
  }

  if (gripperOn) {
    double adjFw = 0;
    double adjBk = 0;
    if (base_goal_deg > base_crr_deg) {
      adjFw = B_ERR + 1;
      adjBk = B_ERR;
    } else {
      adjFw = -B_ERR - 1;
      adjBk = -B_ERR;
    }
    // move past the target and the final move after will reset it
    RUN_CMD(cmdMoveServoDeg(ID_BASE, base_goal_deg + adjFw), "base to goal + adj fw");
    RUN_CMD(cmdMoveServoDeg(ID_BASE, base_goal_deg - adjBk), "base to goal - adj bk");
  }
  RUN_CMD(cmdMoveServoDeg(ID_BASE, base_goal_deg), "base to goal");
  return true;
}

bool alignCube() {

  // open gripper and move to center
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");
  RUN_CMD(cmdMoveYmm(Y_CENTER), "move y to center");
  RUN_CMD(cmdMoveXmm(X_CENTER), "move x to center");
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "set wrist horizontal right");

  // align individual gripper fingers
  RUN_CMD(cmdMoveServoPer(ID_GRIP2, G_ALIGN_LEFT), "align left gripper finger");
  RUN_CMD(cmdMoveServoDeg(ID_GRIP2, G_OPEN), "open left gripper finger");
  RUN_CMD(cmdMoveServoDeg(ID_GRIP1, G_ALIGN_RIGHT), "align right gripper finger");
  RUN_CMD(cmdMoveServoDeg(ID_GRIP1, G_OPEN), "open right gripper finger");

  // ensure both grippers are fully open
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");

  // move to alignment height and clamp
  RUN_CMD(cmdMoveYmm(Y_ALIGN), "move y to align height");
  RUN_CMD(cmdMoveGripperClamp(), "clamp gripper for alignment");

  // release and restore center position
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "release gripper");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");
  RUN_CMD(cmdMoveYmm(Y_CENTER), "move y to center");
  RUN_CMD(cmdMoveXmm(X_CENTER), "final re-center x");

  return true;
}

bool lowerCube() {

  // ensure x is centered before descent
  RUN_CMD(cmdMoveXmm(X_CENTER), "center x before lowering");
  // slow approach to above-drop height
  max_speed = 0.35;
  RUN_CMD(cmdMoveYmm(Y_ABOVE_DROP), "move y to above drop");
  // re-center x to correct any deflection
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x before final drop");
  RUN_CMD(cmdMoveXmm(X_CENTER), "final x re-center before drop");
  // very slow final descent
  max_speed = 0.20;
  RUN_CMD(cmdMoveYmm(Y_DROP), "move y to drop height");
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper to release cube");
  // restore speed
  max_speed = 1.0;
  return true;
}

bool liftCube() {

  RUN_CMD(cmdMoveXmm(X_CENTER), "center x before lift");
  RUN_CMD(cmdMoveYmm(Y_ABOVE_DROP), "move y to above drop");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x after lift");

  return true;
}


char crrColorChar = '.';

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
      if (!cmdMoveYmm(Y_CENTER)) return false;
      if (!cmdMoveXmm(X_CENTER)) return false;
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
  bool ok = false;

  if (do_full) {  //
    LOG_INFO(MOD_RUN, "info", "full");
    // the function below handles clearing etc
    ok = color_reader.read_cube_full();
    if (!ok) {
      LOG_ERR(MOD_RUN, "error", "read full cube failed");
    }
  } else if (do_solved) {  //
    LOG_INFO(MOD_RUN, "info", "solved");
    // the function below handles clearing etc
    ok = color_reader.read_cube_solved();
    if (!ok) {
      LOG_ERR(MOD_RUN, "error", "read solved cube failed");
    }
  } else if (do_bottom) {  //
    LOG_INFO(MOD_RUN, "info", "bottom");
    // the function below handles clearing etc
    ok = color_reader.read_cube_bottom();
    if (!ok) {
      LOG_ERR(MOD_RUN, "error", "read bottom cube failed");
    }
  } else if (do_centers) {  //
    LOG_INFO(MOD_RUN, "info", "f and r centers");
    // the function below handles clearing etc
    ok = color_reader.read_cube_f_and_r_centers();
    if (!ok) {
      LOG_ERR(MOD_RUN, "error", "read cube f and r centers failed");
    }
  }

  String colors_just_read = color_reader.get_justread_color_string_54();
  LOG_INFO(MOD_RUN, "just_read_string_54", colors_just_read);
  // use colors to set the orientation to match colors
  // lfr... should be set in colors by the robot pos
  if (ok) {
    LOG_INFO(MOD_RUN, "updating ori from color_string_54", colors_just_read);
    LOG_INFO(MOD_RUN, "before update", ori.get_orientation_string());
    if (!update_ori_from_color_reader_54(colors_just_read)) {
      LOG_ERR(MOD_RUN, "error", "could not update ori from colors");
      // color_reader.clear_color_reader();
      ok = false;
    }
    LOG_INFO(MOD_RUN, "after update", ori.get_orientation_string());
  }
  // do not set the final colors in the analyzer if it failed
  if (!ok) {
    LOG_ERR(MOD_RUN, "error", "failed");
    return false;
  }
  if (!do_centers) {
    // set the colors and clear the reader
    ok = color_analyzer.set_colors(colors_just_read);
    // clear reader after done

    if (!ok) {
      LOG_ERR(MOD_RUN, "color_analyzer_set_colors_failed", colors_just_read);
      String diagram_str = rubik_54_to_labeled_diagram(colors_just_read);
      Serial.print(diagram_str);
      color_reader.clear_color_reader();
      ori.restore_cube_orientation();
      return false;
    }
    color_reader.clear_color_reader();
  }

  // restore ori
  ori.restore_cube_orientation();

  // After read
  LOG_INFO(MOD_RUN, "color_reader_should_be_clear_string_54", color_analyzer.get_standard_color_string_54());
  LOG_INFO(MOD_RUN, "color_analyze_color_string_54", color_analyzer.get_standard_color_string_54());
  // LOG_INFO(MOD_RUN, "color_analyze_string_faces", color_analyzer.get_color_string_faces());
  LOG_INFO(MOD_RUN, "color_analyzer_is_valid", color_analyzer.is_color_string_valid_bool());
  LOG_INFO(MOD_RUN, "color_analyzer_is_fixable", color_analyzer.is_string_fixable_bool());
  LOG_INFO(MOD_RUN, "ori_orientation", ori.get_orientation_string());
  return true;
}

bool cmd_getcolor_data(int argc, double *argv) {
  LOG_INFO(MOD_RUN, "cube_color_string_54", color_analyzer.get_standard_color_string_54());
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
  RUN_PING(ID_GRIP1);
  RUN_PING(ID_GRIP2);

  // bring the grip in position
  LOG_INFO(MOD_SERVO_MOVE, "bring grip in position", Y_CENTER);
  // open gripper before positioning
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");

  // move to center and correct any deflection
  RUN_CMD(cmdMoveXmm(X_CENTER), "move x to center");
  RUN_CMD(cmdMoveYmm(Y_CENTER), "move y to center");
  RUN_CMD(cmdMoveXmm(X_CENTER), "re-center x");

  // ensure wrist is in known horizontal orientation
  RUN_CMD(cmdMoveWristDegVertical(W_HORIZ_RIGHT), "set wrist horizontal right");

  // final re-center after wrist motion
  RUN_CMD(cmdMoveXmm(X_CENTER), "final re-center x");
  RUN_CMD(cmdMoveYmm(Y_CENTER), "final re-center y");

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
      RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");
      RUN_CMD(cmdMoveXmm(X_CENTER), "move x to center");
      RUN_CMD(cmdMoveYmm(Y_DOWN), "move y down");

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
      RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");
      RUN_CMD(cmdMoveXmm(X_CENTER), "move x to center");
      RUN_CMD(cmdMoveYmm(Y_DOWN), "move y down");
      LOG_INFO(MOD_SERVO_MOVE, "cube_detection_info", "(free close)");
      LOG_INFO(MOD_SERVO_MOVE, "cube_detected", "no");
      return false;
    }
  }

  // Safety fallback: no touch detected
  RUN_CMD(cmdMoveGripperPer(G_OPEN), "open gripper");
  RUN_CMD(cmdMoveXmm(X_CENTER), "move x to center");
  RUN_CMD(cmdMoveYmm(Y_DOWN), "move y down");
  LOG_INFO(MOD_SERVO_MOVE, "cube_detection_info", "(timeout)");
  LOG_INFO(MOD_SERVO_MOVE, "cube_detected", "no");
  return false;
}

// -------------------------------------------------------------------
// INFOSERVO <id> : print key control-table data for one servo
// -------------------------------------------------------------------
bool print_servo_info(uint8_t id) {
  RUN_PING(id);

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
  return cmd_check_ori_run();
}

bool cmd_check_ori_run() {

  char just_read_robot_color_front = cmd_read_one_color_run(5);  // 5 is the slot for the middle
  LOG_INFO(MOD_CMD, "color showing in front", just_read_robot_color_front);
  if (!is_valid_color(just_read_robot_color_front)) {
    LOG_ERR(MOD_RUN, "invalid_robot_center_color", just_read_robot_color_front);
    return false;
  }
  // cube face showing in front
  char robot_face_showing_in_front = color_to_face(just_read_robot_color_front);
  LOG_INFO(MOD_CMD, "corresponding face showing in front", robot_face_showing_in_front);
  if (!is_valid_face(robot_face_showing_in_front)) {
    LOG_ERR(MOD_RUN, "invalid_robot_center_face", robot_face_showing_in_front);
    return false;
  }
  // face ori says should be in front
  char robot_face_that_should_be_showing_in_front = ori.robot_face_to_cube_face('f');
  LOG_INFO(MOD_CMD, "face that should be showing per ori", robot_face_that_should_be_showing_in_front);

  if (robot_face_that_should_be_showing_in_front != robot_face_showing_in_front) {
    LOG_ERR(MOD_CMD, "ori face does not match robot face in front, robot", robot_face_showing_in_front);
    LOG_VAR("ori", robot_face_that_should_be_showing_in_front);
    return false;
  }
  LOG_INFO(MOD_CMD, "front face matches ori, robot", robot_face_showing_in_front);
  LOG_VAR("ori", robot_face_that_should_be_showing_in_front);
  return true;
}

bool cmd_detect_ori(int argc, double *argv) {
  LOG_INFO(MOD_RUN, "info", "color read for centers will update ori");
  if (!cmd_read_cube_colors("centers")) return false;
  return true;
}