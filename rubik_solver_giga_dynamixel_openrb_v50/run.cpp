#include "utils.h"
#include "rb_interface.h"
#include "logging.h"
#include "ui_touch.h"
#include "ui_button.h"
#include "run.h"
#include "rubik_solver.h"
#include "ui_cube_view.h"

bool send_move_cube_progress_bool = false;
bool send_move_robot_progress_bool = false;
bool send_orientation_data_bool = false;
bool send_readcolors_progress_bool = false;
bool send_cube_view_bool = false;
int last_onecolor_read_slot = -1;

String getRbVersion();

//-----------------------------------------------------------------------------------------

String getSystemText() {
  String servoText = getLastServoStatusStr(0);

  String systemText =
    String("#FFA500 main build# ") + getSketchVersionWithDate() + "\n" +  //
    String("#FFA500 rb build# ") + getRbVersion() + "\n\n" +              //
    String("#FFA500 servos status#\n") + servoText + "\n" +               //
    String("#FFA500 errors log#\n") + getAllErrorLines();
  return systemText;
}

void buttonAction_run(const char* btn_key) {

  //String footer_txt = String("start ") + String(btn_key);
  //setFooter(footer_txt.c_str());
  delay(11);
  LOG_PRINTF_RUN("running action for key {%s}\n", btn_key);

  bool handled = false;
  bool ok = false;

  handled = runRobotMovesByKey(btn_key, ok);
  if (!handled) handled = runCubeMoveByKey(btn_key, ok);
  if (!handled) handled = runRunOrientationByKey(btn_key, ok);
  if (!handled) handled = runSolveReadByKey(btn_key, ok);
  if (!handled) handled = runColorReadByKey(btn_key, ok);
  if (!handled) handled = runColorOrientationByKey(btn_key, ok);
  if (!handled) handled = runSolveCubeFindSolutionByKey(btn_key, ok);
  if (!handled) handled = runSolveCubeRunSolutionByKey(btn_key, ok);
  if (!handled) handled = runColorStickerByKey(btn_key, ok);
  if (!handled) handled = runSystemByKey(btn_key, ok);

  //TODO-> TO TEST if more actions go here

  if (handled) {
    // footer_txt = String("end ") + String(btn_key);
    // setFooter(footer_txt.c_str());
  }

  if (!handled) {
    String text = String("action not implemented ") + String(btn_key);
    LOG_ERR("[RUN] unhandled action key=%s\n", btn_key);
    setFooter(text.c_str(), _ERROR);
  }
}

bool runRobotMovesByKey(const char* key, bool& result) {
  if (!key) return false;

  String move;

  // ---------------- D moves ----------------
  if (strcmp(key, "k_robot_moves_d_plus") == 0) move = "d+";
  else if (strcmp(key, "k_robot_moves_d_minus") == 0) move = "d-";
  else if (strcmp(key, "k_robot_moves_d2") == 0) move = "d2";

  // ---------------- Z moves ----------------
  else if (strcmp(key, "k_robot_moves_z_plus") == 0) move = "z+";
  else if (strcmp(key, "k_robot_moves_z_minus") == 0) move = "z-";
  else if (strcmp(key, "k_robot_moves_z2") == 0) move = "z2";

  // ---------------- Y moves ----------------
  else if (strcmp(key, "k_robot_moves_y_plus") == 0) move = "y+";
  else if (strcmp(key, "k_robot_moves_y_minus") == 0) move = "y-";
  else if (strcmp(key, "k_robot_moves_y2") == 0) move = "y2";

  else return false;

  // updated footer
  String text = String("runnning... robot move ") + String(move);
  setFooter(text.c_str(), _RUNNING_STOP);
  // run
  int cmd_id = -1;

  send_move_robot_progress_bool = true;
  result = runCommand("moverobot", move, &cmd_id);
  send_move_robot_progress_bool = false;

  // updated footer
  if (!result) {
    LOG_ERR("[RUN] moverobot error=%s", getLastError(cmd_id).c_str());
    text = String("robot move ") + String(move) + String(" failed");
    setFooter(text.c_str(), _DONE_ERROR);
  } else {
    text = String("robot move ") + String(move) + String(" done");
    setFooter(text.c_str(), _DONE_SUCCESS);
  }
  return true;
}

//-----------------------------------------------------------------------------------------

bool runCubeMoveByKey(const char* key, bool& result) {
  if (!key) return false;

  String move;

  // ---------------- F moves ----------------
  if (strcmp(key, "k_cube_moves_f_plus") == 0) move = "f+";
  else if (strcmp(key, "k_cube_moves_f_minus") == 0) move = "f-";
  else if (strcmp(key, "k_cube_moves_f2") == 0) move = "f2";

  // ---------------- R moves ----------------
  else if (strcmp(key, "k_cube_moves_r_plus") == 0) move = "r+";
  else if (strcmp(key, "k_cube_moves_r_minus") == 0) move = "r-";
  else if (strcmp(key, "k_cube_moves_r2") == 0) move = "r2";

  // ---------------- U moves ----------------
  else if (strcmp(key, "k_cube_moves_u_plus") == 0) move = "u+";
  else if (strcmp(key, "k_cube_moves_u_minus") == 0) move = "u-";
  else if (strcmp(key, "k_cube_moves_u2") == 0) move = "u2";

  // ---------------- B moves ----------------
  else if (strcmp(key, "k_cube_moves_b_plus") == 0) move = "b+";
  else if (strcmp(key, "k_cube_moves_b_minus") == 0) move = "b-";
  else if (strcmp(key, "k_cube_moves_b2") == 0) move = "b2";

  // ---------------- L moves ----------------
  else if (strcmp(key, "k_cube_moves_l_plus") == 0) move = "l+";
  else if (strcmp(key, "k_cube_moves_l_minus") == 0) move = "l-";
  else if (strcmp(key, "k_cube_moves_l2") == 0) move = "l2";

  // ---------------- D moves ----------------
  else if (strcmp(key, "k_cube_moves_d_plus") == 0) move = "d+";
  else if (strcmp(key, "k_cube_moves_d_minus") == 0) move = "d-";
  else if (strcmp(key, "k_cube_moves_d2") == 0) move = "d2";

  else return false;  // not a cube move key

  // ---------------- Footer update ----------------
  String text = String("running... cube move ") + move;
  setFooter(text.c_str(), _RUNNING_STOP);

  // ---------------- Run command ----------------
  int cmd_id = -1;
  send_cube_view_bool = true;
  result = runCommand("movecube", move, &cmd_id);
  send_cube_view_bool = false;
  // ---------------- Footer + logging ----------------
  if (!result) {
    LOG_ERR("[RUN] movecube error=%s", getLastError(cmd_id).c_str());
    text = String("cube move ") + move + " failed";
    setFooter(text.c_str(), _DONE_ERROR);
  } else {
    text = String("cube move ") + move + " ok";
    setFooter(text.c_str(), _DONE_SUCCESS);
  }

  return true;
}

bool runRunOrientationByKey(const char* key, bool& result) {
  if (!key) return false;

  String cmd;

  if (strcmp(key, "k_orientation_detect") == 0) cmd = "DETECTORI";
  else if (strcmp(key, "k_orientation_check") == 0) cmd = "CHECKORI";
  else if (strcmp(key, "k_orientation_restore") == 0) cmd = "RESTOREORI";
  else return false;

  String text = String("running... ") + cmd;
  setFooter(text.c_str(), _RUNNING_STOP);

  int cmd_id = -1;
  send_orientation_data_bool = true;
  result = runCommand("RUN", cmd, &cmd_id);
  send_orientation_data_bool = false;

  if (!result) {
    LOG_ERR("[RUN] error=%s", getLastError(cmd_id).c_str());
    setFooter((text + " failed").c_str(), _DONE_SUCCESS);
  } else {
    setFooter((text + " ok").c_str(), _DONE_ERROR);
  }
  return true;
}

bool runColorStickerByKey(const char* key, bool& result) {
  if (!key) return false;

  String slot;

  // ---------------- Color sticker slots ----------------
  if (strcmp(key, "k_color_c1") == 0) slot = "1";
  else if (strcmp(key, "k_color_c2") == 0) slot = "2";
  else if (strcmp(key, "k_color_c3") == 0) slot = "3";
  else if (strcmp(key, "k_color_c4") == 0) slot = "4";
  else if (strcmp(key, "k_color_c5") == 0) slot = "5";
  else if (strcmp(key, "k_color_c6") == 0) slot = "6";
  else return false;  // not a color sticker key

  // ---------------- Footer update ----------------
  String text = String("running... read color sticker c") + slot;
  setFooter(text.c_str(), _RUNNING_NOSTOP);

  // ---------------- Run command ----------------
  int cmd_id = -1;
  result = runCommand("ONECOLOR", slot, &cmd_id);

  // ---------------- Footer + logging ----------------
  if (!result) {
    LOG_ERR("[RUN] ONECOLOR error {%s}", getLastError(cmd_id).c_str());
    text = String("read color sticker c") + slot + " failed";
    setFooter(text.c_str(), _DONE_ERROR);
    last_onecolor_read_slot = -1;
  } else {
    int icmd = -1;
    char one_color_char = getLastColorOneColor(&icmd);
    last_onecolor_read_slot = slot.toInt();
    buttons_set_text_by_key(key, String(one_color_char).c_str());
    text = String("read color sticker c") + slot + " " + String(one_color_char) + " done";
    setFooter(text.c_str(), _DONE_SUCCESS);
  }

  return true;
}

bool runSolveCubeFindSolutionByKey(const char* key, bool& result) {
  if (!key) return false;
  if (strcmp(key, "k_solve_cube_find_solution") != 0) return false;  //k_solve_cube_solve

  String text = String("running... find solution");
  setFooter(text.c_str(), _RUNNING_NOSTOP);

  int cmd_id = -1;
  result = false;

  String cube = getLastColorString();
  String solution = "";

  bool ok = is_valid_color_string(cube);
  if (!ok) {
    LOG_ERR("[RUN] not a valid color string %s", cube.c_str());
    setFooter("color read failed", _DONE_ERROR);
  }
  if (ok) {
    ok = is_solved_top_two_layers(cube);
    if (ok) {
      solution = find_solution_for_bottom_layer(cube);
    } else {
      //TODO-> TO IMPLEMENT add solution for full cube
      solution = "";
      LOG_ERR("[SOLVER] for now supporting only bottom layer solution %s", cube.c_str());
      setFooter("full solution not implemented", _DONE_ERROR);
    }
  }

  if (solution.isEmpty()) {
    LOG_ERR("[SOLVER] no solution found for %s\n", cube.c_str());
    result = false;
  } else {
    setLastCubeSolution(solution);
    result = true;
  }

  if (!result) {
    LOG_ERR("[RUN] RUN %s\n", getLastError(cmd_id).c_str());
    setFooter("no solution found", _DONE_ERROR);
  } else {
    setFooter("find solution done", _DONE_SUCCESS);
  }

  return true;
}

bool runSolveCubeRunSolutionByKey(const char* key, bool& result) {
  if (!key) return false;
  if (strcmp(key, "k_solve_cube_run_solution") != 0) return false;  //k_solve_cube_solve

  setFooter("running... solve cube", _RUNNING_STOP);

  int cmd_id = -1;
  result = false;

  String solution = getLastCubeSolution();
  ui_cube_view_set_colors(getLastColorString());
  if (solution = "") {
    LOG_ERR("[SOLVER] %s no solution\n", key);
    setFooter("no solution found", _DONE_ERROR);
    return true;  // handled
  }

  cmd_id = -1;
  ui_moves_progress_set_map(solution, getColorsForSolution(solution));
  ui_moves_progress_set_index(0);
  LOG_PRINTF("progress bar created with {%s}\n", solution.c_str());

  send_move_cube_progress_bool = true;
  bool ok = runCommand("movecube", solution, &cmd_id);
  send_move_cube_progress_bool = false;
  result = ok;

  if (ok) setFooter("solve cube done", _DONE_SUCCESS);
  else setFooter("solve cube failed", _DONE_ERROR);

  return true;  //handled
}

// TODO-> TO TEST
bool runSystemByKey(const char* key, bool& result) {
  if (!key) return false;

  String cmd;

  if (strcmp(key, "k_servos_info") == 0) cmd = "INFOSERVO";
  else if (strcmp(key, "k_reboot_all") == 0) cmd = "REBOOTALL";
  else if (strcmp(key, "k_set_stop_all") == 0) cmd = "SETSTOPALL";
  else if (strcmp(key, "k_run_zero") == 0) cmd = "RUN 0";
  else return false;

  String text = "running... " + cmd;
  if (strcmp(key, "k_run_zero") == 0) setFooter(text.c_str(), _RUNNING_STOP);
  else setFooter(text.c_str(), _RUNNING_NOSTOP);

  int cmd_id = -1;
  result = runCommand(cmd, "", &cmd_id);
  if (result) {
    if (strcmp(key, "k_reboot_all") == 0) {
      // reboot does also clear
      cmd = cmd = "CLEARSTOPALL";
      result = runCommand(cmd, "", &cmd_id);
    }
  }

  if (!result) {
    LOG_ERR("[RUN] %s: error {%s}", cmd.c_str(), getLastError(cmd_id).c_str());
    setFooter((cmd + " failed").c_str(), _DONE_ERROR);
  } else {
    setFooter((cmd + " done").c_str(), _DONE_SUCCESS);
  }
  String system_info = getSystemText();
  LOG_PRINTF("updated system info /n{%s}/n", system_info.c_str());
  buttons_set_text_by_key("k_system_info_text", system_info.c_str());

  return true;
}

bool runColorReadByKey(const char* key, bool& result) {
  if (!key) return false;

  String cmd, param;

  if (strcmp(key, "k_orientation_color_read_all") == 0 ||  //
      strcmp(key, "k_solve_cube_read_cube_all") == 0) {
    cmd = "READCOLORS";
    param = "all";
  } else if (strcmp(key, "k_orientation_color_read_bottom") == 0) {
    cmd = "READCOLORS";
    param = "bottom";
  } else if (strcmp(key, "k_orientation_color_read_sensor") == 0) {
    cmd = "READCOLORS";
    param = "centers";
  } else return false;

  setFooter("running... read colors", _RUNNING_STOP);

  int cmd_id = -1;
  send_readcolors_progress_bool = true;
  result = runCommand(cmd, param, &cmd_id);
  send_readcolors_progress_bool = false;

  if (!result) {
    LOG_ERR("[RUN] %s: error {%s}", cmd.c_str(), getLastError(cmd_id).c_str());
    setFooter("read colors failed", _DONE_ERROR);
  } else {
    setFooter("read colors done", _DONE_SUCCESS);
  }
  return true;
}

bool runSolveReadByKey(const char* key, bool& result) {
  if (!key) return false;

  String cmd, param;

  if (strcmp(key, "k_read_cube_all") == 0) {
    cmd = "READCOLORS";
    param = "all";
  } else if (strcmp(key, "k_read_cube_bottom") == 0) {
    cmd = "READCOLORS";
    param = "bottom";
  } else if (strcmp(key, "k_read_cube_centers") == 0) {
    cmd = "READCOLORS";
    param = "centers";
  } else return false;

  setFooter("running... read colors", _RUNNING_STOP);

  int cmd_id = -1;
  send_readcolors_progress_bool = true;
  result = runCommand(cmd, param, &cmd_id);
  send_readcolors_progress_bool = false;

  if (!result) {
    LOG_ERR("[RUN] %s: error {%s}", cmd.c_str(), getLastError(cmd_id).c_str());
    setFooter("read colors failed", _DONE_ERROR);
  } else {
    setFooter("read colors done", _DONE_SUCCESS);
  }
  return true;
}

bool runColorOrientationByKey(const char* key, bool& result) {
  if (!key) return false;

  String move;

  if (strcmp(key, "k_color_y_plus") == 0) move = "y+";
  else if (strcmp(key, "k_color_z_plus") == 0) move = "z+";
  else return false;

  setFooter("running... rotate", _RUNNING_STOP);
  int cmd_id = -1;
  result = runCommand("moverobot", move, &cmd_id);

  if (!result) {
    LOG_ERR("[RUN] moverobot error=%s", getLastError(cmd_id).c_str());
    setFooter("rotate failed", _DONE_ERROR);
  } else {
    setFooter("rotate done", _DONE_SUCCESS);
  }
  return true;
}

//-----------------------------------------------------------------------------------------
