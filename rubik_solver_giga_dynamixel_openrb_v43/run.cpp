#include "utils.h"
#include "rb_interface.h"
#include "logging.h"
#include "ui_touch.h"

//-----------------------------------------------------------------------------------------

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
  String text = String("run robot move ") + String(move);
  setFooter(text.c_str());
  // run
  int cmd_id = 0;
  result = runCommand("ROBOTMOVE", move, &cmd_id);
  // updated footer
  if (!result) {
    LOG_ERR("ROBOTMOVE %s", getLastError(cmd_id).c_str());
    text = String("run robot move ") + String(move) + String(" failed");
    setFooter(text.c_str());
  } else {
    text = String("run robot move ") + String(move) + String(" ok");
    setFooter(text.c_str());
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
  String text = String("run cube move ") + move;
  setFooter(text.c_str());

  // ---------------- Run command ----------------
  int cmd_id = 0;
  result = runCommand("CUBEMOVE", move, &cmd_id);

  // ---------------- Footer + logging ----------------
  if (!result) {
    LOG_ERR("CUBEMOVE %s", getLastError(cmd_id).c_str());
    text = String("run cube move ") + move + " failed";
    setFooter(text.c_str());
  } else {
    text = String("run cube move ") + move + " ok";
    setFooter(text.c_str());
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

  String text = String("run ") + cmd;
  setFooter(text.c_str());

  int cmd_id = 0;
  result = runCommand("RUN", cmd, &cmd_id);

  if (!result) {
    LOG_ERR("RUN %s", getLastError(cmd_id).c_str());
    setFooter((text + " failed").c_str());
  } else {
    setFooter((text + " ok").c_str());
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
  String text = String("read color sticker c") + slot;
  setFooter(text.c_str());

  // ---------------- Run command ----------------
  int cmd_id = 0;
  result = runCommand("ONECOLOR", slot, &cmd_id);

  // ---------------- Footer + logging ----------------
  if (!result) {
    LOG_ERR("ONECOLOR", "error", getLastError(cmd_id).c_str());
    text = String("read color sticker c") + slot + " failed";
    setFooter(text.c_str());
  } else {
    text = String("read color sticker c") + slot + " ok";
    setFooter(text.c_str());
  }

  return true;
}

bool runSolveCubeFindSolutionByKey(const char* key, bool& result) {
  if (!key) return false;
  if (strcmp(key, "k_solve_cube_find_solution") != 0) return false;  //k_solve_cube_solve

  String text = String("run solve cube");
  setFooter(text.c_str());

  int cmd_id = -1;

  //TODO find cube goes here;

  if (!result) {
    LOG_ERR("RUN %s", getLastError(cmd_id).c_str());
    setFooter((text + " failed").c_str());
  } else {
    setFooter((text + " ok").c_str());
  }

  return true;
}

bool runSolveCubeRunSolutionByKey(const char* key, bool& result) {
  if (!key) return false;
  if (strcmp(key, "k_solve_cube_run_solution") != 0) return false;  //k_solve_cube_solve

  String text = String("run solve cube");
  setFooter(text.c_str());

  int cmd_id = -1;

  //TODO solve cube goes here;

  if (!result) {
    LOG_ERR("RUN %s", getLastError(cmd_id).c_str());
    setFooter((text + " failed").c_str());
  } else {
    setFooter((text + " ok").c_str());
  }

  return true;
}
bool runSystemByKey(const char* key, bool& result) {
  if (!key) return false;

  String cmd;

  if (strcmp(key, "k_system") == 0) cmd = "READSERVO";
  // else if (strcmp(key, "k_tests") == 0) cmd = "INFOSERVO"; //TODO to add ?
  else if (strcmp(key, "k_reboot_all") == 0) cmd = "REBOOTALL";         //TODO to add ?
  else if (strcmp(key, "k_set_stop_all") == 0) cmd = "SETSTOPALL";      //TODO to add ?
  else if (strcmp(key, "k_clear_stop_all") == 0) cmd = "CLEARSTOPALL";  //TODO to add ?
  else return false;

  setFooter(cmd.c_str());

  int cmd_id = 0;
  result = runCommand(cmd, "", &cmd_id);

  if (!result) {
    LOG_ERR(cmd.c_str(), "error", getLastError(cmd_id).c_str());
    setFooter((cmd + " failed").c_str());
  } else {
    setFooter((cmd + " ok").c_str());
  }
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

  setFooter(cmd.c_str());

  int cmd_id = 0;
  result = runCommand(cmd, param, &cmd_id);

  if (!result) {
    LOG_ERR(cmd.c_str(), "error", getLastError(cmd_id).c_str());
    setFooter((cmd + " failed").c_str());
  } else {
    setFooter((cmd + " ok").c_str());
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

  String text = cmd + " " + param;
  setFooter(text.c_str());

  int cmd_id = 0;
  result = runCommand(cmd, param, &cmd_id);

  if (!result) {
    LOG_ERR(cmd.c_str(), "error", getLastError(cmd_id).c_str());
    setFooter((text + " failed").c_str());
  } else {
    setFooter((text + " ok").c_str());
  }
  return true;
}

bool runColorOrientationByKey(const char* key, bool& result) {
  if (!key) return false;

  String move;

  if (strcmp(key, "k_color_y_plus") == 0) move = "y+";
  else if (strcmp(key, "k_color_z_plus") == 0) move = "z+";
  else return false;

  setFooter(("rotate " + move).c_str());

  int cmd_id = 0;
  result = runCommand("ROBOTMOVE", move, &cmd_id);

  if (!result) {
    LOG_ERR("ROBOTMOVE %s", getLastError(cmd_id).c_str());
    setFooter("rotate failed");
  } else {
    setFooter("rotate ok");
  }
  return true;
}

//-----------------------------------------------------------------------------------------