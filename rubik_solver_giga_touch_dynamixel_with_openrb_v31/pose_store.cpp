#include "pose_store.h"
#include "param_store.h"
#include "logging.h"
#include "rb_interface.h"
#include "ui_status.h"

extern RBInterface rb;

// -----------------------------------------------------------
// Add or update a pose
// -----------------------------------------------------------
bool PoseStore::add_pose(const char *name, const char *type, double p1, double p2,
                         const char *btn_key, double step, double min_val,
                         double max_val) {
  if (!name || !*name || !type || !*type) return false;
  int idx = find_pose_index(name);
  if (idx < 0 && count >= MAX_POSES) return false;
  if (idx < 0) idx = count++;

  Pose &p = poses_list[idx];
  p.name = name;
  p.move_type = type;
  p.p1 = p1;
  p.p2 = p2;
  p.button_key = btn_key ? btn_key : "";
  p.step = step;
  p.min_val = min_val;
  p.max_val = max_val;
  p.last_run_ok = true;

  setParamValue((String("pose_") + name + "_p1").c_str(), (int)(p1 * 100.0));
  return true;
}

// -----------------------------------------------------------
// Find pose by name / button
// -----------------------------------------------------------
int PoseStore::find_pose_index(const char *name) const {
  for (int i = 0; i < count; i++)
    if (poses_list[i].name.equalsIgnoreCase(name))
      return i;
  return -1;
}

int PoseStore::find_pose_by_button(const char *btn_key) const {
  for (int i = 0; i < count; i++)
    if (poses_list[i].button_key.equalsIgnoreCase(btn_key))
      return i;
  return -1;
}

bool PoseStore::is_pose(const char *name) const {
  return find_pose_index(name) >= 0;
}
bool PoseStore::is_button_for_pose(const char *btn_key) const {
  return find_pose_by_button(btn_key) >= 0;
}

// -----------------------------------------------------------
// Accessors
// -----------------------------------------------------------
bool PoseStore::get_pose_params(const char *name, double *p1, double *p2) const {
  int idx = find_pose_index(name);
  if (idx < 0) return false;
  if (p1) *p1 = poses_list[idx].p1;
  if (p2) *p2 = poses_list[idx].p2;
  return true;
}

bool PoseStore::set_pose_params(const char *name, double p1, double p2) {
  int idx = find_pose_index(name);
  if (idx < 0) return false;
  Pose &p = poses_list[idx];
  p.p1 = p1;
  p.p2 = p2;
  setParamValue((String("pose_") + name + "_p1").c_str(), (int)(p1 * 100.0));
  return true;
}

char *PoseStore::param_to_pose(const char *param_name) const {
  if (!param_name || !*param_name) return nullptr;
  String key = param_name;
  if (!key.startsWith("pose_") || !key.endsWith("_p1")) return nullptr;
  static String name;
  name = key.substring(5, key.length() - 3);
  return (char *)name.c_str();
}

// -----------------------------------------------------------
// Increment / Decrement
// -----------------------------------------------------------
bool PoseStore::increment_pose_param(const char *param_name, int units, double &new_value_ref) {
  if (!param_name || !*param_name) return false;

  String key = param_name;
  if (!key.endsWith("_param")) return false;

  // Extract pose name: everything before "_param"
  String pose_name = key.substring(0, key.length() - 6);

  int idx = find_pose_index(pose_name.c_str());
  if (idx < 0) return false;

  Pose &p = poses_list[idx];
  double new_val = p.p1 + (units * p.step);

  if (new_val < p.min_val) new_val = p.min_val;
  if (new_val > p.max_val) new_val = p.max_val;

  p.p1 = new_val;
  setParamValue(param_name, (int)(new_val * 100.0));
  new_value_ref = new_val;

  LOG_PRINTF("running pose{%s} with new val {%d}\n", pose_name.c_str(), new_val);
  run_pose(pose_name.c_str());

  LOG_PRINTF("pose{%s} adjusted to %.2f units{%d}\n", pose_name.c_str(), new_val, units);
  return true;
}

// -----------------------------------------------------------
// Map UI / param names to internal pose records
// -----------------------------------------------------------
void PoseStore::set_pose_val_from_param(const char *param_name, double val) {
  LOG_SECTION_START_VAR("set_pose_val_from_param", "param", param_name ? param_name : "(null)");

  if (!param_name || !*param_name) {
    LOG_PRINTF("invalid or empty param name\n");
    LOG_SECTION_END();
    return;
  }

  String key = param_name;
  if (!key.endsWith("_param")) {
    LOG_PRINTF("not a pose param {%s}\n", param_name);
    LOG_SECTION_END();
    return;
  }

  // Extract pose name: everything before "_param"
  String pose_name = key.substring(0, key.length() - 6);  // remove "_param"

  // Find pose by name
  int idx = find_pose_index(pose_name.c_str());
  if (idx < 0) {
    LOG_PRINTF("pose {%s} not found\n", pose_name.c_str());
    LOG_SECTION_END();
    return;
  }

  Pose &p = poses_list[idx];
  p.p1 = val;

  // Save the integer-scaled version (for persistent param store)
  setParamValue(param_name, (int)(val * 100.0));

  LOG_PRINTF("updated pose {%s} p1=%.2f\n", pose_name.c_str(), val);
  LOG_SECTION_END();
}

// -----------------------------------------------------------
// Convert a button name (e.g. "x_left_btn") to a pose name ("x_left")
// -----------------------------------------------------------
char *PoseStore::btn_to_pose(const char *btn_name) const {
  if (!btn_name || !*btn_name) return nullptr;

  int idx = find_pose_by_button(btn_name);
  if (idx < 0) return nullptr;

  // Return a pointer to the internal string buffer
  return (char *)poses_list[idx].name.c_str();
}

// -----------------------------------------------------------
// Check if a parameter corresponds to a stored pose
// -----------------------------------------------------------
bool PoseStore::is_param_for_pose(const char *param_name) const {
  if (!param_name || !*param_name) return false;

  String key = param_name;
  if (!key.endsWith("_param"))
    return false;

  // Replace "_param" with "_btn" to get the corresponding button key
  String btn_key = key;
  btn_key.replace("_param", "_btn");

  // Check if any pose has this button key
  return is_button_for_pose(btn_key.c_str());
}

bool PoseStore::is_btn_for_pose(const char *btn_key) const {
  if (find_pose_by_button(btn_key) < 0) return false;
  return true;
}

// -----------------------------------------------------------
// Run a pose
// -----------------------------------------------------------
bool PoseStore::run_pose(const char *pose_name) {
  int idx = find_pose_index(pose_name);
  if (idx < 0) return false;
  Pose &pose = poses_list[idx];

  double p1 = pose.p1, p2 = pose.p2;
  String type = pose.move_type;

  LOG_SECTION_START_PRINTF("pose store run_pose", "| {%s} type{%s}", pose_name, type.c_str());
  bool ok = false;

  if (type == "x") {
    ok = rb.moveXmm(p1);
  } else if (type == "y") {
    ok = rb.moveYmm(p1);
  } else if (type == "base") {
    ok = rb.moveBaseDeg(p1);
  } else if (type == "wrist") {
    ok = rb.moveWristVertDeg(p1);
  } else if (type == "grippers") {
    ok = rb.moveGrippersPer(p1);
  } else if (type == "gripper1") {
    ok = rb.moveGripper1Per(p1);
  } else if (type == "gripper2") {
    ok = rb.moveGripper2Per(p1);
  } else {
    LOG_PRINTF("ERR ⚠ unknown move type for {%s}\n", type.c_str());
  }
  pose.last_run_ok = ok;
  LOG_PRINTF("pose store run pose result {%}\n", ok);
  LOG_SECTION_END();
  return ok;
}

bool PoseStore::run_pose_by_button(const char *btn_key) {
  int idx = find_pose_by_button(btn_key);
  if (idx < 0) return false;
  return run_pose(poses_list[idx].name.c_str());
}

// -----------------------------------------------------------
// Position comparison
// -----------------------------------------------------------
bool PoseStore::is_at_pose(const char *name, double tol_mm, double tol_deg) {
  int idx = find_pose_index(name);
  if (idx < 0) return false;
  Pose &p = poses_list[idx];
  double val1 = 0, val2 = 0;

  if (p.move_type == "x") {
    rb.xyInfoMm(&val1, &val2);
    return fabs(val1 - p.p1) < tol_mm;
  }
  if (p.move_type == "y") {
    rb.xyInfoMm(&val1, &val2);
    return fabs(val2 - p.p1) < tol_mm;
  } else if (p.move_type == "servo" || p.move_type == "base") {
    double b;
    rb.baseInfoDeg(&b);
    return fabs(b - p.p1) < tol_deg;
  } else if (p.move_type == "wrist") {
    double w;
    rb.wristVertInfoDeg(&w);
    return fabs(w - p.p1) < tol_deg;
  } else if (p.move_type == "grippers") {
    double g;
    rb.grippersInfoPer(&g);
    return fabs(g - p.p1) < tol_deg;
  } else if (p.move_type == "gripper1") {
    double g;
    rb.gripper1InfoPer(&g);
    return fabs(g - p.p1) < tol_deg;
  } else if (p.move_type == "gripper2") {
    double g;
    rb.gripper2InfoPer(&g);
    return fabs(g - p.p1) < tol_deg;
  }

  return false;
}

// -----------------------------------------------------------
// Init from defaults (use RB zero for _0 servos)
// -----------------------------------------------------------
void PoseStore::init_from_defaults(const Pose *defaults, int def_count) {
  LOG_SECTION_START("PoseStore::init_from_defaults");

  for (int i = 0; i < def_count; i++) {
    const Pose &def = defaults[i];
    double val_p1 = def.p1, val_p2 = def.p2;

    add_pose(def.name.c_str(), def.move_type.c_str(), val_p1, val_p2,
             def.button_key.c_str(), def.step, def.min_val, def.max_val);
  }

  list_poses();
  LOG_SECTION_END();
}

void PoseStore::reflect_poses_ui() {
  LOG_SECTION_START("PoseStore::reflect_poses_ui");
  for (int i = 0; i < count; i++) {
    const Pose &p = poses_list[i];
    bool issue = p.last_run_ok;  //TODO change this to also reflect is rb is not working at all
    bool active = is_at_pose(p.button_key.c_str(), 0.5, 1.0);
    if (!issue) active = false;
    updateButtonStateByKey(p.button_key.c_str(), issue, active);
  }
  LOG_SECTION_END();
}

void PoseStore::set_all_poses_last_run(bool b) {
  LOG_SECTION_START("PoseStore::reflect_poses_ui");
  for (int i = 0; i < count; i++) {
    Pose &p = poses_list[i];
    p.last_run_ok = b;
    bool issue = b;  //TODO change this to also reflect is rb is not working at all
    bool active = is_at_pose(p.button_key.c_str(), 0.5, 1.0);
    if (!issue) active = false;
    updateButtonStateByKey(p.button_key.c_str(), issue, active);
  }
  LOG_SECTION_END();
}


// -----------------------------------------------------------
// Debug list
// -----------------------------------------------------------
void PoseStore::list_poses() const {
  LOG_SECTION_START("PoseStore::list_poses");
  for (int i = 0; i < count; i++) {
    const Pose &p = poses_list[i];
    LOG_PRINTF("(%2d) name{%-10s} | type{%-8s} | p1{%.2f} p2{%.2f} | step{%.2f} min{%.2f} max{%.2f} btn{%s} last{%d}\n",
               i, p.name.c_str(), p.move_type.c_str(), p.p1, p.p2,
               p.step, p.min_val, p.max_val, p.button_key.c_str(),p.last_run_ok);
  }
  LOG_SECTION_END();
}

// ============================================================
// Default poses_list (needs to match the menus)
// ============================================================
/*
  p.name = name;
  p.move_type = type;
  p.p1 = p1;
  p.p2 = p2;
  p.button_key = btn_key ? btn_key : "";
  p.step = step;
  p.min_val = min_val;
  p.max_val = max_val;
  p.servo_id = servo_id;
  */

// TODO - servo id and p2 are not used

Pose default_poses[] = {

  // XY poses_list
  { "y_zero", "y", 40.0, 0.0, "y_zero_btn", 0.5, 40.0, 110.0 },
  { "y_1st", "y", 50.0, 0.0, "y_1st_btn", 0.5, 40.0, 110.0 },
  { "y_2nd", "y", 60.0, 0.0, "y_2nd_btn", 0.5, 40.0, 110.0 },
  { "y_3rd", "y", 70.0, 0.0, "y_3rd_btn", 0.5, 40.0, 110.0 },

  { "y_c2", "y", 50.0, 0.0, "y_c2_btn", 0.5, 40.0, 110.0 },
  { "y_c3", "y", 60.0, 0.0, "y_c3_btn", 0.5, 40.0, 110.0 },

  { "x_center", "x", 0.0, 0.0, "x_center_btn", 0.5, -25.0, 25.0 },
  { "x_left", "x", -25.0, 0.0, "x_left_btn", 0.5, -35.0, 0.0 },
  { "x_right", "x", 25.0, 0.0, "x_right_btn", 0.5, 0.0, 35.0 },


  // Combined grippers
  { "grippers_open", "grippers", 80.0, 0.0, "grippers_open_btn", 0.5, 0.0, 100.0 },
  { "grippers_close", "grippers", 10.0, 0.0, "grippers_close_btn", 0.5, 0.0, 100.0 },

  // Individual gripper 1
  { "gripper1_open", "gripper1", 80.0, 0.0, "gripper1_open_btn", 0.5, 0.0, 100.0 },
  { "gripper1_close", "gripper1", 10.0, 0.0, "gripper1_close_btn", 0.5, 0.0, 100.0 },

  // Individual gripper 2
  { "gripper2_open", "gripper2", 80.0, 0.0, "gripper2_open_btn", 0.5, 0.0, 100.0 },
  { "gripper2_close", "gripper2", 10.0, 0.0, "gripper2_close_btn", 0.5, 0.0, 100.0 },

  // Wrist poses_list
  { "wrist_vert", "wrist", 0.0, 0.0, "wrist_vert_btn", 0.5, -45.0, 45.0 },
  { "wrist_horiz_left", "wrist", -90.0, 0.0, "wrist_horiz_left_btn", 0.5, 45.0, 135.0 },
  { "wrist_horiz_right", "wrist", 90.0, 0.0, "wrist_horiz_right_btn", 0.5, 135.0, 205.0 },

  // Base rotation
  { "base_front", "base", 0.0, 0.0, "base_front_btn", 0.5, -45.0, 45.0 },
  { "base_right", "base", 90.0, 0.0, "base_right_btn", 0.5, 45.0, 135.0 },
  { "base_left", "base", -90.0, 0.0, "base_left_btn", 0.5, 135.0, 205.0 }
};

const int DEFAULT_POSE_COUNT = sizeof(default_poses) / sizeof(default_poses[0]);

PoseStore pose_store;

bool initPoseStore() {
  pose_store.init_from_defaults(default_poses, DEFAULT_POSE_COUNT);
  return true;
}
