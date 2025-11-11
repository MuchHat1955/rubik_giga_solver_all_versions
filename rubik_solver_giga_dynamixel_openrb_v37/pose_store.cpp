#include "pose_store.h"
#include "param_store.h"
#include "rb_interface.h"
#include "logging.h"
#include "ui_status.h"
#include "ui_touch.h"


extern RBInterface rb;

// -----------------------------------------------------------
// Add or update a pose
// -----------------------------------------------------------
bool PoseStore::add_pose(const char *name, const char *type, double p1,
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
  p.button_key = btn_key ? btn_key : "";
  p.step = step;
  p.min_val = min_val;
  p.max_val = max_val;
  p.last_run_ok = true;

  setParamValue(name, p1);
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
bool PoseStore::get_pose_params(const char *name, double *p1) {

  // make a mutable local copy of the name
  char base_name[64];
  strncpy(base_name, name, sizeof(base_name));
  base_name[sizeof(base_name) - 1] = '\0';

  // if the name ends with "_param", remove it
  const char suffix[] = "_param";
  size_t len = strlen(base_name);
  size_t suf_len = strlen(suffix);
  if (len > suf_len && strcmp(base_name + len - suf_len, suffix) == 0) {
    base_name[len - suf_len] = '\0';
  }

  int idx = find_pose_index(base_name);
  if (idx < 0) return false;
  if (p1) *p1 = poses_list[idx].p1;
  return true;
}

bool PoseStore::set_pose_params(const char *name, double p1) {
  // --- Make a mutable copy of the input name
  char base_name[64];
  strncpy(base_name, name, sizeof(base_name));
  base_name[sizeof(base_name) - 1] = '\0';

  // --- If name ends with "_param", remove it for pose lookup
  const char suffix[] = "_param";
  size_t len = strlen(base_name);
  size_t suf_len = strlen(suffix);
  if (len > suf_len && strcmp(base_name + len - suf_len, suffix) == 0) {
    base_name[len - suf_len] = '\0';
  }

  // --- Lookup pose by base name (without "_param")
  int idx = find_pose_index(base_name);
  if (idx < 0) return false;

  Pose &p = poses_list[idx];
  p.p1 = p1;

  // --- Build key for saving parameter (ensure single "_param")
  char key_buf[64];
  if (strstr(base_name, "_param_param")) {
    // Defensive fix in case of malformed names
    strncpy(key_buf, base_name, sizeof(key_buf));
    key_buf[sizeof(key_buf) - 1] = '\0';
  } else if (strstr(base_name, "_param")) {
    // Already contains one _param
    strncpy(key_buf, base_name, sizeof(key_buf));
    key_buf[sizeof(key_buf) - 1] = '\0';
  } else {
    // Append _param
    snprintf(key_buf, sizeof(key_buf), "%s_param", base_name);
  }

  // --- Save to persistent storage
  setParamValue(key_buf, p1);

  return true;
}

char *PoseStore::param_to_pose(const char *param_name) const {
  if (!param_name || !*param_name) return nullptr;

  String key = param_name;
  if (!key.endsWith("_param")) return nullptr;

  // Extract the middle part between "pose_" and "_p1"
  static String name;
  name = key.substring(5, key.length() - 3);

  // If the extracted name ends with "_param", remove it
  const char suffix[] = "_param";
  int suf_len = strlen(suffix);
  if (name.length() > suf_len && name.endsWith(suffix)) {
    name.remove(name.length() - suf_len);
  }

  return (char *)name.c_str();
}

// -----------------------------------------------------------
// Increment / Decrement
// -----------------------------------------------------------
bool PoseStore::increment_pose_param(const char *param_name, int units, double &new_value_ref) {
  if (!param_name || !*param_name) return false;

  LOG_SECTION_START_PARAM("increment pose param", "name  {%s}", param_name);

  String key = param_name;

  // --- Normalize key to ensure it ends with "_param"
  if (!key.endsWith("_param")) key += "_param";

  // --- Extract base pose name (without "_param")
  String pose_name = key.substring(0, key.length() - 6);

  // --- Find pose by base name
  int idx = find_pose_index(pose_name.c_str());
  if (idx < 0) {
    LOG_PRINTF("pose not found for %s", pose_name.c_str());
    LOG_SECTION_END();
    return false;
  }

  Pose &p = poses_list[idx];

  // --- Compute new value within limits
  double new_val = p.p1 + (units * p.step);
  if (new_val < p.min_val) new_val = p.min_val;
  if (new_val > p.max_val) new_val = p.max_val;

  p.p1 = new_val;
  new_value_ref = new_val;

  // --- Build consistent param key for persistence
  String save_key = String(pose_name) + "_param";

  // --- Store new parameter value (x100)
  LOG_PRINTF("calling set param value for pose {%s} val {%.2f}\n", pose_name.c_str(), new_val);
  setParamValue(save_key.c_str(), new_val);

  // --- Execute updated pose
  LOG_PRINTF("running pose{%s} new val{%.2f} units{%d}\n", pose_name.c_str(), new_val, units);
  // run_pose(pose_name.c_str()); do not run here, has to click, but reflect in UI somehow is changed eg is_at_pose?
  // TODO reflect UI

  LOG_PRINTF("pose{%s} adjusted to %.2f (units %d)\n", pose_name.c_str(), new_val, units);
  LOG_SECTION_END();
  return true;
}

// -----------------------------------------------------------
// Map UI / param names to internal pose records
// -----------------------------------------------------------
void PoseStore::set_pose_val_from_param(const char *param_name, double val) {
  LOG_SECTION_START("set_pose_val_from_param", "param {%s}", param_name ? param_name : "(null)");

  if (!param_name || !*param_name) {
    LOG_PRINTF("invalid or empty param name\n");
    LOG_SECTION_END();
    return;
  }

  String key = param_name;
  if (!key.endsWith("_param")) {
    LOG_PRINTF("[!] not a pose param {%s}\n", param_name);
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

  // for persistent param store
  setParamValue(param_name, val);

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

  double p1 = pose.p1;
  String type = pose.move_type;

  updateButtonStateByKey(pose.button_key.c_str(), false, false, true);
  String text = "run pose " + String(pose_name);
  setFooter(text.c_str());

  LOG_SECTION_START("pose store run_pose", "| {%s} type{%s}", pose_name, type.c_str());
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
    LOG_PRINTF("[!] unknown move type for {%s}\n", type.c_str());
  }
  pose.last_run_ok = ok;
  LOG_PRINTF("pose store last run set for {%} to {%s}\n", pose.button_key.c_str(), pose.last_run_ok ? "true" : "false");

  bool active = is_at_pose(pose.button_key.c_str(), 0.5, 1.0);
  bool issue = !ok;
  LOG_PRINTF("    ---- reflect UI after pose run {%s} run with issue {%s} active{%s}\n",  //
             pose.button_key.c_str(),                                                     //
             issue ? "yes" : "no",                                                        //
             active ? "yes" : "no");
  updateButtonStateByKey(pose.button_key.c_str(), issue, active, false);

  LOG_SECTION_END();
  return ok;
}

bool PoseStore::run_pose_by_button(const char *btn_key) {
  int idx = find_pose_by_button(btn_key);
  if (idx < 0) idx = find_pose_index(btn_key);
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
  LOG_SECTION_START("PoseStore::init_from_defaults", "");

  for (int i = 0; i < def_count; i++) {
    const Pose &def = defaults[i];
    double val_p1 = def.p1;

    add_pose(def.name.c_str(), def.move_type.c_str(), val_p1,
             def.button_key.c_str(), def.step, def.min_val, def.max_val);
  }

  list_poses();
  LOG_SECTION_END();
}

void PoseStore::reflect_poses_ui() {
  //LOG_SECTION_START("PoseStore::reflect_poses_ui");
  for (int i = 0; i < count; i++) {
    const Pose &p = poses_list[i];
    bool issue = !p.last_run_ok;
    bool active = is_at_pose(p.button_key.c_str(), 0.5, 1.0);
    if (!issue) active = 0;
    if (issue) LOG_PRINTF("    ---- reflect UI for {%s} with issue {true}\n", p.button_key.c_str());
    updateButtonStateByKey(p.button_key.c_str(), issue, active, false);
  }
  //LOG_SECTION_END();
}

void PoseStore::set_all_poses_last_run(bool b) {
  LOG_SECTION_START_MENU("PoseStore::reflect_poses_ui", "");
  for (int i = 0; i < count; i++) {
    Pose &p = poses_list[i];
    p.last_run_ok = b;
    bool issue = !b;
    bool active = is_at_pose(p.button_key.c_str(), 0.5, 1.0);
    if (!issue) active = false;
    updateButtonStateByKey(p.button_key.c_str(), issue, active, false);
  }
  LOG_SECTION_END();
}

void PoseStore::update_pose_store_from_param_store(const Pose *defaults, int def_count) {
  LOG_SECTION_START("PoseStore::update_pose_store_from_param_store", "");

  for (int i = 0; i < def_count; i++) {
    const Pose &def = defaults[i];

    // --- Build the key used in ParamStore: "pose_param"
    String key = def.name;
    if (!key.endsWith("_param")) {
      key += "_param";
    }

    // --- Read from param store
    double stored_val = getParamValue(key.c_str());

    // --- getParamValue() returns PARAM_VAL_NA if not found,
    // so we’ll only apply if it’s non-zero and different from default
    if (stored_val < PARAM_VAL_NA && stored_val != 0.0) {  //TODO for now given the storage has many 0.0
      double restored_p1 = stored_val;                     // convert back from scaled int
      int idx = find_pose_index(def.name.c_str());

      if (idx >= 0) {
        Pose &p = poses_list[idx];
        p.p1 = restored_p1;

        LOG_PRINTF("Restored pose {%s} from param_store val{%.2f}\n",
                   def.name.c_str(), restored_p1);
      } else {
        LOG_ERROR("pose not found for default {%s}", def.name.c_str());
      }
    } else {
      LOG_PRINTF("    ---- No saved param for pose{%s}, keeping default {%.2f}\n",
                 def.name.c_str(), def.p1);
    }
  }

  LOG_SECTION_END();
}

// -----------------------------------------------------------
// Debug list
// -----------------------------------------------------------
void PoseStore::list_poses() const {
  LOG_SECTION_START("PoseStore::list_poses", "");
  for (int i = 0; i < count; i++) {
    const Pose &p = poses_list[i];
    LOG_PRINTF("(%2d) name{%-10s} | type{%-8s} | p1{%.2f} | step{%.2f} min{%.2f} max{%.2f} btn{%s} last{%s}\n",
               i, p.name.c_str(), p.move_type.c_str(), p.p1,
               p.step, p.min_val, p.max_val, p.button_key.c_str(), p.last_run_ok ? "true" : "false");
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
  p.button_key = btn_key ? btn_key : "";
  p.step = step;
  p.min_val = min_val;
  p.max_val = max_val;
  p.servo_id = servo_id;
  */

// TODO - servo id and p2 are not used

Pose default_poses[] = {

  // XY poses_list
  { "y_zero", "y", 40.0, "y_zero_btn", 0.5, 40.0, 110.0 },
  { "y_1st", "y", 50.0, "y_1st_btn", 0.5, 40.0, 110.0 },
  { "y_2nd", "y", 60.0, "y_2nd_btn", 0.5, 40.0, 110.0 },
  { "y_3rd", "y", 70.0, "y_3rd_btn", 0.5, 40.0, 110.0 },

  { "y_c2", "y", 50.0, "y_c2_btn", 0.5, 40.0, 110.0 },
  { "y_c3", "y", 60.0, "y_c3_btn", 0.5, 40.0, 110.0 },

  { "x_center", "x", 0.1, "x_center_btn", 0.5, -25.0, 25.0 },  // 0.1 to avoid the 0.0 meaning the value in the storage
  { "x_left", "x", -25.0, "x_left_btn", 0.5, -35.0, 0.0 },
  { "x_right", "x", 25.0, "x_right_btn", 0.5, 0.0, 35.0 },


  // Combined grippers
  { "grippers_open", "grippers", 80.0, "grippers_open_btn", 0.5, 0.0, 100.0 },
  { "grippers_close", "grippers", 10.0, "grippers_close_btn", 0.5, 0.0, 100.0 },

  // Individual gripper 1
  { "gripper1_open", "gripper1", 80.0, "gripper1_open_btn", 0.5, 0.0, 100.0 },
  { "gripper1_close", "gripper1", 10.0, "gripper1_close_btn", 0.5, 0.0, 100.0 },

  // Individual gripper 2
  { "gripper2_open", "gripper2", 80.0, "gripper2_open_btn", 0.5, 0.0, 100.0 },
  { "gripper2_close", "gripper2", 10.0, "gripper2_close_btn", 0.5, 0.0, 100.0 },

  // Wrist poses_list
  { "wrist_vert", "wrist", 0.1, "wrist_vert_btn", 0.5, -45.0, 45.0 },  // 0.1 to avoid the 0.0 meaning the value in the storage
  { "wrist_horiz_left", "wrist", -90.0, "wrist_horiz_left_btn", 0.5, 45.0, 135.0 },
  { "wrist_horiz_right", "wrist", 90.0, "wrist_horiz_right_btn", 0.5, 135.0, 205.0 },

  // Base rotation
  { "base_front", "base", 0.1, "base_front_btn", 0.5, -45.0, 45.0 },  // 0.1 to avoid the 0.0 meaning the value in the storage
  { "base_right", "base", 90.0, "base_right_btn", 0.5, 45.0, 135.0 },
  { "base_left", "base", -90.0, "base_left_btn", 0.5, 135.0, 205.0 }
};

const int DEFAULT_POSE_COUNT = sizeof(default_poses) / sizeof(default_poses[0]);

PoseStore pose_store;

bool initPoseStore() {
  pose_store.init_from_defaults(default_poses, DEFAULT_POSE_COUNT);
  pose_store.update_pose_store_from_param_store(default_poses, DEFAULT_POSE_COUNT);
  return true;
}
