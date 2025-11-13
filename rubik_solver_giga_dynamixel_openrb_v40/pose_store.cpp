#include "pose_store.h"
#include "param_store.h"
#include "rb_interface.h"
#include "logging.h"
#include "ui_status.h"
#include "ui_touch.h"
#include "ui_button.h"

extern RBInterface rb;

// -----------------------------------------------------------
// Add or update a pose
// -----------------------------------------------------------
bool PoseStore::add_pose(const char *key, const char *name, const char *type, double p1,
                         const char *btn_key, double step, double min_val,
                         double max_val) {
  if (!key || !*key || !key || !*type) return false;

  LOG_PRINTF_POSE("add pose | key {%s} | name {%s} | type {%s} | p1 {%.2f} | step {%.2f} | min {%.2f} | max {%.2f} | btn {%s}\n",
                  key, name, type, p1,
                  step, min_val, max_val, btn_key);

  int idx = find_pose_index(key);
  if (idx < 0 && count >= MAX_POSES) return false;
  if (idx < 0) idx = count++;

  Pose &p = poses_list[idx];
  p.name = name;
  p.key = key;
  p.move_type = type;
  p.p1 = p1;
  p.button_key = btn_key ? btn_key : "";
  p.step = step;
  p.min_val = min_val;
  p.max_val = max_val;
  p.last_run_ok = true;

  // setParamValue(key, p1);
  return true;
}

// -----------------------------------------------------------
// Find pose by name / button
// -----------------------------------------------------------
int PoseStore::find_pose_index(const char *akey) const {
  for (int i = 0; i < count; i++)
    if (poses_list[i].key.equalsIgnoreCase(String(akey)))
      return i;
  return -1;
}

int PoseStore::find_pose_by_key_param_button(const char *btn_key) const {
  if (!btn_key || !*btn_key) return -1;
  String target = String(btn_key);

  LOG_SECTION_START_POSE("find_pose_by_key_param_button | target {%s}", target.c_str());

  for (int i = 0; i < count; i++) {
    String curr = poses_list[i].button_key;
    LOG_PRINTF("button index {%d} crr {%s} target {%s}\n",
               i, curr.c_str(), target.c_str());
    if (curr.equalsIgnoreCase(target)) {
      LOG_PRINTF("return index {%d}\n", i);
      LOG_SECTION_END_POSE();
      return i;
    }
  }
  LOG_PRINTF("pose not found for {%s}\n", btn_key);
  LOG_SECTION_END_POSE();
  return -1;
}

bool PoseStore::is_pose(const char *key) const {
  return find_pose_index(key) >= 0;
}
bool PoseStore::is_button_for_pose(const char *btn_key) const {
  int ret = find_pose_by_key_param_button(btn_key) >= 0;
  return (ret >= 0);
}

// -----------------------------------------------------------
// Accessors
// -----------------------------------------------------------
bool PoseStore::get_pose_params(const char *name, double *p1) {
  int idx = find_pose_index(name);
  if (idx < 0) {
    LOG_ERROR("could not find pose param for pose{%s}\n", name);
    return false;
  }
  if (p1) *p1 = poses_list[idx].p1;
  return true;
}

bool PoseStore::save_pose_in_param_store(const char *name, double p1) {

  // --- Lookup pose by  name
  int idx = find_pose_index(name);
  if (idx < 0) return false;

  Pose &p = poses_list[idx];
  p.p1 = p1;

  // --- Save to persistent storage
  setParamValue(name, p1);

  return true;
}

// -----------------------------------------------------------
// Increment / Decrement
// -----------------------------------------------------------
bool PoseStore::increment_in_pose_store(const char *param_name, int units, double &new_value_ref) {
  if (!param_name || !*param_name) return false;

  LOG_SECTION_START_POSE("increment pose param | name  {%s}", param_name);

  // --- Find pose by name
  int idx = find_pose_index(param_name);
  if (idx < 0) {
    LOG_PRINTF_POSE("pose not found for %s", param_name);
    LOG_SECTION_END_POSE();
    return false;
  }

  Pose &p = poses_list[idx];

  // --- Compute new value within limits
  double new_val = p.p1 + (units * p.step);
  if (new_val < p.min_val) new_val = p.min_val;
  if (new_val > p.max_val) new_val = p.max_val;

  p.p1 = new_val;
  new_value_ref = new_val;

  // --- Store new parameter value (x100)
  LOG_PRINTF_POSE("calling set param value for pose {%s} val {%.2f}\n", param_name, new_val);
  setParamValue(param_name, new_val);

  LOG_PRINTF_POSE("pose{%s} adjusted in pose store to {%.2f} units {%d}\n", param_name, new_val, units);
  LOG_SECTION_END_POSE();
  return true;
}

// -----------------------------------------------------------
// Map UI / param names to internal pose records
// -----------------------------------------------------------
void PoseStore::set_pose_val_from_param(const char *param_name, double val) {
  LOG_SECTION_START_POSE("set_pose_val_from_param | param {%s}", param_name ? param_name : "(null)");

  if (!param_name || !*param_name) {
    LOG_PRINTF_POSE("invalid or empty param name\n");
    LOG_SECTION_END_POSE();
    return;
  }

  // Find pose by name
  int idx = find_pose_index(param_name);
  if (idx < 0) {
    LOG_PRINTF_POSE("pose {%s} not found\n", param_name);
    LOG_SECTION_END_POSE();
    return;
  }

  Pose &p = poses_list[idx];
  p.p1 = val;

  // for persistent param store
  setParamValue(param_name, val);

  LOG_PRINTF_POSE("updated pose | key {%s} | val {%.2f}\n", param_name, val);
  LOG_SECTION_END_POSE();
}

// -----------------------------------------------------------
// Check if a parameter corresponds to a stored pose
// -----------------------------------------------------------
bool PoseStore::is_param_for_pose(const char *param_name) const {
  if (!param_name || !*param_name) return false;
  return is_button_for_pose(param_name);
}

bool PoseStore::is_btn_for_pose(const char *btn_key) const {
  if (find_pose_by_key_param_button(btn_key) < 0) return false;
  return true;
}

// -----------------------------------------------------------
// Run a pose
// -----------------------------------------------------------
bool PoseStore::run_pose(const char *pose_btn) {
  int idx = find_pose_by_key_param_button(pose_btn);
  if (idx < 0) return false;
  Pose &pose = poses_list[idx];

  double p1 = pose.p1;
  String type = pose.move_type;

  String text = "run pose " + String(pose.key);
  setFooter(text.c_str());

  LOG_SECTION_START_POSE("run pose | pose {%s} | type{%s}", pose_btn, type.c_str());
  bool ok = false;

  drawButtonOverlayByText(pose.name.c_str());
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
    LOG_PRINTF_POSE("[!] unknown move type for {%s}\n", type.c_str());
  }
  pose.last_run_ok = ok;
  LOG_PRINTF_POSE("pose store last run for {%s} set to {%s}\n", pose.button_key.c_str(), pose.last_run_ok ? "true" : "false");

  bool active = is_at_pose(pose.button_key.c_str(), 0.5, 1.0);
  bool issue = !ok;
  LOG_PRINTF_POSE("reflect UI after pose run {%s} run with issue {%s} active {%s}\n",  //
                  pose.button_key.c_str(),                                             //
                  issue ? "yes" : "no",                                                //
                  active ? "yes" : "no");
  UIButton *b = find_button_by_text(pose.name.c_str());
  if (!b) {
    LOG_ERROR("[!] cannot find button for pose {%s}", pose.name.c_str());
  } else {
    drawButtonOverlayById(b->get_id());
  }
  LOG_SECTION_END_POSE();
  return ok;
}

// -----------------------------------------------------------
// Position comparison
// -----------------------------------------------------------
bool PoseStore::is_at_pose(const char *key, double tol_mm, double tol_deg) {
  int idx = find_pose_index(key);
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
  LOG_SECTION_START_POSE("PoseStore::init_from_defaults");

  for (int i = 0; i < def_count; i++) {
    const Pose &def = defaults[i];
    double val_p1 = def.p1;

    add_pose(def.key.c_str(), def.name.c_str(), def.move_type.c_str(), val_p1,
             def.button_key.c_str(), def.step, def.min_val, def.max_val);
  }

  list_poses();
  LOG_SECTION_END_POSE();
}

void PoseStore::reflect_poses_last_run() {
  LOG_SECTION_START_MENU("PoseStore::reflect_poses_last_run");
  for (int i = 0; i < count; i++) {
    const Pose &p = poses_list[i];
    bool issue = !p.last_run_ok;
    bool active = is_at_pose(p.button_key.c_str(), 0.5, 1.0);
    if (!issue) active = 0;
    UIButton *b = find_button_by_text(p.name.c_str());
    if (!b) {
      LOG_ERROR("[!] trying to update issue and active state for non existing button {%s}\n", p.button_key.c_str());
    } else {
      b->set_has_issue(issue);
      b->set_is_active(active);
      drawButtonOverlayById(b->get_id());
    }
  }
  LOG_SECTION_END_MENU();
}

void PoseStore::set_all_poses_last_run(bool b) {
  LOG_SECTION_START_MENU("PoseStore::set_all_poses_last_run", "");
  for (int i = 0; i < count; i++) {
    Pose &p = poses_list[i];
    p.last_run_ok = b;
    bool issue = !b;
    bool active = is_at_pose(p.button_key.c_str(), 0.5, 1.0);
    if (!issue) active = false;
    UIButton *b = find_button_by_text(p.name.c_str());
    if (!b) {
      LOG_ERROR("[!] trying to update issue and active state for non existing button {%s}\n", p.button_key.c_str());
    } else {
      b->set_has_issue(issue);
      b->set_is_active(active);
      drawButtonOverlayById(b->get_id());
    }
  }
  LOG_SECTION_END_MENU();
}

void PoseStore::update_pose_store_from_param_store(const Pose *defaults, int def_count) {
  LOG_SECTION_START_POSE("PoseStore::update_pose_store_from_param_store");

  for (int i = 0; i < def_count; i++) {
    const Pose &def = defaults[i];

    // --- Read from param store
    double stored_val = getParamValue(def.key.c_str());

    // --- getParamValue() returns PARAM_VAL_NA if not found,
    // so we’ll only apply if it’s non-zero and different from default
    if (stored_val < PARAM_VAL_NA && stored_val != 0.0) {  //TODO for now given the storage has many 0.0
      double restored_p1 = stored_val;                     // convert back from scaled int
      int idx = find_pose_index(def.key.c_str());

      if (idx >= 0) {
        Pose &p = poses_list[idx];
        p.p1 = restored_p1;

        LOG_PRINTF_POSE("value from flash | pose {%s} | val {%.2f}\n",
                        def.key.c_str(), restored_p1);
      } else {
        LOG_ERROR("pose not found in flash {%s}", def.key.c_str());
      }
    } else {
      LOG_PRINTF_POSE("keeping default | pose {%s} | default {%.2f}\n",
                      def.key.c_str(), def.p1);
    }
  }
  LOG_SECTION_END_POSE();
}

// -----------------------------------------------------------
// Debug list
// -----------------------------------------------------------
void PoseStore::list_poses() const {
  LOG_SECTION_START_POSE("PoseStore::list_poses");
  for (int i = 0; i < count; i++) {
    const Pose &p = poses_list[i];
    LOG_PRINTF_POSE("(%2d) | key {%-10s} | name {%-10s} | type {%-8s} | p1 {%.2f} | step {%.2f} | min {%.2f} | max {%.2f} | btn {%s} | last {%s}\n",
                    i, p.key.c_str(), p.name.c_str(), p.move_type.c_str(), p.p1,
                    p.step, p.min_val, p.max_val, p.button_key.c_str(), p.last_run_ok ? "true" : "false");
  }
  LOG_SECTION_END_POSE();
}

// ============================================================
// Default poses_list (needs to match the menus)
// ============================================================
/*
  p.name = name;
  p.key = key;
  p.move_type = type;
  p.p1 = p1;
  p.button_key = btn_key ? btn_key : "";
  p.step = step;
  p.min_val = min_val;
  p.max_val = max_val;
  */

Pose default_poses[] = {

  // XY poses list
  { "y zero", "y zero", "y", 40.0, "y zero", 0.5, 40.0, 110.0 },
  { "y 1st", "y 1st", "y", 50.0, "y 1st", 0.5, 40.0, 110.0 },
  { "y 2nd", "y 2nd", "y", 60.0, "y 2nd", 0.5, 40.0, 110.0 },
  { "y 3rd", "y 3rd", "y", 70.0, "y 3rd", 0.5, 40.0, 110.0 },

  { "y c2", "y c2", "y", 50.0, "y c2", 0.5, 40.0, 110.0 },
  { "y c3", "y c3", "y", 60.0, "y c3", 0.5, 40.0, 110.0 },

  { "x center", "x center", "x", 0.1, "x center", 0.5, -25.0, 25.0 },  // 0.1 to avoid the 0.0 meaning the value in the storage
  { "x left", "x left", "x", -25.0, "x left", 0.5, -35.0, 0.0 },
  { "x right", "x right", "x", 25.0, "x right", 0.5, 0.0, 35.0 },

  // Combined grippers
  { "grippers open", "grippers open", "grippers", 80.0, "grippers open", 0.5, 0.0, 100.0 },
  { "grippers close", "grippers close", "grippers", 10.0, "grippers close", 0.5, 0.0, 100.0 },

  // Individual gripper 1
  { "gripper 1 open", "gripper 1 open", "gripper1", 80.0, "gripper 1 open", 0.5, 0.0, 100.0 },
  { "gripper 1 close", "gripper 1 close", "gripper1", 10.0, "gripper 1 close", 0.5, 0.0, 100.0 },

  // Individual gripper 2
  { "gripper 2 open", "gripper 2 open", "gripper2", 80.0, "gripper 2 open", 0.5, 0.0, 100.0 },
  { "gripper 2 close", "gripper 2 close", "gripper2", 10.0, "gripper 2 close", 0.5, 0.0, 100.0 },

  // Wrist poses list
  { "wrist vert", "wrist vert", "wrist", 0.1, "wrist vert", 0.5, -45.0, 45.0 },  // 0.1 to avoid the 0.0 meaning the value in the storage
  { "wrist horiz left", "wrist horiz left", "wrist", -90.0, "wrist horiz left", 0.5, 45.0, 135.0 },
  { "wrist horiz right", "wrist horiz right", "wrist", 90.0, "wrist horiz right", 0.5, 135.0, 205.0 },

  // Base rotation
  { "base front", "base front", "base", 0.1, "base front", 0.5, -45.0, 45.0 },  // 0.1 to avoid the 0.0 meaning the value in the storage
  { "base right", "base right", "base", 90.0, "base right", 0.5, 45.0, 135.0 },
  { "base left", "base left", "base", -90.0, "base left", 0.5, 135.0, 205.0 }
};

const int DEFAULT_POSE_COUNT = sizeof(default_poses) / sizeof(default_poses[0]);

PoseStore pose_store;

bool initPoseStore() {
  pose_store.init_from_defaults(default_poses, DEFAULT_POSE_COUNT);
  pose_store.update_pose_store_from_param_store(default_poses, DEFAULT_POSE_COUNT);
  return true;
}

bool updatePoseStoreFromParamStore() {
  pose_store.update_pose_store_from_param_store(default_poses, DEFAULT_POSE_COUNT);
  return true;
}
