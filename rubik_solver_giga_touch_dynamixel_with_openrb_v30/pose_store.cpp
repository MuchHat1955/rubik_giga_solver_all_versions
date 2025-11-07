#include "pose_store.h"
#include "param_store.h"
#include "logging.h"
#include "rb_interface.h"

extern RBInterface rb;

// -----------------------------------------------------------
// Add or update a pose
// -----------------------------------------------------------
bool PoseStore::add_pose(const char *name, const char *type, double p1, double p2,
                         const char *btn_key, double step, double min_val,
                         double max_val, int servo_id) {
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
  p.servo_id = servo_id;

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
bool PoseStore::get_pose_params(const char *name, double *p1, double *p2, String *type) const {
  int idx = find_pose_index(name);
  if (idx < 0) return false;
  if (p1) *p1 = poses_list[idx].p1;
  if (p2) *p2 = poses_list[idx].p2;
  if (type) *type = poses_list[idx].move_type;
  return true;
}

bool PoseStore::set_pose_params(const char *name, double p1, double p2) {
  int idx = find_pose_index(name);
  if (idx < 0) return false;
  Pose &p = poses_list[idx];
  p.p1 = p1;
  p.p2 = p2;
  setParamValue((String("pose_") + name + "_p1").c_str(), (int)(p1*100.0));
  return true;
}

// -----------------------------------------------------------
// Increment / Decrement
// -----------------------------------------------------------
bool PoseStore::increment_pose_param(const char *name, int units, double &new_value_ref) {
  int idx = find_pose_index(name);
  if (idx < 0) return false;

  Pose &p = poses_list[idx];
  double new_val = p.p1 + (units * p.step);
  if (new_val < p.min_val) new_val = p.min_val;
  if (new_val > p.max_val) new_val = p.max_val;

  p.p1 = new_val;
  setParamValue((String("pose_") + name + "_p1").c_str(), (int)(new_val * 100.0));
  new_value_ref = new_val;
  LOG_PRINTF("pose{%s} adjusted to{%.2f} units{%d}\n", name, new_val, units);
  return true;
}

// -----------------------------------------------------------
// Stub helpers for RB zero operations
// -----------------------------------------------------------
bool setZeroServo(int servo_id, double value) {
  // TODO
  return false;
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

  // Handle SETZERO
  if (pose.servo_id >= 0 && pose.name.endsWith("_0")) {
    // return setZeroServo(pose.servo_id, p1); //TODO
    return false;
  }

  LOG_SECTION_START_PRINTF("pose store run_pose", "| {%s} type{%s}", pose_name, type.c_str());
  bool ok = false;

  if (type == "xy") {
    ok = rb.moveXmm(p1) && rb.moveYmm(p2);
  } else if (type == "servo" || type == "base") {
    ok = rb.moveBaseDeg(p1);
  } else if (type == "wrist") {
    ok = rb.moveWristVertDeg(p1);
  } else if (type == "gripper") {
    ok = rb.moveGrippersPer(p1);
  } else {
    LOG_PRINTF("nnknown move type:{%s}\n", type.c_str());
  }

  LOG_PRINTF("pose store run pose result {%d}", ok);
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

  if (p.move_type == "xy") {
    rb.xyInfoMm(&val1, &val2);
    return fabs(val1 - p.p1) < tol_mm && fabs(val2 - p.p2) < tol_mm;
  } else if (p.move_type == "servo" || p.move_type == "base") {
    double b;
    rb.baseInfoDeg(&b);
    return fabs(b - p.p1) < tol_deg;
  } else if (p.move_type == "wrist") {
    double w;
    rb.wristVertInfoDeg(&w);
    return fabs(w - p.p1) < tol_deg;
  } else if (p.move_type == "gripper") {
    double g;
    rb.gripperInfoPer(&g);
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

    if (def.servo_id >= 0 && def.name.endsWith("_0")) {
  //    if (!zeroInfoMm(def.servo_id, &val_p1)) //TODO
   //     LOG_PRINTF("GETZERO failed for{%s}, using default{%.2f}\n", def.name.c_str(), def.p1);
    } else {
      String key = String("pose_") + def.name + "_p1";
      val_p1 = (double)getParamValue(key.c_str()) / 100.0;
    }

    add_pose(def.name.c_str(), def.move_type.c_str(), val_p1, val_p2,
             def.button_key.c_str(), def.step, def.min_val, def.max_val, def.servo_id);
  }

  list_poses();
  LOG_SECTION_END();
}

// -----------------------------------------------------------
// Debug list
// -----------------------------------------------------------
void PoseStore::list_poses() const {
  LOG_SECTION_START("PoseStore::list_poses");
  for (int i = 0; i < count; i++) {
    const Pose &p = poses_list[i];
    LOG_PRINTF("(%2d) name{%-10s} | typr{%-8s} | p1{%.2f} p2{%.2f} | id{%d} step{%.2f} min{%.2f} max{%.2f} btn{%s}\n",
               i, p.name.c_str(), p.move_type.c_str(), p.p1, p.p2,
               p.servo_id, p.step, p.min_val, p.max_val, p.button_key.c_str());
  }
  LOG_SECTION_END();
}

// Servo IDs (adjust as needed for your setup)
#define SERVO_ID_BASE 16
#define SERVO_ID_WRIST 13
#define SERVO_ID_GRIP1 14
#define SERVO_ID_GRIP2 15

// ============================================================
// Default poses_list (auto-generated from menu keys)
// ============================================================
Pose default_poses[] = {

  // XY poses_list
  { "y_0", "xy", 0.0, 0.0, "xy_0_btn", 0.5, -100.0, 200.0, -1 },
  { "y_2nd", "xy", 10.0, 0.0, "xy_2nd_btn", 0.5, -100.0, 200.0, -1 },
  { "y_3rd", "xy", 20.0, 0.0, "xy_3rd_btn", 0.5, -100.0, 200.0, -1 },
  { "xy_c1", "xy", 30.0, 0.0, "xy_c1_btn", 0.5, -100.0, 200.0, -1 },
  { "xy_c2", "xy", 40.0, 0.0, "xy_c2_btn", 0.5, -100.0, 200.0, -1 },
  { "xy_c3", "xy", 50.0, 0.0, "xy_c3_btn", 0.5, -100.0, 200.0, -1 },
  { "xy_c4", "xy", 60.0, 0.0, "xy_c4_btn", 0.5, -100.0, 200.0, -1 },
  { "xy_c5", "xy", 70.0, 0.0, "xy_c5_btn", 0.5, -100.0, 200.0, -1 },
  { "xy_c6", "xy", 80.0, 0.0, "xy_c6_btn", 0.5, -100.0, 200.0, -1 },

  // Combined grippers
  { "grips_open", "grips", 0.0, 0.0, "grips_open_btn", 1.0, 0.0, 100.0, -1 },
  { "grips_close", "grips", 60.0, 0.0, "grips_close_btn", 1.0, 0.0, 100.0, -1 },

  // Individual gripper 1
  { "grip1_open", "grips", 0.0, 0.0, "grip1_open_btn", 1.0, 0.0, 100.0, SERVO_ID_GRIP1 },
  { "grip1_close", "grips", 60.0, 0.0, "grip1_close_btn", 1.0, 0.0, 100.0, SERVO_ID_GRIP1 },

  // Individual gripper 2
  { "grip2_open", "grips", 0.0, 0.0, "grip2_open_btn", 1.0, 0.0, 100.0, SERVO_ID_GRIP2 },
  { "grip2_close", "grips", 60.0, 0.0, "grip2_close_btn", 1.0, 0.0, 100.0, SERVO_ID_GRIP2 },

  // Wrist poses_list
  { "wrist_90", "wrist", 0.0, 0.0, "wrist_90_btn", 1.0, -180.0, 180.0, SERVO_ID_WRIST },
  { "wrist_0", "wrist", 90.0, 0.0, "wrist_0_btn", 1.0, -180.0, 180.0, SERVO_ID_WRIST },
  { "wrist_90minus", "wrist", 90.0, 0.0, "wrist_90minus_btn", 1.0, -180.0, 180.0, SERVO_ID_WRIST },

  // Base rotation
  { "base_0", "servo", 0.0, 0.0, "base_0_btn", 1.0, -180.0, 180.0, SERVO_ID_BASE },
  { "base_90", "servo", 90.0, 0.0, "base_90_btn", 1.0, -180.0, 180.0, SERVO_ID_BASE },
  { "base_90minus", "servo", -90.0, 0.0, "base_90minus_btn", 1.0, -180.0, 180.0, SERVO_ID_BASE }
};

const int DEFAULT_POSE_COUNT = sizeof(default_poses) / sizeof(default_poses[0]);

PoseStore pose_store;

bool initPoseStore() {
  pose_store.init_from_defaults(default_poses, DEFAULT_POSE_COUNT);
  return true; //TODO
}
