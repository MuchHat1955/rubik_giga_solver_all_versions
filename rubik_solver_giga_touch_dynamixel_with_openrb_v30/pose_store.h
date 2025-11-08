#pragma once
#include "utils.h"

// -----------------------------------------------------------
// Single pose entry
// -----------------------------------------------------------
struct Pose {
  String name;       // e.g. "wrist_0"
  String move_type;  // "xy", "servo", "gripper", "wrist"
  double p1;
  double p2;
  String button_key;  // LVGL button key
  double step;
  double min_val;
  double max_val;
  int servo_id;  // servo ID for zero calibration
};

// -----------------------------------------------------------
// PoseStore class
// -----------------------------------------------------------
class PoseStore {
public:
  PoseStore() {
    count = 0;
  };

  bool add_pose(const char *name, const char *type, double p1, double p2,
                const char *btn_key = nullptr, double step = 1.0,
                double min_val = -9999.0, double max_val = 9999.0,
                int servo_id = -1);

  bool is_pose(const char *name) const;
  bool is_button_for_pose(const char *btn_key) const;

  bool get_pose_params(const char *name, double *p1, double *p2) const;
  bool set_pose_params(const char *name, double p1, double p2);

  bool run_pose(const char *name);
  bool run_pose_by_button(const char *btn_key);

  void set_pose_val(const char *param_name, double val);
  char *btn_to_pose(const char *btn_name) const;
  bool is_btn_for_pose(const char *btn_key) const;
  bool is_param_for_pose(const char *param_name) const;
  char *param_to_pose(const char *param_name) const;
  bool increment_pose_param(const char *param_name, int units, double &new_value_ref);

  bool is_at_pose(const char *name, double tol_mm = 0.5, double tol_deg = 1.0);

  void init_from_defaults(const Pose *defaults, int count);
  void list_poses() const;

  static const int MAX_POSES = 64;
  Pose poses_list[MAX_POSES];
  int count = 0;

  int find_pose_index(const char *name) const;
  int find_pose_by_button(const char *btn_key) const;
};

bool initPoseStore();
