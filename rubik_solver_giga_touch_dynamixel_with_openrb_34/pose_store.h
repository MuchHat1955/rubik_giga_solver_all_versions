#pragma once
#include "utils.h"

// -----------------------------------------------------------
// Single pose entry
// -----------------------------------------------------------
struct Pose {
  String name;       // e.g. "wrist_0"
  String move_type;  // "xy", "gripper", "wrist"
  double p1;
  String button_key;  // LVGL button key
  double step;
  double min_val;
  double max_val;
  bool last_run_ok;
};

// -----------------------------------------------------------
// PoseStore class
// -----------------------------------------------------------
class PoseStore {
public:
  PoseStore() {
    count = 0;
  };

  bool add_pose(const char *name, const char *type, double p1,
                const char *btn_key = nullptr, double step = 1.0,
                double min_val = -9999.0, double max_val = 9999.0);

  bool is_pose(const char *name) const;
  bool is_button_for_pose(const char *btn_key) const;

  bool get_pose_params(const char *name, double *p1) const;
  bool set_pose_params(const char *name, double p1);

  bool run_pose(const char *name);
  bool run_pose_by_button(const char *btn_key);

  void set_pose_val_from_param(const char *param_name, double val);
  char *btn_to_pose(const char *btn_name) const;
  bool is_btn_for_pose(const char *btn_key) const;
  bool is_param_for_pose(const char *param_name) const;
  char *param_to_pose(const char *param_name) const;
  bool increment_pose_param(const char *param_name, int units, double &new_value_ref);

  bool is_at_pose(const char *name, double tol_mm = 0.5, double tol_deg = 1.0);

  void init_from_defaults(const Pose *defaults, int count);
  void update_pose_store_from_param_store(const Pose *defaults, int def_count);
  void reflect_poses_ui();
  void set_all_poses_last_run(bool b);
  void list_poses() const;

  static const int MAX_POSES = 64;
  Pose poses_list[MAX_POSES];
  int count = 0;

  int find_pose_index(const char *name) const;
  int find_pose_by_button(const char *btn_key) const;
};

bool initPoseStore();
