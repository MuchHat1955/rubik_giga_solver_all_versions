#pragma once
#include <Arduino.h>
#include "rb_interface.h"
#include "param_store.h"
#include "logging.h"

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
  PoseStore(RBInterface &rb_ref, ParamStore &param_ref);

  bool add_pose(const char *name, const char *type, double p1, double p2,
                const char *btn_key = nullptr, double step = 1.0,
                double min_val = -9999.0, double max_val = 9999.0,
                int servo_id = -1);

  bool is_pose(const char *name) const;
  bool is_button_for_pose(const char *btn_key) const;

  bool get_pose_params(const char *name, double *p1, double *p2, String *type = nullptr) const;
  bool set_pose_params(const char *name, double p1, double p2);

  bool run_pose(const char *name);
  bool run_pose_by_button(const char *btn_key);

  bool increment_pose_param(const char *name, int units, double &new_value_ref);
  bool is_at_pose(const char *name, double tol_mm = 0.5, double tol_deg = 1.0);

  void init_from_defaults(const Pose *defaults, int count);
  void list_poses() const;

private:
  static const int MAX_POSES = 64;
  Pose poses[MAX_POSES];
  int count = 0;

  RBInterface &rb;
  ParamStore &params;

  int find_pose_index(const char *name) const;
  int find_pose_by_button(const char *btn_key) const;
};

void initPoseStore();
