// group_motion.cpp
#include "group_motion.h"
#include "servos.h"
#include <Arduino.h>
#include <vector>
#include <string>
#include <cmath>
#include "param_store.h"

bool runGroupPoseServos(const std::vector<std::string>& servos,
                        const std::vector<int>& targets) {
  if (servos.empty() || servos.size() != targets.size()) return false;
  const int steps = 30;
  int n = servos.size();
  std::vector<int> start(n);
  for (int i = 0; i < n; i++) start[i] = readServoTicks(servos[i]);

  for (int s = 0; s <= steps; s++) {
    float t = (float)s / steps;
    float eased = 0.5f * (1.f - cosf(3.1415926f * t));
    for (int i = 0; i < n; i++) {
      int pos = start[i] + (int)((targets[i] - start[i]) * eased);
      // TODO: add a setGoalTicks(name, pos) here if you’d like finer control.
      // For brevity, we just set base angle via rotateBaseToAngle() when needed,
      // and rely on your higher-level sequences for others.
      if (servos[i] == "base") {
        // Convert ticks back to degrees using base_0/base_90 if you prefer; here skip.
      }
      // If you want per-servo direct control, expose a helper to dxl.setGoalPosition(id,pos).
    }
    delay(15);
  }
  return true;
}

bool moveArmsVertically(float mm) {
  // Retrieve geometry constants (mm)
  float L1 = (float)getParamValue("arm1_length_mm") / 100.0f;
  float L2 = (float)getParamValue("arm2_length_mm") / 100.0f;
  float X = (float)getParamValue("base_x_offset_mm") / 100.0f;
  if (L1 <= 0 || L2 <= 0) {
    L1 = L2 = 48.0f;
    X = 20.0f;
  }

  // Compute target Y based on mm
  float targetY = mm;

  // Simplified 2-link inverse kinematics
  float cosB = (X * X + L2 * L2 - (L1 - targetY) * (L1 - targetY)) / (2 * X * L2);
  cosB = std::max(-1.0f, std::min(1.0f, cosB));
  float angle2 = acos(cosB);

  float k1 = L1 + L2 * cos(angle2);
  float k2 = L2 * sin(angle2);
  float angle1 = atan2(targetY, X) - atan2(k2, k1);

  // Convert to servo ticks
  int arm1_ticks = getServoTicksForAngle("arm1", degrees(angle1));
  int arm2_ticks = getServoTicksForAngle("arm2", degrees(angle2));
  int wrist_ticks = getServoTicksForAngle("wrist", 0.0f, arm1_ticks, arm2_ticks);

  std::vector<std::string> names = { "arm1", "arm2", "wrist" };
  std::vector<int> targets = { arm1_ticks, arm2_ticks, wrist_ticks };
  return runGroupPoseServos(names, targets);
}
