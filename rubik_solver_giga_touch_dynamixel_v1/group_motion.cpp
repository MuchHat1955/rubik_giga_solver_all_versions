// group_motion.cpp
#include "group_motion.h"
#include "servos.h"
#include <Arduino.h>
#include <vector>
#include <string>
#include <cmath>

bool runGroupPoseServos(const std::vector<std::string>& servos,
                        const std::vector<int>& targets)
{
  if(servos.empty() || servos.size()!=targets.size()) return false;
  const int steps = 30;
  int n=servos.size();
  std::vector<int> start(n);
  for(int i=0;i<n;i++) start[i]=readServoTicks(servos[i]);

  for(int s=0;s<=steps;s++){
    float t=(float)s/steps;
    float eased = 0.5f*(1.f - cosf(3.1415926f*t));
    for(int i=0;i<n;i++){
      int pos = start[i] + (int)((targets[i]-start[i])*eased);
      // TODO: add a setGoalTicks(name, pos) here if you’d like finer control.
      // For brevity, we just set base angle via rotateBaseToAngle() when needed,
      // and rely on your higher-level sequences for others.
      if (servos[i]=="base") {
        // Convert ticks back to degrees using base_0/base_90 if you prefer; here skip.
      }
      // If you want per-servo direct control, expose a helper to dxl.setGoalPosition(id,pos).
    }
    delay(15);
  }
  return true;
}

bool moveArmsVertically(float mm) {
  // Example placeholder — replace with your kinematic math later

  // Each mm of vertical lift corresponds to some tick delta
  // You can later calibrate the scale factor.
  const float mm_per_tick = 0.5f;  // <-- tune this
  int delta = (int)(mm / mm_per_tick);

  int arm1 = readServoTicks("arm1");
  int arm2 = readServoTicks("arm2");

  // Move both arms together by the same delta
  int arm1_target = arm1 + delta;
  int arm2_target = arm2 + delta;

  // Keep wrist horizontal by computing its compensating ticks if needed
  int wrist_target = getServoTicksForAngle("wrist", 0.0f, arm1_target, arm2_target);

  std::vector<std::string> names = {"arm1", "arm2", "wrist"};
  std::vector<int> values = {arm1_target, arm2_target, wrist_target};

  // Execute the synchronized motion
  return runGroupPoseServos(names, values);
}
