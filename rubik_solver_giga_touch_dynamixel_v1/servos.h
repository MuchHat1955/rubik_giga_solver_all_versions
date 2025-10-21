// servos.h
#pragma once
#include <string>
int  readServoTicks(const std::string& name);
int  getServoTicksForAngle(const std::string& servo, float angle_deg,
                           int arm1_ticks_for_comp=-1, int arm2_ticks_for_comp=-1);
void rotateBaseToAngle(int angle_deg);
void dynamixel_begin();
