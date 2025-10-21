// servos.h
#pragma once
#include <string>
#include <Dynamixel2Arduino.h>

extern Dynamixel2Arduino dxl;

#define XL430_360DEG_TICKS 4036

int  readServoTicks(const std::string& name);
int  getServoTicksForAngle(const std::string& servo, float angle_deg);
void rotateToAngle(const std::string& servo, int angle_deg);
void dynamixel_begin();
