#pragma once
#include <Arduino.h>
#include "servos.h"
#include "utils.h"

// -------------------------------------------------------------------
//                  VERTICAL KINEMATICS CLASS - DECLARATION
// -------------------------------------------------------------------

class VerticalKinematics {
public:
  VerticalKinematics();

  // ---------------- Independent setters ----------------
  void setA1deg(double a1);
  void setA2deg(double a2);
  void setGdeg(double g);
  void setA1ticks(int ticks);
  void setA2ticks(int ticks);
  void setGticks(int ticks);

  // ---------------- XY setters ----------------
  void setXYmm(double x, double y);

  // ---------------- Getters ----------------
  double getA1deg() const;
  double getA2deg() const;
  int getA1ticks() const;
  int getA2ticks() const;

  double getXmm() const;
  double getYmm() const;

  double getGdeg() const;
  double getGdeg_horizontal() const;
  double getGdeg_vertical() const;
  double getGdeg_aligned() const;
  int getGticks() const;
  int getGticksAligned() const;

  // ---------------- Motion utilities ----------------
  void readPresentPositions();
  void setGoalPositions(double x, double y, double gdeg);

  // ---------------- Geometry solver ----------------
  void solve_deg_angles_from_xy(double x_mm, double y_mm, double &a1r, double &a2r);

  // ---------------- Gripper alignment ----------------
  void update_g_alignment();

private:
  double a1_deg = 0.0;
  double a2_deg = 0.0;
  double g_deg = 0.0;
  double x_mm = 0.0;
  double y_mm = 0.0;
  bool g_horizontal = false;

  // ---------------- Internal forward solver ----------------
  void update_from_angles();
};

// -------------------------------------------------------------------
//                 Global instance and constants
// -------------------------------------------------------------------

extern VerticalKinematics kin;
extern float arm_length_mm;
