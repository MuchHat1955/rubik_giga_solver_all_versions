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
  bool solve_a2_y_from_a1_x(double _a1_center_deg, double _x);
  bool solve_a2_x_from_a1_y(double _a1_center_deg, double _y);
  bool solve_a1_a2_from_x_y(double _x, double _y);
  bool solve_x_y_from_a1_a2(double _a1_center_deg, double _a2_center_deg);

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
  double getGdeg_for_horizontal() const;
  double getGdeg_for_vertical() const;
  double getGdeg_closest_aligned() const;
  int getGticks() const;
  int getGticks_closest_aligned() const;

  // ---------------- Motion utilities ----------------
  void readPresentPositions();
  void setGoalPositions(double x, double y, double gdeg);

  // ---------------- Geometry solver ----------------
  void solve_deg_angles_from_xy(double x_mm, double y_mm, double &a1r, double &a2r);

  // ---------------- Gripper alignment ----------------
  void update_g_alignment();

private:
  double a1_center_deg = 0.0;
  double a2_center_deg = 0.0;
  double g_center_deg = 0.0;
  double x_mm = 0.0;
  double y_mm = 0.0;
  bool g_closest_horizontal = false;

  // ---------------- Internal forward solver ----------------
  void update_from_angles();
};

// -------------------------------------------------------------------
//                 Global instance and constants
// -------------------------------------------------------------------

extern VerticalKinematics kin;
extern float l_mm;
