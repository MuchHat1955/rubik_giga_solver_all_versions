#include "vertical_kinematics.h"

// -------------------------------------------------------------------
//                    GLOBAL LINK LENGTH
// -------------------------------------------------------------------
float arm_length_mm = 60.0f;

// -------------------------------------------------------------------
//                 VERTICAL KINEMATICS IMPLEMENTATION
// -------------------------------------------------------------------

VerticalKinematics::VerticalKinematics() {}

// ---------------- Independent setters ----------------
void VerticalKinematics::setA1deg(double a1) {
  a1_deg = a1;
  update_from_angles();
}

void VerticalKinematics::setA2deg(double a2) {
  a2_deg = a2;
  update_from_angles();
}

void VerticalKinematics::setGdeg(double g) {
  g_deg = g;
  update_g_alignment();
}

void VerticalKinematics::setA1ticks(int ticks) {
  a1_deg = ticks2deg(ID_ARM1, ticks);
  update_from_angles();
}

void VerticalKinematics::setA2ticks(int ticks) {
  a2_deg = ticks2deg(ID_ARM2, ticks);
  update_from_angles();
}

void VerticalKinematics::setGticks(int ticks) {
  g_deg = ticks2deg(ID_GRIP, ticks);
  update_g_alignment();
}

// -------------------------------------------------------------------
// setXYmm(x, y)
//   • setXYmm(0, Y)   -> vertical up/down at given Y
//   • setXYmm(X, -1)  -> horizontal offset X, preserving current Y
//   • full inverse kinematics (updates both angles)
// -------------------------------------------------------------------
void VerticalKinematics::solve_deg_angles_from_xy(double x_mm, double y_mm,
                                                  double &a1r, double &a2r) {
  double A = x_mm / arm_length_mm;
  double B = y_mm / arm_length_mm;

  // step 1: compute q
  double q = atan2(-A, B);

  // step 2: compute sin(p)
  double cos_q = cos(q);
  double sin_p = 0.0;
  if (fabs(cos_q) > 1e-9) {
    sin_p = clamp(B / (2.0 * cos_q), -1.0, 1.0);
  }
  if (sin_p > 1.0) sin_p = 1.0;
  if (sin_p < -1.0) sin_p = -1.0;

  double p = asin(sin_p);

  // step 3: compute angles
  double a1abs_right_rad = p + q;
  double a2abs_left_rad = p - q;

  serial_printf("a1 abs right rad=%.2f a2 abs left rad=%.2f\n",
                a1abs_right_rad, a2abs_left_rad);

  double a1abs_right_deg = rad2deg(a1abs_right_rad);
  double a2abs_left_deg = rad2deg(a2abs_left_rad);

  serial_printf("a1 abs right deg=%.2f a2 abs left deg=%.2f\n",
                a1abs_right_deg, a2abs_left_deg);

  a1r = 90 - a1abs_right_deg;
  a2r = a2abs_left_deg + a1abs_right_deg - 90;

  serial_printf("a1 rel deg=%.2f a2 rel deg=%.2f\n", a1r, a2r);
}

void VerticalKinematics::setXYmm(double x, double y) {
  if (x < 0 && y < 0) return;
  if (x < 0) x = x_mm;
  if (y < 0) y = y_mm;

  solve_deg_angles_from_xy(x, y, a1_deg, a2_deg);
  update_from_angles();
}

// -------------------------------------------------------------------
//                           GETTERS
// -------------------------------------------------------------------
double VerticalKinematics::getA1deg() const {
  return a1_deg;
}
double VerticalKinematics::getA2deg() const {
  return a2_deg;
}

int VerticalKinematics::getA1ticks() const {
  return deg2ticks(ID_ARM1, a1_deg);
}
int VerticalKinematics::getA2ticks() const {
  return deg2ticks(ID_ARM2, a2_deg);
}

double VerticalKinematics::getXmm() const {
  return x_mm;
}
double VerticalKinematics::getYmm() const {
  return y_mm;
}

double VerticalKinematics::getGdeg() const {
  return g_deg;
}

double VerticalKinematics::getGdeg_for_vertical() const {  //TODO check
  // 0° = gripper vertical; positive tilts along Arm2
  return 180 + a2_deg - a1_deg;
}

double VerticalKinematics::getGdeg_for_horizontal() const {  //TODO check
  // 0° = gripper vertical; positive tilts along Arm2
  return 90 + a2_deg - a1_deg;
}

double VerticalKinematics::getGdeg_closest_aligned() const {
  // 0° = gripper vertical; positive tilts along Arm2
  if (g_closest_horizontal) return getGdeg_for_horizontal();
  return getGdeg_for_vertical();
}

int VerticalKinematics::getGticks() const {
  return deg2ticks(ID_GRIP, getGdeg());
}
int VerticalKinematics::getGticks_closest_aligned() const {
  return deg2ticks(ID_GRIP, getGdeg_closest_aligned());
}

// -------------------------------------------------------------------
//                     GRIPPER ALIGNMENT
// -------------------------------------------------------------------
void VerticalKinematics::update_g_alignment() {
  double g_vert_deg = getGdeg_for_vertical();
  double g_horiz_deg = getGdeg_for_horizontal();
  double err_vert = fabs(g_deg - g_vert_deg);
  double err_horiz = fabs(g_deg - g_horiz_deg);
  if (err_horiz < err_vert)
    g_closest_horizontal = true;
  else
    g_closest_horizontal = false;
}

// -------------------------------------------------------------------
//                     READ & GOAL POSITIONS
// -------------------------------------------------------------------
void VerticalKinematics::readPresentPositions() {
  setA1ticks(dxl.getPresentPosition(ID_ARM1));
  setA2ticks(dxl.getPresentPosition(ID_ARM2));
  setGticks(dxl.getPresentPosition(ID_GRIP));
}

void VerticalKinematics::setGoalPositions(double x, double y, double gdeg) {
  setXYmm(x, y);
  setGdeg(gdeg);
  update_from_angles();

  dxl.setGoalPosition(deg2ticks(ID_ARM1, a1_deg),
                      deg2ticks(ID_ARM2, a1_deg),
                      deg2ticks(ID_GRIP, getGdeg()));
}

// -------------------------------------------------------------------
//                     FORWARD KINEMATICS
// -------------------------------------------------------------------
void VerticalKinematics::update_from_angles() {
  double a1r_rad = deg2rad(a1_deg);
  double a2r_rad = deg2rad(a2_deg);
  double a1abs_right_rad = M_PI / 2.0 - a1r_rad;
  double a2abs_left_rad = a1r_rad + a2r_rad;

  // compute using absolute, 0 is vertical, 90 is horizontal right
  y_mm = arm_length_mm * sin(a1abs_right_rad) + arm_length_mm * sin(a2abs_left_rad);
  x_mm = arm_length_mm * cos(a1abs_right_rad) - arm_length_mm * cos(a2abs_left_rad);
}

// -------------------------------------------------------------------
//                   GLOBAL INSTANCE
// -------------------------------------------------------------------
VerticalKinematics kin;
