#include "vertical_kinematics.h"

// -------------------------------------------------------------------
//                    HELPERS
// -------------------------------------------------------------------
double l_mm = 53.0;
double _90_rad = M_PI / 2.0;
double _180_rad = M_PI;

static inline double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static constexpr double EPS = 1e-9;

// Convenience tight bounds
bool x_in_bounds(double x) {
  bool b = (x >= -0.5 * l_mm - EPS) && (x <= 0.5 * l_mm + EPS);
  if (b) return true;
  serial_printf("ERR x_in_bounds x=%.2f\n", x);
  return false;
}
bool y_in_bounds(double y) {
  bool b = (y > 0.5 * l_mm + EPS) && (y < 1.8 * l_mm - EPS);
  if (b) return true;
  serial_printf("ERR y_in_bounds y=%.2f\n", y);
  return false;
}
bool a1_center_deg_in_bounds(double a) {
  bool b = (a > 0.0 + EPS) && (a < 90.0 - EPS);
  if (b) return true;
  serial_printf("ERR a1_center_deg_in_bounds a=%.2f\n", a);
  return false;
}
bool a2_center_deg_in_bounds(double a) {
  bool b = (a > -90.0 + EPS) && (a < 90.0 - EPS);
  if (b) return true;
  serial_printf("ERR a2_center_deg_in_bounds a=%.2f\n", a);
  return false;
}

// ------------------------------------------------------------
// A2 and Y from A1 and X
// ------------------------------------------------------------
bool VerticalKinematics::solve_a2_y_from_a1_x(double _a1_center_deg, double _x) {
  if (!x_in_bounds(_x)) return false;
  if (!a1_center_deg_in_bounds(_a1_center_deg)) return false;

  double _a1_center_rad = deg2rad(_a1_center_deg);
  double _a1_right_rad = _90_rad - _a1_center_rad;

  double _c2 = cos(_a1_right_rad) - _x / l_mm;
  // Must yield a2 in [0, pi/2] → cos in [0,1]
  if (_c2 < -EPS || _c2 > 1.0 + EPS) return false;
  _c2 = clampd(_c2, 0.0, 1.0);

  double _a2_left_rad = acos(_c2);
  if (_a2_left_rad < 0.0 + EPS || _a2_left_rad > _90_rad + EPS) return false;

  double _y = l_mm * (sin(_a1_right_rad) + sin(_a2_left_rad));
  double _a2_center_rad = _a2_left_rad - _a1_center_rad;
  double _a2_center_deg = rad2deg(_a2_center_rad);

  if (!y_in_bounds(_y)) return false;
  if (!a2_center_deg_in_bounds(_a2_center_deg)) return false;

  // final result
  a2_center_deg = _a2_center_deg;
  y_mm = _y;
  return true;
}

// ------------------------------------------------------------
// A2 and X from A1 and Y
// ------------------------------------------------------------
bool VerticalKinematics::solve_a2_x_from_a1_y(double _a1_center_deg, double _y) {
  if (!y_in_bounds(_y)) return false;
  if (!a1_center_deg_in_bounds(_a1_center_deg)) return false;

  double _a1_center_rad = deg2rad(_a1_center_deg);
  double _a1_right_rad = _90_rad - _a1_center_rad;

  double _s2 = (_y / l_mm) - sin(_a1_right_rad);
  // Must yield a2 in [0, pi/2] → sin in [0,1]
  if (_s2 < -EPS || _s2 > 1.0 + EPS) return false;
  _s2 = clampd(_s2, 0.0, 1.0);

  double _a2_left_rad = asin(_s2);
  if (_a2_left_rad < 0.0 || _a2_left_rad > _90_rad + EPS) return false;

  double _x = l_mm * (cos(_a1_right_rad) - cos(_a2_left_rad));
  double _a2_center_rad = _a2_left_rad - _a1_center_rad;
  double _a2_center_deg = rad2deg(_a2_center_rad);

  if (!x_in_bounds(_x)) return false;
  if (!a2_center_deg_in_bounds(_a2_center_deg)) return false;

  // final result
  a2_center_deg = _a2_center_deg;
  x_mm = _x;
  return true;
}

// ------------------------------------------------------------
// X and Y from A1 and A2
// ------------------------------------------------------------
bool VerticalKinematics::solve_x_y_from_a1_a2(double _a1_center_deg, double _a2_center_deg) {
  if (!a1_center_deg_in_bounds(_a1_center_deg)) return false;
  if (!a2_center_deg_in_bounds(_a2_center_deg)) return false;

  double _a1_center_rad = deg2rad(_a1_center_deg);
  double _a2_center_rad = deg2rad(_a2_center_deg);
  double _a1_right_rad = _90_rad - _a1_center_rad;
  double _a2_left_rad = _a1_center_rad + _a2_center_rad;

  if (_a1_right_rad < 0.0 || _a1_right_rad > _90_rad + EPS) return false;
  if (_a2_left_rad < 0.0 || _a2_left_rad > _90_rad + EPS) return false;

  double _x = l_mm * (cos(_a1_right_rad) - cos(_a2_left_rad));
  double _y = l_mm * (sin(_a1_right_rad) + sin(_a2_left_rad));

  if (!x_in_bounds(_x)) return false;
  if (!y_in_bounds(_y)) return false;

  // final result
  x_mm = _x;
  y_mm = _y;
  return true;
}

// ------------------------------------------------------------
// A1 and A2 from X and Y
// ------------------------------------------------------------
bool VerticalKinematics::solve_a1_a2_from_x_y(double _x, double _y) {
  if (!x_in_bounds(_x)) return false;
  if (!y_in_bounds(_y)) return false;

  double A = _x / l_mm;
  double B = _y / l_mm;

  // step 1: compute q
  double q = atan2(-A, B);

  // step 2: compute sin(p)
  double cos_q = cos(q);
  double sin_p = 0.0;
  if (fabs(cos_q) > 1e-9) {
    sin_p = clampd(B / (2.0 * cos_q), -1.0, 1.0);
  }
  double p = asin(sin_p);

  // step 3: compute angles
  double _a1_right_rad = p + q;
  double _a2_left_rad = p - q;

  double _a1_right_deg = rad2deg(_a1_right_rad);
  double _a2_left_deg = rad2deg(_a2_left_rad);

  double _a1_center_deg = 90.0 - _a1_right_deg;
  double _a2_center_deg = _a2_left_deg + _a1_right_deg - 90.0;

  if (!a1_center_deg_in_bounds(_a1_center_deg)) return false;
  if (!a2_center_deg_in_bounds(_a2_center_deg)) return false;

  // final result
  a1_center_deg = _a1_center_deg;
  a2_center_deg = _a2_center_deg;
  return true;
}

// -------------------------------------------------------------------
//                 VERTICAL KINEMATICS IMPLEMENTATION
// -------------------------------------------------------------------

VerticalKinematics::VerticalKinematics() {}

// -------------------------------------------------------------------
//                           GETTERS
// -------------------------------------------------------------------
double VerticalKinematics::getA1deg() const {
  return a1_center_deg;
}
double VerticalKinematics::getA2deg() const {
  return a2_center_deg;
}

int VerticalKinematics::getA1ticks() const {
  return deg2ticks(ID_ARM1, a1_center_deg);
}
int VerticalKinematics::getA2ticks() const {
  return deg2ticks(ID_ARM2, a2_center_deg);
}

double VerticalKinematics::getXmm() const {
  return x_mm;
}
double VerticalKinematics::getYmm() const {
  return y_mm;
}

double VerticalKinematics::getGdeg() const {
  return g_center_deg;
}

double VerticalKinematics::getGdeg_for_vertical() const {  //TODO check
  // 0° = gripper vertical; positive tilts along Arm2
  return 90 + a2_center_deg - a1_center_deg;
}

double VerticalKinematics::getGdeg_for_horizontal() const {  //TODO check
  // 0° = gripper vertical; positive tilts along Arm2
  return 180 + a2_center_deg - a1_center_deg;
}

double VerticalKinematics::getGdeg_closest_aligned() const {
  // 0° = gripper vertical; positive tilts along Arm2
  if (g_closest_horizontal) return getGdeg_for_horizontal();
  return getGdeg_for_vertical();
}

int VerticalKinematics::getGticks() const {
  return deg2ticks(ID_WRIST, getGdeg());
}
int VerticalKinematics::getGticks_closest_aligned() const {
  return deg2ticks(ID_WRIST, getGdeg_closest_aligned());
}

// -------------------------------------------------------------------
//                     GRIPPER ALIGNMENT
// -------------------------------------------------------------------
void VerticalKinematics::update_g_alignment() {
  double g_vert_deg = getGdeg_for_vertical();
  double g_horiz_deg = getGdeg_for_horizontal();
  double err_vert = fabs(g_center_deg - g_vert_deg);
  double err_horiz = fabs(g_center_deg - g_horiz_deg);
  if (err_horiz < err_vert)
    g_closest_horizontal = true;
  else
    g_closest_horizontal = false;
}

// -------------------------------------------------------------------
//                   GLOBAL INSTANCE
// -------------------------------------------------------------------
VerticalKinematics kin;
