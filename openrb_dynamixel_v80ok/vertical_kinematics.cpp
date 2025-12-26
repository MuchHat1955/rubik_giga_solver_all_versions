#include "vertical_kinematics.h"

// ============================================================
//               KINEMATICS SOLVERS (Final Hybrid Version)
// ============================================================

double l_mm = 60.0;
double min_ymm = 25.0;
double max_ymm = l_mm * 1.99;
double min_xmm = -30;
double max_xmm = 30;

double min_a1 = -180;
double max_a1 = 105;
double min_a2 = -180;
double max_a2 = 105;

double _90_rad = M_PI / 2.0;
double _45_rad = M_PI / 4.0;
double _135_rad = M_PI / 2.0 + M_PI / 4.0;
double _180_rad = M_PI;
static constexpr double EPS = 0.05;

static inline double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ============================================================
// Bounds helpers
// ============================================================
bool x_in_bounds(double _x) {
  bool _b = (_x >= min_xmm - EPS) && (_x <= max_xmm + EPS);
  if (_b) return true;

  LOG_ERR(MOD_KIN, "x out of bounds", _x);
  LOG_VAR("expected from", min_xmm);
  LOG_VAR("to", max_xmm);
  return false;
}
bool y_in_bounds(double _y) {
  bool _b = (_y >= min_ymm - EPS) && (_y <= max_ymm + EPS);
  if (_b) return true;

  LOG_ERR(MOD_KIN, "y out of bounds", _y);
  LOG_VAR("expected from", min_ymm);
  LOG_VAR("to", max_ymm);
  return false;
}
bool a1_servo_deg_in_bounds(double _a) {
  bool _b = (_a > min_a1 - EPS) && (_a < max_a1 + EPS);
  if (_b) return true;

  LOG_ERR(MOD_KIN, "a1_deg_out_of_bounds", _a);
  LOG_VAR("expected from", min_a1 - EPS);
  LOG_VAR("to", max_a1 + EPS);
  return false;
}
bool a2_servo_deg_in_bounds(double _a) {
  bool _b = (_a > min_a2 - EPS) && (_a < max_a2 + EPS);
  if (_b) return true;

  LOG_ERR(MOD_KIN, "a2_deg_out_of_bounds", _a);
  LOG_VAR("expected from", min_a2 - EPS);
  LOG_VAR("to", max_a2 + EPS);
  return false;
}

// ============================================================
// A2 and Y from A1 and X
// ============================================================
bool VerticalKinematics::solve_a2_y_from_a1_x(double _a1_servo_deg, double _x, double _w_servo_deg) {
  // // serial_printf_verbose("            x=%.2f\n", _x);
  if (!x_in_bounds(_x))
    return false;
  if (!a1_servo_deg_in_bounds(_a1_servo_deg))
    return false;

  double _a1_servo_rad = deg2rad(_a1_servo_deg);
  double _a1_global_rad = _90_rad - _a1_servo_rad;

  double _c2 = cos(_a1_global_rad) - _x / l_mm;
  if (_c2 < -EPS || _c2 > 1.0 + EPS)
    return false;
  _c2 = clampd(_c2, 0.0, 1.0);

  double _a2_global_rad = acos(_c2);
  if (_a2_global_rad < 0.0 || _a2_global_rad > _90_rad + EPS)
    return false;

  double _y = l_mm * (sin(_a1_global_rad) + sin(_a2_global_rad));
  double _a2_servo_rad = _a2_global_rad - _a1_servo_rad;
  double _a2_servo_deg = rad2deg(_a2_servo_rad);

  if (!y_in_bounds(_y))
    return false;
  if (!a2_servo_deg_in_bounds(_a2_servo_deg))
    return false;

  a1_servo_deg = _a1_servo_deg;
  a2_servo_deg = _a2_servo_deg;
  x_mm = _x;
  y_mm = _y;
  if (_w_servo_deg >= -95.0 && _w_servo_deg <= 185.0) w_servo_deg = _w_servo_deg;
  else w_servo_deg = ticks2deg(ID_WRIST, dxl.getPresentPosition(ID_WRIST));

  // // serial_printf_verbose("        a1=%.2f x=%.2f -> x=%.2f y=%.2f\n", _a1_servo_deg, _x, x_mm, y_mm);
  // // kin_summary("a1=%.1f° x=%.1f → a2=%.1f° y=%.1f", _a1_servo_deg, _x, a2_servo_deg, y_mm);
  return true;
}

// ============================================================
// A2 and X from A1 and Y
// ============================================================
bool VerticalKinematics::solve_a2_x_from_a1_y(double _a1_servo_deg, double _y, double _w_servo_deg) {
  if (!y_in_bounds(_y))
    return false;
  if (!a1_servo_deg_in_bounds(_a1_servo_deg))
    return false;

  double _a1_servo_rad = deg2rad(_a1_servo_deg);
  double _a1_global_rad = _90_rad - _a1_servo_rad;

  double _s2 = (_y / l_mm) - sin(_a1_global_rad);
  if (_s2 < -EPS || _s2 > 1.0 + EPS)
    return false;
  _s2 = clampd(_s2, 0.0, 1.0);

  double _a2_global_rad = asin(_s2);
  if (_a2_global_rad < 0.0 || _a2_global_rad > _90_rad + EPS)
    return false;

  double _x = l_mm * (cos(_a1_global_rad) - cos(_a2_global_rad));
  double _a2_servo_rad = _a2_global_rad - _a1_servo_rad;
  double _a2_servo_deg = rad2deg(_a2_servo_rad);

  if (!x_in_bounds(_x))
    return false;
  if (!a2_servo_deg_in_bounds(_a2_servo_deg))
    return false;

  a1_servo_deg = _a1_servo_deg;
  a2_servo_deg = _a2_servo_deg;
  x_mm = _x;
  y_mm = _y;
  if (_w_servo_deg >= -95.0 && _w_servo_deg <= 185.0) w_servo_deg = _w_servo_deg;
  else w_servo_deg = ticks2deg(ID_WRIST, dxl.getPresentPosition(ID_WRIST));

  // // serial_printf_verbose("        a1=%.2f y=%.2f -> x=%.2f y=%.2f\n", _a1_servo_deg, _y, x_mm, y_mm);
  // kin_summary("a1=%.1f° y=%.1f → a2=%.1f° x=%.1f", _a1_servo_deg, _y, a2_servo_deg, x_mm);
  return true;
}

bool VerticalKinematics::update_from_present_pos() {

  if (!dxl_ping_cached(ID_ARM1) || !dxl_ping_cached(ID_ARM2) || !dxl_ping_cached(ID_WRIST)) return false;

  double _a1_servo_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double _a2_servo_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  double _w_servo_deg = ticks2deg(ID_WRIST, dxl.getPresentPosition(ID_WRIST));

  return solve_x_y_from_a1_a2(_a1_servo_deg, _a2_servo_deg, _w_servo_deg);
}

// ============================================================
// X and Y from A1 and A2  (Forward Kinematics)
// ============================================================
bool VerticalKinematics::solve_x_y_from_a1_a2(double _a1_servo_deg, double _a2_servo_deg, double _w_servo_deg) {
  if (!a1_servo_deg_in_bounds(_a1_servo_deg))
    return false;
  if (!a2_servo_deg_in_bounds(_a2_servo_deg))
    return false;

  double _a1_servo_rad = deg2rad(_a1_servo_deg);
  double _a2_servo_rad = deg2rad(_a2_servo_deg);

  double _a1_global_rad = _90_rad - _a1_servo_rad;
  double _a2_global_rad = _a1_servo_rad + _a2_servo_rad;

  if (_a1_global_rad < -_45_rad || _a1_global_rad > _135_rad + EPS)
    return false;
  if (_a2_global_rad < -_45_rad || _a2_global_rad > _135_rad + EPS)
    return false;

  double _x = l_mm * (cos(_a1_global_rad) - cos(_a2_global_rad));
  double _y = l_mm * (sin(_a1_global_rad) + sin(_a2_global_rad));

  if (!x_in_bounds(_x))
    return false;
  if (!y_in_bounds(_y))
    return false;

  x_mm = _x;
  y_mm = _y;
  a1_servo_deg = _a1_servo_deg;
  a2_servo_deg = _a2_servo_deg;
  if (_w_servo_deg >= -95.0 && _w_servo_deg <= 185.0) w_servo_deg = _w_servo_deg;
  else w_servo_deg = ticks2deg(ID_WRIST, dxl.getPresentPosition(ID_WRIST));

  // // serial_printf_verbose("        kin | x=%.2f y=%.2f\n", x_mm, y_mm);
  // kin_summary("a1=%.1f° a2=%.1f° | x=%.1f y=%.1f", _a1_servo_deg, _a2_servo_deg, x_mm, y_mm);
  return true;
}

// ============================================================
// A1 and A2 from X and Y (Inverse Kinematics)
// ============================================================
bool VerticalKinematics::solve_a1_a2_from_x_y(double _x, double _y, double _w_servo_deg) {
  if (!x_in_bounds(_x))
    return false;
  if (!y_in_bounds(_y))
    return false;

  double _ax = _x / l_mm;
  double _by = _y / l_mm;

  double _q = atan2(-_ax, _by);
  double _cos_q = cos(_q);
  double _sin_p = 0.0;
  if (fabs(_cos_q) > 1e-9)
    _sin_p = clampd(_by / (2.0 * _cos_q), -1.0, 1.0);
  else
    return false;

  double _p = asin(_sin_p);

  double _a1_global_rad = _p + _q;
  double _a2_global_rad = _p - _q;

  double _a1_servo_deg = 90.0 - rad2deg(_a1_global_rad);
  double _a2_servo_deg = rad2deg(_a2_global_rad) + rad2deg(_a1_global_rad) - 90.0;

  if (!a1_servo_deg_in_bounds(_a1_servo_deg))
    return false;
  if (!a2_servo_deg_in_bounds(_a2_servo_deg))
    return false;

  a1_servo_deg = _a1_servo_deg;
  a2_servo_deg = _a2_servo_deg;
  x_mm = _x;
  y_mm = _y;
  if (_w_servo_deg >= -95.0 && _w_servo_deg <= 185.0) w_servo_deg = _w_servo_deg;
  else w_servo_deg = ticks2deg(ID_WRIST, dxl.getPresentPosition(ID_WRIST));

  // // serial_printf_verbose("        kin | x=%.2f y=%.2f\n", x_mm, y_mm);
  // kin_summary("x=%.1f y=%.1f | a1=%.1f° a2=%.1f°", _x, _y, a1_servo_deg, a2_servo_deg);
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
  return a1_servo_deg;
}
double VerticalKinematics::getA2deg() const {
  return a2_servo_deg;
}

int VerticalKinematics::getA1ticks() const {
  return deg2ticks(ID_ARM1, a1_servo_deg);
}
int VerticalKinematics::getA2ticks() const {
  return deg2ticks(ID_ARM2, a2_servo_deg);
}

double VerticalKinematics::getXmm() const {
  return x_mm;
}
double VerticalKinematics::getYmm() const {
  return y_mm;
}

double VerticalKinematics::getWdeg() const {
  return w_servo_deg;
}

/*
#define W_HORIZ_RIGHT -90 + VERT_CORRECTION // -85
#define W_VERT 0 + VERT_CORRECTION // 5
#define W_HORIZ_LEFT 90 + VERT_CORRECTION //95
*/

double VerticalKinematics::getWdeg_for_vertical() const {
  // 0° = gripper vertical; positive tilts along Arm2
  return -a2_servo_deg - a1_servo_deg + W_VERT + 180;  // was + 180
}

double VerticalKinematics::getWdeg_for_horizontal_right() const {
  // 0° = gripper vertical; positive tilts along Arm2
  return getWdeg_for_vertical() + W_HORIZ_RIGHT;  // was -90
}

double VerticalKinematics::getWdeg_for_horizontal_left() const {
  // 0° = gripper vertical; positive tilts along Arm2
  return getWdeg_for_vertical() + W_HORIZ_LEFT;  // was + 90
}

int VerticalKinematics::getGticks() const {
  return deg2ticks(ID_WRIST, getWdeg());
}

// -------------------------------------------------------------------
//                   GLOBAL INSTANCE
// -------------------------------------------------------------------
VerticalKinematics kin;