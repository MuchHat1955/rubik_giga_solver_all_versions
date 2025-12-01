#include "vertical_kinematics.h"

// ============================================================
//               KINEMATICS SOLVERS (Final Hybrid Version)
// ============================================================

double l_mm = 60.0;
double min_ymm = 33.5;
double max_ymm = l_mm * 1.99;
double max_xmm = 30;

double _90_rad = M_PI / 2.0;
double _45_rad = M_PI / 4.0;
double _135_rad = M_PI / 2.0 + M_PI / 4.0;
double _180_rad = M_PI;
static constexpr double EPS = 1e-9;

static inline double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ============================================================
// Unified logging macros
// ============================================================
#define return_false(fmt, ...) \
  do { \
    serial_printf_verbose("ERR KIN | %s | " fmt "\n", __FUNCTION__, ##__VA_ARGS__); \
    return false; \
  } while (0)

#define return_true(fmt, ...) \
  do { \
    return true; \
  } while (0)

/*
#define return_true(fmt, ...) \
  do { \
    serial_printf_verbose("  KIN OK  | %s | " fmt "\n", __FUNCTION__, ##__VA_ARGS__); \
    return true; \
  } while (0)
*/

// Disable compact motion summary
#undef KIN_SUMMARY

#ifdef KIN_SUMMARY
#define kin_summary(fmt, ...) \
  do { serial_printf_verbose("KIN SUM | " fmt "\n", ##__VA_ARGS__); } while (0)
#else
#define kin_summary(fmt, ...)
#endif

// ============================================================
// Bounds helpers
// ============================================================
bool x_in_bounds(double _x) {
  bool _b = (_x >= -max_xmm - EPS) && (_x <= max_xmm + EPS);
  if (_b) return true;
  serial_printf_verbose("ERR x_in_bounds x=%.2f expected +/-%.2f\n", _x, max_xmm);
  return false;
}
bool y_in_bounds(double _y) {
  bool _b = (_y >= min_ymm + EPS) && (_y <= max_ymm - EPS);
  if (_b) return true;
  serial_printf_verbose("ERR y_in_bounds y=%.2f expected between %.2f to %.2f\n", _y, min_ymm, max_ymm);
  return false;
}
bool a1_servo_deg_in_bounds(double _a) {
  bool _b = (_a > -180.0 + EPS) && (_a < 95.0 - EPS);
  if (_b) return true;
  serial_printf_verbose("ERR a1_servo_deg_in_bounds a=%.2f expected between %.2f to %.2f\n", _a, -180.0, 95.0);
  return false;
}
bool a2_servo_deg_in_bounds(double _a) {
  bool _b = (_a > -180.0 + EPS) && (_a < 95.0 - EPS);
  if (_b) return true;
  serial_printf_verbose("ERR a2_servo_deg_in_bounds a=%.2f expected between %.2f to %.2f\n", _a, -180.0, 95.0);
  return false;
}

// ============================================================
// A2 and Y from A1 and X
// ============================================================
bool VerticalKinematics::solve_a2_y_from_a1_x(double _a1_servo_deg, double _x, double _g_servo_deg) {
  // serial_printf_verbose("            x=%.2f\n", _x);
  if (!x_in_bounds(_x))
    return_false("x=%.2f out of range | expected ±%.2f", _x, max_xmm);
  if (!a1_servo_deg_in_bounds(_a1_servo_deg))
    return_false("a1_servo_deg=%.2f° out of range", _a1_servo_deg);

  double _a1_servo_rad = deg2rad(_a1_servo_deg);
  double _a1_global_rad = _90_rad - _a1_servo_rad;

  double _c2 = cos(_a1_global_rad) - _x / l_mm;
  if (_c2 < -EPS || _c2 > 1.0 + EPS)
    return_false("invalid cos(a2)=%.3f (a1=%.2f°, x=%.2f)", _c2, _a1_servo_deg, _x);
  _c2 = clampd(_c2, 0.0, 1.0);

  double _a2_global_rad = acos(_c2);
  if (_a2_global_rad < 0.0 || _a2_global_rad > _90_rad + EPS)
    return_false("a2_global_rad=%.3f out of [0,%.3f]", _a2_global_rad, _90_rad);

  double _y = l_mm * (sin(_a1_global_rad) + sin(_a2_global_rad));
  double _a2_servo_rad = _a2_global_rad - _a1_servo_rad;
  double _a2_servo_deg = rad2deg(_a2_servo_rad);

  if (!y_in_bounds(_y))
    return_false("y=%.2f out of range [%.1f, %.1f]", _y, 0.01 * l_mm, 1.99 * l_mm);
  if (!a2_servo_deg_in_bounds(_a2_servo_deg))
    return_false("a2_servo_deg=%.2f° out of range", _a2_servo_deg);

  a1_servo_deg = _a1_servo_deg;
  a2_servo_deg = _a2_servo_deg;
  x_mm = _x;
  y_mm = _y;
  if (_g_servo_deg >= -95.0 && _g_servo_deg <= 185.0) g_servo_deg = _g_servo_deg;
  else g_servo_deg = ticks2deg(ID_WRIST, dxl.getPresentPosition(ID_WRIST));

  // serial_printf_verbose("        a1=%.2f x=%.2f -> x=%.2f y=%.2f\n", _a1_servo_deg, _x, x_mm, y_mm);
  kin_summary("a1=%.1f° x=%.1f → a2=%.1f° y=%.1f", _a1_servo_deg, _x, a2_servo_deg, y_mm);
  return_true("OK | a1=%.2f° x=%.2f → a2=%.2f° y=%.2f\n", _a1_servo_deg, _x, a2_servo_deg, y_mm);
}

// ============================================================
// A2 and X from A1 and Y
// ============================================================
bool VerticalKinematics::solve_a2_x_from_a1_y(double _a1_servo_deg, double _y, double _g_servo_deg) {
  if (!y_in_bounds(_y))
    return_false("y=%.2f out of range [%.1f, %.1f]", _y, 0.01 * l_mm, 1.99 * l_mm);
  if (!a1_servo_deg_in_bounds(_a1_servo_deg))
    return_false("a1_servo_deg=%.2f° out of range", _a1_servo_deg);

  double _a1_servo_rad = deg2rad(_a1_servo_deg);
  double _a1_global_rad = _90_rad - _a1_servo_rad;

  double _s2 = (_y / l_mm) - sin(_a1_global_rad);
  if (_s2 < -EPS || _s2 > 1.0 + EPS)
    return_false("invalid sin(a2)=%.3f (a1=%.2f°, y=%.2f)", _s2, _a1_servo_deg, _y);
  _s2 = clampd(_s2, 0.0, 1.0);

  double _a2_global_rad = asin(_s2);
  if (_a2_global_rad < 0.0 || _a2_global_rad > _90_rad + EPS)
    return_false("a2_global_rad=%.3f out of [0,%.3f]", _a2_global_rad, _90_rad);

  double _x = l_mm * (cos(_a1_global_rad) - cos(_a2_global_rad));
  double _a2_servo_rad = _a2_global_rad - _a1_servo_rad;
  double _a2_servo_deg = rad2deg(_a2_servo_rad);

  if (!x_in_bounds(_x))
    return_false("x=%.2f out of range [%.1f, %.1f]", _x, -0.5 * l_mm, 0.5 * l_mm);
  if (!a2_servo_deg_in_bounds(_a2_servo_deg))
    return_false("a2_servo_deg=%.2f° out of range", _a2_servo_deg);

  a1_servo_deg = _a1_servo_deg;
  a2_servo_deg = _a2_servo_deg;
  x_mm = _x;
  y_mm = _y;
  if (_g_servo_deg >= -95.0 && _g_servo_deg <= 185.0) g_servo_deg = _g_servo_deg;
  else g_servo_deg = ticks2deg(ID_WRIST, dxl.getPresentPosition(ID_WRIST));

  // serial_printf_verbose("        a1=%.2f y=%.2f -> x=%.2f y=%.2f\n", _a1_servo_deg, _y, x_mm, y_mm);
  kin_summary("a1=%.1f° y=%.1f → a2=%.1f° x=%.1f", _a1_servo_deg, _y, a2_servo_deg, x_mm);
  return_true("OK | a1=%.2f° y=%.2f → a2=%.2f° x=%.2f", _a1_servo_deg, _y, a2_servo_deg, x_mm);
}

bool VerticalKinematics::update_from_present_pos() {

  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM2) || !dxl.ping(ID_WRIST)) return false;

  double _a1_servo_deg = ticks2deg(ID_ARM1, dxl.getPresentPosition(ID_ARM1));
  double _a2_servo_deg = ticks2deg(ID_ARM2, dxl.getPresentPosition(ID_ARM2));
  double _g_servo_deg = ticks2deg(ID_WRIST, dxl.getPresentPosition(ID_WRIST));

  return solve_x_y_from_a1_a2(_a1_servo_deg, _a2_servo_deg, _g_servo_deg);
}

// ============================================================
// X and Y from A1 and A2  (Forward Kinematics)
// ============================================================
bool VerticalKinematics::solve_x_y_from_a1_a2(double _a1_servo_deg, double _a2_servo_deg, double _g_servo_deg) {
  if (!a1_servo_deg_in_bounds(_a1_servo_deg))
    return_false("a1_servo_deg=%.2f° out of range", _a1_servo_deg);
  if (!a2_servo_deg_in_bounds(_a2_servo_deg))
    return_false("a2_servo_deg=%.2f° out of range", _a2_servo_deg);

  double _a1_servo_rad = deg2rad(_a1_servo_deg);
  double _a2_servo_rad = deg2rad(_a2_servo_deg);

  double _a1_global_rad = _90_rad - _a1_servo_rad;
  double _a2_global_rad = _a1_servo_rad + _a2_servo_rad;

  if (_a1_global_rad < -_45_rad || _a1_global_rad > _135_rad + EPS)
    return_false("a1_global_rad %.3f out of [-45,+135]", rad2deg(_a1_global_rad));
  if (_a2_global_rad < -_45_rad || _a2_global_rad > _135_rad + EPS)
    return_false("a2_global_rad %.3f out of [-45,+135]", rad2deg(_a2_global_rad));

  double _x = l_mm * (cos(_a1_global_rad) - cos(_a2_global_rad));
  double _y = l_mm * (sin(_a1_global_rad) + sin(_a2_global_rad));

  if (!x_in_bounds(_x))
    return_false("x=%.2f out of range", _x);
  if (!y_in_bounds(_y))
    return_false("y=%.2f out of range", _y);

  x_mm = _x;
  y_mm = _y;
  a1_servo_deg = _a1_servo_deg;
  a2_servo_deg = _a2_servo_deg;
  if (_g_servo_deg >= -95.0 && _g_servo_deg <= 185.0) g_servo_deg = _g_servo_deg;
  else g_servo_deg = ticks2deg(ID_WRIST, dxl.getPresentPosition(ID_WRIST));
  update_g_alignment();

  // serial_printf_verbose("        kin | x=%.2f y=%.2f\n", x_mm, y_mm);
  kin_summary("a1=%.1f° a2=%.1f° | x=%.1f y=%.1f", _a1_servo_deg, _a2_servo_deg, x_mm, y_mm);
  return_true("OK | a1=%.2f° a2=%.2f° → x=%.2f y=%.2f\n", _a1_servo_deg, _a2_servo_deg, x_mm, y_mm);
}

// ============================================================
// A1 and A2 from X and Y (Inverse Kinematics)
// ============================================================
bool VerticalKinematics::solve_a1_a2_from_x_y(double _x, double _y, double _g_servo_deg) {
  if (!x_in_bounds(_x))
    return_false("x=%.2f out of range [%.1f, %.1f]", _x, -0.01 * l_mm, 0.99 * l_mm);
  if (!y_in_bounds(_y))
    return_false("y=%.2f out of range [%.1f, %.1f]", _y, 0.05 * l_mm, 1.99 * l_mm);

  double _ax = _x / l_mm;
  double _by = _y / l_mm;

  double _q = atan2(-_ax, _by);
  double _cos_q = cos(_q);
  double _sin_p = 0.0;
  if (fabs(_cos_q) > 1e-9)
    _sin_p = clampd(_by / (2.0 * _cos_q), -1.0, 1.0);
  else
    return_false("cos(q) near zero: %.6f", _cos_q);

  double _p = asin(_sin_p);

  double _a1_global_rad = _p + _q;
  double _a2_global_rad = _p - _q;

  double _a1_servo_deg = 90.0 - rad2deg(_a1_global_rad);
  double _a2_servo_deg = rad2deg(_a2_global_rad) + rad2deg(_a1_global_rad) - 90.0;

  if (!a1_servo_deg_in_bounds(_a1_servo_deg))
    return_false("a1_servo_deg=%.2f° out of range", _a1_servo_deg);
  if (!a2_servo_deg_in_bounds(_a2_servo_deg))
    return_false("a2_servo_deg=%.2f° out of range", _a2_servo_deg);

  a1_servo_deg = _a1_servo_deg;
  a2_servo_deg = _a2_servo_deg;
  x_mm = _x;
  y_mm = _y;
  if (_g_servo_deg >= -95.0 && _g_servo_deg <= 185.0) g_servo_deg = _g_servo_deg;
  else g_servo_deg = ticks2deg(ID_WRIST, dxl.getPresentPosition(ID_WRIST));
  update_g_alignment();

  // serial_printf_verbose("        kin | x=%.2f y=%.2f\n", x_mm, y_mm);
  kin_summary("x=%.1f y=%.1f | a1=%.1f° a2=%.1f°", _x, _y, a1_servo_deg, a2_servo_deg);
  return_true("OK | x=%.2f y=%.2f → a1=%.2f° a2=%.2f°", _x, _y, a1_servo_deg, a2_servo_deg);
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

double VerticalKinematics::getGdeg() const {
  return g_servo_deg;
}

double VerticalKinematics::getGdeg_for_vertical() const {  //TODO check
  // 0° = gripper vertical; positive tilts along Arm2
  return -a2_servo_deg - a1_servo_deg + 180;  //TODO changed per latest installation of hw
}

double VerticalKinematics::getGdeg_for_horizontal() const {  //TODO check // mount gripper 90 towards right vs others
  // 0° = gripper vertical; positive tilts along Arm2
  return getGdeg_for_vertical() - 90.0;
}

double VerticalKinematics::getGdeg_closest_aligned() {
  // 0° = gripper vertical; positive tilts along Arm2
  update_g_alignment();
  if (g_closest_horizontal) return getGdeg_for_horizontal();
  return getGdeg_for_vertical();
}

int VerticalKinematics::getGticks() const {
  return deg2ticks(ID_WRIST, getGdeg());
}
int VerticalKinematics::getGticks_closest_aligned() {
  return deg2ticks(ID_WRIST, getGdeg_closest_aligned());
}

// -------------------------------------------------------------------
//                     GRIPPER ALIGNMENT
// -------------------------------------------------------------------
void VerticalKinematics::update_g_alignment() {
  double g_vert_deg = getGdeg_for_vertical();
  double g_horiz_deg = getGdeg_for_horizontal();
  double err_vert = fabs(g_servo_deg - g_vert_deg);
  double err_horiz = fabs(g_servo_deg - g_horiz_deg);
  if (err_horiz < err_vert)
    g_closest_horizontal = true;
  else
    g_closest_horizontal = false;
}

// -------------------------------------------------------------------
//                   GLOBAL INSTANCE
// -------------------------------------------------------------------
VerticalKinematics kin;

void print_xy_status() {
  serial_printf_verbose("STATUS XY X=%.2fmm Y=%.2fmm A1=%.2fdeg A2=%.2fdeg G=%.2fdeg G horiz=%.2fdeg G vert=%.2fdeg G closest=%.2fdeg\n",
                        kin.getXmm(), kin.getYmm(), kin.getA1deg(), kin.getA2deg(),
                        kin.getGdeg(), kin.getGdeg_for_horizontal(), kin.getGdeg_for_vertical(), kin.getGdeg_closest_aligned());
}
