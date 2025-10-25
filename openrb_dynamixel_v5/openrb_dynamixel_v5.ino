#include <Dynamixel2Arduino.h>
#include <math.h>

// ---------------- Test stats ----------------
struct TestStats {
  long cntSmall = 0;
  long cntLarge = 0;
  long sumAbsErrSmall = 0;
  long sumAbsErrLarge = 0;
};

// ---------------- Serial printf helper ----------------
template<typename... Args>
void serial_printf(const char *fmt, Args... args) {
  char buf[200];
  snprintf(buf, sizeof(buf), fmt, args...);
  Serial.print(buf);
}

#define DXL_SERIAL Serial1
#define DXL_DIR_PIN -1
#define PROTOCOL 2.0
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ---------------- Servo IDs ----------------
#define ID_ARM1 11
#define ID_ARM2 12
#define ID_WRIST 13
#define ID_GRIP1 14
#define ID_GRIP2 15
#define ID_BASE 16
uint8_t servo_ids[] = { ID_ARM1, ID_ARM2, ID_WRIST, ID_GRIP1, ID_GRIP2, ID_BASE };
const uint8_t SERVO_COUNT = sizeof(servo_ids) / sizeof(servo_ids[0]);

// ---------------- Constants ----------------
const int STALL_CURRENT_mA = 1000;
const int TEMP_LIMIT_C = 70;
const uint8_t CHECK_INTERVAL_MS = 20;

// ---------------- TUNING (easy to adjust) ----------------
// STEP vel% mapping (demo-style STEP command)
const int PV_MIN = 40;   // Profile Velocity @ 1%
const int PV_MAX = 600;  // Profile Velocity @ 100%
const int PA_MIN = 40;   // Profile Acceleration mirrors velocity
const int PA_MAX = 600;

// Smart-move thresholds & profiles
const int SMALL_MOVE_TICKS = 10;       // |rel| <= => "small move"
const int FINE_WINDOW_TICKS = 40;      // enter fine profile when |err| <= this
const int SMALL_OVER_UP_TICKS = 6;     // overshoot when moving "up/against load"
const int SMALL_OVER_DOWN_TICKS = 2;   // overshoot when moving "down/with load"
const int OVER_DELAY_MS_PER_TICK = 8;  // overshoot dwell per tick

// Coarse (long) move profile/gains
const int COARSE_PV = 300;
const int COARSE_PA = 200;
const int COARSE_PG = 900;  // Position P gain
const int COARSE_DG = 100;  // Position D gain (damping)

/*
// Fine (finish) profile/gains
const int FINE_PV = 80;
const int FINE_PA = 80;
const int FINE_PG = 1100;  // stronger bite near end
const int FINE_DG = 10;
const int FINE_IG = 200;  // small I to chew through stiction
*/

// Fine (finish) profile/gains
const int FINE_PV = 80;
const int FINE_PA = 80;
const int FINE_PG = 1200;  // sharper final response
const int FINE_DG = 10;
const int FINE_IG = 300;  // stronger integral correction

// Finisher / micro-jog
const int FINISH_TOL_TICKS = 1;       // acceptable final error
const int MICROJOG_MAX_MS = 600;      // how long to micro-jog
const int MICROJOG_SAMPLE_DELAY = 8;  // ms between jog decisions

// Timeouts (upper caps)
const uint32_t SMALL_MOVE_TIMEOUT_MS = 600;
const uint32_t LARGE_MOVE_TIMEOUT_MS = 3000;

// ---- Motion timing model (for dynamic wait) ----
const float TICKS_PER_REV = 4096.0f;
const float DEG_PER_TICK = 360.0f / TICKS_PER_REV;
const float PV_UNIT_RPM = 0.229f;        // 1 LSB of PROFILE_VELOCITY ≈ 0.229 rpm
const float TIME_SAFETY_FACTOR = 1.20f;  // multiply estimate by this
const uint32_t MIN_WAIT_MS = 120;        // min budget for wait loop
const uint32_t EXTRA_SETTLE_MS = 50;     // added after estimate

// ---------------- Batch test options ----------------
const int TEST_DEFAULT_COUNT = 40;  // default #cycles/moves
const int TEST_PAUSE_MS = 20;
const int STOW_TICK = 2000;  // "home/stow" absolute tick

// For randomized tests
const int RAND_INCS[] = { 5, 19, 30, 50 };
const int RAND_INCS_N = sizeof(RAND_INCS) / sizeof(RAND_INCS[0]);
const int SMALL_THRESH_ABS = 20;  // <20 small, >20 large

// ---------------- Calibration ----------------
float arm_length_mm = 0;  // 0 = unset
int tick_zero_arm1 = -1;  // -1 = unset
int tick_zero_arm2 = -1;  // -1 = unset
bool has_arm_len = false;
bool has_zero_arm1 = false;
bool has_zero_arm2 = false;

// ---------------- Forward declarations ----------------
void cmdMoveAdaptive(uint8_t id, int goal, uint8_t depth);

// ---------------- LED helpers (XL-430 single red LED; on/off) ----------------
void lOn(uint8_t id) {
  if (dxl.ping(id)) dxl.ledOn(id);
}
void lOff(uint8_t id) {
  if (dxl.ping(id)) dxl.ledOff(id);
}
void lBlink(uint8_t id, uint16_t period_ms = 400) {
  static unsigned long last_toggle[256] = { 0 };
  static bool state[256] = { false };
  unsigned long now = millis();
  if (now - last_toggle[id] >= period_ms) {
    last_toggle[id] = now;
    state[id] = !state[id];
    if (state[id]) dxl.ledOn(id);
    else dxl.ledOff(id);
  }
}

// ---------------- Status helpers ----------------
bool isInPosition(uint8_t id) {
  int s = dxl.readControlTableItem(ControlTableItem::MOVING_STATUS, id);
  return (s >= 0) && (s & 0x01);  // bit0
}
bool isMoving(uint8_t id) {
  int s = dxl.readControlTableItem(ControlTableItem::MOVING_STATUS, id);
  return (s >= 0) && (s & 0x02);  // bit1
}

// ---------------- Stall detection ----------------
bool checkStall(uint8_t id) {
  int curr = dxl.getPresentCurrent(id);
  int temp = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, id);
  if (temp >= TEMP_LIMIT_C || curr > STALL_CURRENT_mA) {
    dxl.torqueOff(id);
    lBlink(id, 200);
    serial_printf("STALL ID %d curr=%d temp=%d\n", id, curr, temp);
    serial_printf("MOVE END id=%d goal=-- final=%d err=--\n", id, dxl.getPresentPosition(id));
    return true;
  }
  return false;
}

// ---------------- Test stats ----------------
namespace tst {
static inline void addErr(TestStats &s, int plannedDist, int absErr) {
  if (plannedDist < SMALL_THRESH_ABS) {
    s.cntSmall++;
    s.sumAbsErrSmall += absErr;
  } else if (plannedDist > SMALL_THRESH_ABS) {
    s.cntLarge++;
    s.sumAbsErrLarge += absErr;
  }
}
static inline void printStats(const char *label, uint8_t id, const TestStats &s) {
  long avgSmall = (s.cntSmall ? (s.sumAbsErrSmall + s.cntSmall / 2) / s.cntSmall : 0);
  long avgLarge = (s.cntLarge ? (s.sumAbsErrLarge + s.cntLarge / 2) / s.cntLarge : 0);
  serial_printf("STATS %s id=%d small_cnt=%ld avg_abs_err_small=%ld  large_cnt=%ld avg_abs_err_large=%ld\n",
                label, id, s.cntSmall, avgSmall, s.cntLarge, avgLarge);
}
}

// ---------------- Profile & timing helpers ----------------
static inline float pvToTicksPerSec(int pvLSB) {
  float rpm = (pvLSB <= 0 ? 0.0f : pvLSB * PV_UNIT_RPM);
  float rps = rpm / 60.0f;
  float tps = rps * TICKS_PER_REV;
  if (tps < 1.0f) tps = 1.0f;
  return tps;
}
static inline uint32_t estimateTravelTimeMs(uint8_t id, int deltaTicks) {
  int pv = dxl.readControlTableItem(ControlTableItem::PROFILE_VELOCITY, id);
  float tps = pvToTicksPerSec(pv);
  float base = (float)MIN_WAIT_MS;
  float moveMs = (fabs((float)deltaTicks) / tps) * 1000.0f * TIME_SAFETY_FACTOR;
  float total = base + moveMs + (float)EXTRA_SETTLE_MS;
  if (total < (float)MIN_WAIT_MS) total = (float)MIN_WAIT_MS;
  return (uint32_t)total;
}

// ---------------- Smart move (dynamic wait, fine approach, micro-jog) -----------
void moveServoSmart(uint8_t id, int rel_ticks) {
  if (!dxl.ping(id) || rel_ticks == 0) return;

  const int start = dxl.getPresentPosition(id);
  const int goal = start + rel_ticks;

  const int absMove = abs(rel_ticks);
  const bool smallMove = (absMove <= SMALL_MOVE_TICKS);

  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id, 885);
  dxl.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, id, 1);
  dxl.writeControlTableItem(ControlTableItem::DRIVE_MODE, id, 0);  // velocity-profile
  if (!dxl.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id)) dxl.torqueOn(id);

  lOn(id);

  if (abs(goal - start) > FINE_WINDOW_TICKS) {
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, COARSE_PV);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, COARSE_PA);
    dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, COARSE_PG);
    dxl.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, id, COARSE_DG);
  } else {
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, FINE_PV);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, FINE_PA);
    dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, FINE_PG);
    dxl.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, id, FINE_DG);
    dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, id, FINE_IG);
  }

  if (smallMove) {
    const int over = (rel_ticks > 0 ? SMALL_OVER_UP_TICKS : -SMALL_OVER_DOWN_TICKS);
    dxl.setGoalPosition(id, goal + over);
    delay(abs(over) * OVER_DELAY_MS_PER_TICK);
  }

  dxl.setGoalPosition(id, goal);

  uint32_t dynBudget = estimateTravelTimeMs(id, goal - start);
  uint32_t cap = smallMove ? SMALL_MOVE_TIMEOUT_MS : LARGE_MOVE_TIMEOUT_MS;
  uint32_t maxTime = (dynBudget > cap) ? cap : dynBudget;

  uint32_t t0 = millis();
  bool fine_applied = (abs(goal - start) <= FINE_WINDOW_TICKS);

  while (millis() - t0 < maxTime) {
    if (checkStall(id)) return;

    int pos_now = dxl.getPresentPosition(id);
    int err_now = goal - pos_now;

    if (!fine_applied && abs(err_now) <= FINE_WINDOW_TICKS) {
      dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, FINE_PV);
      dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, FINE_PA);
      dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, FINE_PG);
      dxl.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, id, FINE_DG);
      dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, id, FINE_IG);
      dxl.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, id, 1);
      fine_applied = true;
    }

    if (!isMoving(id) && isInPosition(id)) break;
    delay(10);
  }

  int pos = dxl.getPresentPosition(id);
  int diff = goal - pos;
  uint32_t t_trim = millis();

  while (millis() - t_trim < MICROJOG_MAX_MS) {
    if (abs(diff) <= FINISH_TOL_TICKS) break;
    int nudge = (diff > 0 ? 1 : -1);
    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, pos + nudge);
    delay(MICROJOG_SAMPLE_DELAY);
    pos = dxl.getPresentPosition(id);
    diff = goal - pos;
  }

  delay(20);
  pos = dxl.getPresentPosition(id);
  int err = pos - goal;

  lOff(id);
  serial_printf("MOVE END id=%d start=%d goal=%d final=%d err=%d\n", id, start, goal, pos, err);
}

// ---------------- MOVEY / MOVEX ----------------

// ----------------------------------------------------
// Adaptive cmdMoveY: keep X constant by compensating with Arm2
// ----------------------------------------------------
void cmdMoveY(int rel_y_ticks) {
  if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2) {
    Serial.println("ERR: arm not calibrated (SETLEN/SETZERO)");
    return;
  }

  // read current positions
  int pos1 = dxl.getPresentPosition(ID_ARM1);
  int pos2 = dxl.getPresentPosition(ID_ARM2);

  // planned new targets (symmetrical)
  int goal1 = pos1 + rel_y_ticks;
  int goal2 = pos2 - rel_y_ticks;

  // perform coordinated motion in small slices to maintain X
  const int steps = 10;  // interpolate
  const int slice1 = rel_y_ticks / steps;
  const int slice2 = -rel_y_ticks / steps;

  for (int i = 1; i <= steps; i++) {
    int subGoal1 = pos1 + slice1 * i;
    int subGoal2 = pos2 + slice2 * i;

    // issue both moves
    moveServoSmart(ID_ARM1, subGoal1 - dxl.getPresentPosition(ID_ARM1));
    moveServoSmart(ID_ARM2, subGoal2 - dxl.getPresentPosition(ID_ARM2));

    // read back after each slice
    int cur1 = dxl.getPresentPosition(ID_ARM1);
    int cur2 = dxl.getPresentPosition(ID_ARM2);

    // compute X deviation from symmetry
    int errX = (cur1 + cur2) - (pos1 + pos2);  // if >0, midpoint drifted right
    if (abs(errX) > 2) {
      // small correction: let Arm2 compensate
      int comp = (errX / 2);
      dxl.setGoalPosition(ID_ARM2, cur2 - comp);
      delay(15);
    }
  }
  // final alignment pass (fine adaptive)
  cmdMoveAdaptive(ID_ARM1, goal1, 0);
  cmdMoveAdaptive(ID_ARM2, goal2, 0);
}

// ----------------------------------------------------
// Geometry constants for X/Y computation
// ----------------------------------------------------
const float ARM_LENGTH_MM = 48.0f;
const float TICK_TO_RAD = (2.0f * M_PI) / 4096.0f;  // 0.00153398 rad/tick
const int TICK_ZERO = 2000;                         // zero-angle tick

// compute X,Y midpoint (mm) from actual arm ticks
void computeXY(int ticks1, int ticks2, float &x_mm, float &y_mm) {
  float theta1 = (ticks1 - TICK_ZERO) * TICK_TO_RAD;
  float theta2 = (ticks2 - TICK_ZERO) * TICK_TO_RAD;
  float x1 = -ARM_LENGTH_MM * cos(theta1);
  float y1 = ARM_LENGTH_MM * sin(theta1);
  float x2 = ARM_LENGTH_MM * cos(theta2);
  float y2 = ARM_LENGTH_MM * sin(theta2);
  x_mm = (x2 - x1) * 0.5f;
  y_mm = (y1 + y2) * 0.5f;
}

// ----------------------------------------------------
// cmdTestMoveY(start_mm, end_mm, steps)
// ----------------------------------------------------
void cmdTestMoveY(float yStart_mm, float yEnd_mm, int steps = 10) {
  Serial.println("=== cmdMoveY kinematic test ===");
  serial_printf("Arm length=%.1fmm  zeroTick=%d\n", ARM_LENGTH_MM, TICK_ZERO);

  // Convert Y target to servo angles and ticks
  // Using simple arcsin(y/L)
  float thetaStart = asin(constrain(yStart_mm / ARM_LENGTH_MM, -1.0f, 1.0f));
  float thetaEnd = asin(constrain(yEnd_mm / ARM_LENGTH_MM, -1.0f, 1.0f));
  int ticksStart = TICK_ZERO + (int)(thetaStart / TICK_TO_RAD);
  int ticksEnd = TICK_ZERO + (int)(thetaEnd / TICK_TO_RAD);
  int deltaTicks = ticksEnd - ticksStart;

  serial_printf("Ystart=%.2fmm (ticks=%d)  Yend=%.2fmm (ticks=%d)  Δticks=%d\n",
                yStart_mm, ticksStart, yEnd_mm, ticksEnd, deltaTicks);

  // --- bring both arms to starting position ---
  cmdMoveAdaptive(ID_ARM1, ticksStart, 0);
  cmdMoveAdaptive(ID_ARM2, ticksStart, 0);

  delay(300);
  int pos1 = dxl.getPresentPosition(ID_ARM1);
  int pos2 = dxl.getPresentPosition(ID_ARM2);
  float x0, y0;
  computeXY(pos1, pos2, x0, y0);
  serial_printf("Start measured: A1=%d A2=%d  X=%.2f Y=%.2f\n", pos1, pos2, x0, y0);

  // --- execute vertical move in slices ---
  int stepTicks = deltaTicks / steps;
  for (int i = 1; i <= steps; i++) {
    int goal1 = ticksStart + stepTicks * i;
    int goal2 = ticksStart - stepTicks * i;  // mirror
    cmdMoveAdaptive(ID_ARM1, goal1, 0);
    cmdMoveAdaptive(ID_ARM2, goal2, 0);

    int a1 = dxl.getPresentPosition(ID_ARM1);
    int a2 = dxl.getPresentPosition(ID_ARM2);
    float x_mm, y_mm;
    computeXY(a1, a2, x_mm, y_mm);
    float x_err = x_mm - x0;
    serial_printf("Step %2d | A1=%4d A2=%4d | X=%.3fmm  Y=%.3fmm  Xerr=%.3fmm\n",
                  i, a1, a2, x_mm, y_mm, x_err);
  }

  // --- summary ---
  int f1 = dxl.getPresentPosition(ID_ARM1);
  int f2 = dxl.getPresentPosition(ID_ARM2);
  float xF, yF;
  computeXY(f1, f2, xF, yF);
  serial_printf("End measured: A1=%d A2=%d  X=%.3f Y=%.3f  ΔY=%.3f  X drift=%.3f\n",
                f1, f2, xF, yF, yF - y0, xF - x0);
  Serial.println("=== end test ===\n");
}

// ----------------------------------------------------
// cmdTestMoveX(start_mm, end_mm, steps)
// ----------------------------------------------------
// ----------------------------------------------------
// cmdTestMoveX(start_mm, end_mm, steps)
// Parallel sweep of both arms -> lateral motion test
// ----------------------------------------------------
void cmdTestMoveX(float xStart_mm, float xEnd_mm, int steps = 10) {
  Serial.println("=== cmdMoveX kinematic test ===");
  serial_printf("Arm length=%.1fmm  zeroTick=%d\n", ARM_LENGTH_MM, TICK_ZERO);

  // Compute ticks equivalent to small angular delta around zero for given X target
  // For near-symmetric arms, lateral X ≈ L*(cosθ1 - cosθ2)/2
  // We'll approximate θ for small X using linear mapping.
  float span_mm = xEnd_mm - xStart_mm;
  float midX_mm = (xStart_mm + xEnd_mm) * 0.5f;

  // Start: both arms same side offset from zero
  int ticksStart = TICK_ZERO + (int)((xStart_mm / ARM_LENGTH_MM) / TICK_TO_RAD / 2.0f);
  int ticksEnd = TICK_ZERO + (int)((xEnd_mm / ARM_LENGTH_MM) / TICK_TO_RAD / 2.0f);
  int deltaTicks = ticksEnd - ticksStart;

  serial_printf("Xstart=%.2fmm (ticks=%d)  Xend=%.2fmm (ticks=%d)  Δticks=%d\n",
                xStart_mm, ticksStart, xEnd_mm, ticksEnd, deltaTicks);

  // --- bring both arms to starting parallel position ---
  cmdMoveAdaptive(ID_ARM1, ticksStart, 0);
  cmdMoveAdaptive(ID_ARM2, ticksStart, 0);

  delay(300);
  int pos1 = dxl.getPresentPosition(ID_ARM1);
  int pos2 = dxl.getPresentPosition(ID_ARM2);
  float x0, y0;
  computeXY(pos1, pos2, x0, y0);
  serial_printf("Start measured: A1=%d A2=%d  X=%.2f Y=%.2f\n", pos1, pos2, x0, y0);

  // --- execute lateral move in slices ---
  int stepTicks = deltaTicks / steps;
  for (int i = 1; i <= steps; i++) {
    int goal1 = ticksStart + stepTicks * i;
    int goal2 = ticksStart + stepTicks * i;  // same direction for lateral
    cmdMoveAdaptive(ID_ARM1, goal1, 0);
    cmdMoveAdaptive(ID_ARM2, goal2, 0);

    int a1 = dxl.getPresentPosition(ID_ARM1);
    int a2 = dxl.getPresentPosition(ID_ARM2);
    float x_mm, y_mm;
    computeXY(a1, a2, x_mm, y_mm);
    float y_err = y_mm - y0;
    serial_printf("Step %2d | A1=%4d A2=%4d | X=%.3fmm  Y=%.3fmm  Yerr=%.3fmm\n",
                  i, a1, a2, x_mm, y_mm, y_err);
  }

  // --- summary ---
  int f1 = dxl.getPresentPosition(ID_ARM1);
  int f2 = dxl.getPresentPosition(ID_ARM2);
  float xF, yF;
  computeXY(f1, f2, xF, yF);
  serial_printf("End measured: A1=%d A2=%d  X=%.3f Y=%.3f  ΔX=%.3f  Y drift=%.3f\n",
                f1, f2, xF, yF, xF - x0, yF - y0);
  Serial.println("=== end test ===\n");
}

void cmdMoveX(int rel_ticks) {
  if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2) {
    Serial.println("ERR: arm not calibrated (SETLEN/SETZERO)");
    return;
  }
  moveServoSmart(ID_ARM1, +rel_ticks);
  moveServoSmart(ID_ARM2, +rel_ticks);
}

// ---------------- ADAPTIVE finisher (with safe auto-retry) ----------------
void cmdMoveAdaptive(uint8_t id, int goal, uint8_t depth) {
  if (!dxl.ping(id)) {
    serial_printf("Servo %d not found\n", id);
    return;
  }

  // --- recursion safety guard ---
  if (depth > 2) {  // allow at most 1 re-try (depth 0 → 1)
    serial_printf("  aborting extra retries (depth=%d)\n", depth);
    return;
  }

  unsigned long startMillis = millis();
  dxl.torqueOn(id);
  dxl.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, id, 1);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id, 1023);
  lOn(id);

  int start = dxl.getPresentPosition(id);
  dxl.setGoalPosition(id, goal);

  uint32_t dynBudget = estimateTravelTimeMs(id, goal - start);
  uint32_t cap = (abs(goal - start) <= SMALL_MOVE_TICKS)
                   ? SMALL_MOVE_TIMEOUT_MS
                   : LARGE_MOVE_TIMEOUT_MS;
  uint32_t maxTime = (dynBudget > cap) ? cap : dynBudget;

  // proportional settle budget
  uint32_t base = 120;
  uint32_t perTick = 1;
  uint32_t settleBudget = base + (abs(goal - start) * perTick);
  if (settleBudget > maxTime) settleBudget = maxTime;

  uint32_t t0 = millis();
  while (isMoving(id) && (millis() - t0 < maxTime)) delay(10);

  dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, 1150);
  dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, id, 250);

  const int TOL = 1;
  const uint32_t settleLimit = 900;
  uint32_t t1 = millis();
  int stableCount = 0;

  while (millis() - t1 < settleBudget && millis() - t1 < settleLimit) {
    int pos = dxl.getPresentPosition(id);
    int curr = dxl.getPresentCurrent(id);
    int err = goal - pos;

    if (!isMoving(id) && abs(curr) < 30 && abs(err) <= TOL) {
      stableCount++;
      if (stableCount > 3) break;
    } else stableCount = 0;

    if (abs(err) <= TOL) break;

    if (abs(curr) < 50 && abs(err) > 1) {
      int nudge = (err > 0 ? 1 : -1);
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, goal + nudge);
      delay(8);
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, goal);
    }

    if (abs(curr) > 400 && ((curr > 0 && err < 0) || (curr < 0 && err > 0))) {
      int nudge = (err > 0 ? -1 : 1);
      dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, goal + nudge);
    }

    delay(MICROJOG_SAMPLE_DELAY);
  }

  int pos = dxl.getPresentPosition(id);
  int diff = goal - pos;

  // --- final micro-trim ---
  if (abs(diff) > 5) {
    int trim = (diff > 0 ? 1 : -1);
    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, goal + trim);
    delay(15);
    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, goal);
  }

  delay(20);
  pos = dxl.getPresentPosition(id);
  lOff(id);
  serial_printf("MOVE END id=%d start=%d goal=%d final=%d err=%d (ADAPTIVE d=%d)\n",
                id, start, goal, pos, (pos - goal), depth);

  // --- single safe re-run if residual error still >6 ticks ---
  diff = goal - pos;
  if (abs(diff) > 6 && depth < 2) {
    serial_printf("  retry adaptive fine-trim (err=%d)\n", diff);
    int subGoal = goal + (diff > 0 ? 10 : -10);
    if (subGoal < 0) subGoal = 0;
    if (subGoal > 4095) subGoal = 4095;
    cmdMoveAdaptive(id, subGoal, depth + 1);
    cmdMoveAdaptive(id, goal, depth + 1);
  }
}

// ---------------- READ diagnostics ----------------
void print_status() {
  Serial.println("ID | Position | Current(mA) | Temp(C)");
  Serial.println("--------------------------------------");
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    uint8_t id = servo_ids[i];
    if (!dxl.ping(id)) {
      serial_printf("%2d |   ---     |    ---     |  ---\n", id);
      continue;
    }
    int pos = dxl.getPresentPosition(id);
    int curr = dxl.getPresentCurrent(id);
    int temp = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, id);
    serial_printf("%2d | %5d     | %6d     | %3d\n", id, pos, curr, temp);
  }
  Serial.println("--------------------------------------");
}

// ---------------- INFO <id> ----------------
static const char *modeName(int m) {
  switch (m) {
    case 0: return "Current(0)";
    case 1: return "Velocity(1)";
    case 3: return "Position(3)";
    case 4: return "ExtPosition(4)";
    case 5: return "Current+Position(5)";
    case 16: return "PWM(16)";
    default: return "Unknown";
  }
}
void cmdInfo(uint8_t id) {
  if (!dxl.ping(id)) {
    serial_printf("Servo %d not found\n", id);
    return;
  }
  int op = dxl.readControlTableItem(ControlTableItem::OPERATING_MODE, id);
  int drv = dxl.readControlTableItem(ControlTableItem::DRIVE_MODE, id);
  int pv = dxl.readControlTableItem(ControlTableItem::PROFILE_VELOCITY, id);
  int pa = dxl.readControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id);
  int minL = dxl.readControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id);
  int maxL = dxl.readControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id);
  int pos = dxl.getPresentPosition(id);

  float rpm = pv * PV_UNIT_RPM;
  float tps = pvToTicksPerSec(pv);
  float spanTicks = (maxL > 0 && minL >= 0 && maxL > minL) ? (float)(maxL - minL) : TICKS_PER_REV;
  float spanDeg = spanTicks * DEG_PER_TICK;

  serial_printf("INFO id=%d\n", id);
  serial_printf("  OperatingMode : %s\n", modeName(op));
  serial_printf("  DriveMode     : %d (bit0=%s-profile)\n", drv, (drv & 0x01) ? "TIME" : "VELOCITY");
  serial_printf("  Profile Vel   : %d LSB ≈ %.3f rpm (≈ %.1f ticks/s)\n", pv, rpm, tps);
  serial_printf("  Profile Accel : %d LSB\n", pa);
  serial_printf("  PosLimits     : min=%d  max=%d  (span≈ %.1f°)\n", minL, maxL, spanDeg);
  serial_printf("  Present Pos   : %d ticks (≈ %.2f°)\n", pos, pos * DEG_PER_TICK);
  serial_printf("  Resolution    : %.6f deg/tick; wrap at 0/4095 in Position mode\n", DEG_PER_TICK);
}

// ---------------- Test/stat helpers & batch tests ----------------
void recordAdaptiveAbs(uint8_t id, int goal, TestStats &s) {
  int start = dxl.getPresentPosition(id);
  int planned = abs(goal - start);
  cmdMoveAdaptive(id, goal, 0);
  delay(TEST_PAUSE_MS);
  int final = dxl.getPresentPosition(id);
  int absErr = abs(final - goal);
  tst::addErr(s, planned, absErr);
}
static inline int randSign() {
  return (random(0, 2) ? 1 : -1);
}

// TESTMOVE <id> [count]  (count = number of cycles; each cycle ~18 moves)
void cmdTestAdaptive(uint8_t id, int countTests) {
  if (!dxl.ping(id)) {
    serial_printf("Servo %d not found\n", id);
    return;
  }
  if (countTests <= 0) countTests = TEST_DEFAULT_COUNT;

  TestStats stats;
  serial_printf("TESTMOVE start id=%d tests=%d\n", id, countTests);

  int countCycles = (countTests + 11) / 20;  // 20 tests per cycle
  if (countCycles <= 0) countCycles = 1;

  for (int iter = 0; iter < countCycles; ++iter) {

    // 1) STOW twice 2
    recordAdaptiveAbs(id, STOW_TICK, stats);
    recordAdaptiveAbs(id, STOW_TICK, stats);

    // 2) Small increments up & back (absolute): 4 sizes * (up+back) = 8 -> 10
    int baseline = dxl.getPresentPosition(id);
    for (int i = 0; i < RAND_INCS_N; ++i) {
      recordAdaptiveAbs(id, baseline + RAND_INCS[i], stats);
      recordAdaptiveAbs(id, baseline, stats);
    }

    // 3) Large absolute excursion and back: 2 -> 12
    recordAdaptiveAbs(id, STOW_TICK + 1000, stats);
    recordAdaptiveAbs(id, STOW_TICK, stats);

    // 4) Very large absolute excursion and back: 2 -> 14
    recordAdaptiveAbs(id, STOW_TICK + 2100, stats);
    recordAdaptiveAbs(id, STOW_TICK, stats);

    // 5) Same-direction +5 five times, then one return: 6  -> 20
    baseline = dxl.getPresentPosition(id);
    for (int k = 0; k < 5; ++k) { recordAdaptiveAbs(id, baseline + 5 * (k + 1), stats); }
    recordAdaptiveAbs(id, baseline, stats);
  }

  tst::printStats("TESTMOVE", id, stats);
  serial_printf("TESTMOVE end id=%d\n", id);
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  dxl.begin(57600);
  dxl.setPortProtocolVersion(PROTOCOL);

  // allow randomized tests to vary per run
  randomSeed(analogRead(A0));

  Serial.println("=== OpenRB Arm Motion Controller (tuned + info + dyn wait + tests) ===");
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    uint8_t id = servo_ids[i];
    if (dxl.ping(id)) {
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_POSITION);
      dxl.torqueOn(id);
      lOff(id);
      serial_printf("Servo %d OK\n", id);
    } else {
      serial_printf("Servo %d not found!\n", id);
    }
  }

  Serial.println("\nSupported Commands:");

  Serial.println("  MOVE <id> <absgoal>           - adaptive move one servo");
  Serial.println("  TESTMOVE <id> [cycles]        - test adaptive move");
  Serial.println("");
  Serial.println("  SETLEN  <mm>                  - set arm length");
  Serial.println("  SETZERO  <id> <ticks>         - set zero tick");
  Serial.println("  MOVEY <ticks>                 - vertical (mirror arm1/arm2)");
  Serial.println("  TESTMOVEY <ticks>             - test move y");
  Serial.println("  MOVEX <ticks>                 - lateral (same dir)");
  Serial.println("  TESTMOVEX <ticks>             - test move x");
  Serial.println("");
  Serial.println("  INFO  <id>                    - xl430 firmware mode/limits/profiles/rates");
  Serial.println("  READ                          - all servos position, current, temp\n");
  Serial.println("  LEDON <id> / LEDOFF <id>");
  Serial.println("");
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  if (line.length() == 0) return;
  line.trim();
  String U = line;
  U.toUpperCase();

  if (U.startsWith("MOVEY")) {
    int v = line.substring(6).toInt();
    if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2) Serial.println("ERR: arm not calibrated (SETLEN/SETZERO)");
    else cmdMoveY(v);
  } else if (U.startsWith("MOVEX")) {
    int v = line.substring(6).toInt();
    if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2) Serial.println("ERR: arm not calibrated (SETLEN/SETZERO)");
    else cmdMoveX(v);

  } else if (U.startsWith("TESTMOVEY")) {
    float y1 = 0, y2 = 0;
    if (sscanf(line.c_str(), "TESTMOVEY %f %f", &y1, &y2) == 2) cmdTestMoveY(y1, y2);
    else Serial.println("Usage: TESTMOVEY <yStart_mm> <yEnd_mm>");

  } else if (U.startsWith("TESTMOVEX")) {
    float y1 = 0, y2 = 0;
    if (sscanf(line.c_str(), "TESTMOVEX %f %f", &y1, &y2) == 2) cmdTestMoveX(y1, y2);
    else Serial.println("Usage: TESTMOVEXS <yStart_mm> <yEnd_mm>");

  } else if (U.startsWith("MOVE ")) {
    int id = 0, goal = 0;
    if (sscanf(line.c_str(), "MOVE %d %d", &id, &goal) == 2) cmdMoveAdaptive((uint8_t)id, goal, 0);
    else Serial.println("Usage: MOVE <id> <abs_goal>");

  } else if (U.startsWith("INFO")) {
    int id = 0;
    if (sscanf(line.c_str(), "INFO %d", &id) == 1) cmdInfo((uint8_t)id);
    else Serial.println("Usage: INFO <id>");

  } else if (U.startsWith("TESTMOVE")) {
    int id = 0, cnt = TEST_DEFAULT_COUNT;
    if (sscanf(line.c_str(), "TESTMOVE %d %d", &id, &cnt) >= 1) cmdTestAdaptive((uint8_t)id, cnt);
    else Serial.println("Usage: TESTMOVE <id> [cycles]");

  } else if (U.startsWith("LEDON")) {
    int id = line.substring(6).toInt();
    dxl.ledOn(id);
    serial_printf("LED ON %d\n", id);

  } else if (U.startsWith("LEDOFF")) {
    int id = line.substring(7).toInt();
    dxl.ledOff(id);
    serial_printf("LED OFF %d\n", id);

  } else if (U.startsWith("SETLEN")) {
    arm_length_mm = line.substring(5).toFloat();
    has_arm_len = arm_length_mm > 0;
    serial_printf("Arm length set to %.1f mm\n", arm_length_mm);

  } else if (U.startsWith("SETZERO")) {
    int id = 0, ticks = 0;
    if (sscanf(line.c_str(), "SETZERO %d %d", &id, &ticks) == 2) {
      if (id == ID_ARM1) {
        tick_zero_arm1 = ticks;
        has_zero_arm1 = true;
      } else if (id == ID_ARM2) {
        tick_zero_arm2 = ticks;
        has_zero_arm2 = true;
      }
      serial_printf("Zero tick for %d = %d\n", id, ticks);
    } else {
      Serial.println("Usage: SETZERO <id> <ticks>");
    }

  } else if (U.startsWith("READ")) {
    print_status();

  } else {
    Serial.println("ERR: Unknown command");
  }

  Serial.println();
}