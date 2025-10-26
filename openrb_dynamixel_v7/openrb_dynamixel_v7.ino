#include <Dynamixel2Arduino.h>
#include <math.h>

// ============================================================
//                   GLOBAL STRUCTS & HELPERS
// ============================================================

// ---------------- Test stats ----------------
struct TestStats {
  long cntSmall = 0;
  long cntLarge = 0;
  long sumAbsErrSmall = 0;
  long sumAbsErrLarge = 0;
};

// Forward declarations to satisfy the Arduino pre-parser
void print_status();
void cmdInfo(uint8_t id);

// ---------------- Serial printf helper ----------------
template<typename... Args>
void serial_printf(const char *fmt, Args... args) {
  char buf[200];
  snprintf(buf, sizeof(buf), fmt, args...);
  Serial.print(buf);
}

// ---------------- Verbose mode ----------------
bool verboseOn = true;  // default at boot = ON

// ============================================================
//                       SERVO CONFIG
// ============================================================

#define DXL_SERIAL Serial1
#define DXL_DIR_PIN -1
#define PROTOCOL 2.0
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// Servo IDs (customize for your robot)
#define ID_ARM1 11
#define ID_ARM2 12
#define ID_WRIST 13
#define ID_GRIP1 14
#define ID_GRIP2 15
#define ID_BASE 16
uint8_t servo_ids[] = { ID_ARM1, ID_ARM2, ID_WRIST, ID_GRIP1, ID_GRIP2, ID_BASE };
const uint8_t SERVO_COUNT = sizeof(servo_ids) / sizeof(servo_ids[0]);

// ============================================================
//                     GLOBAL CONSTANTS
// ============================================================

const int STALL_CURRENT_mA = 1000;
const int TEMP_LIMIT_C = 70;

const int PV_MIN = 40, PV_MAX = 600;
const int PA_MIN = 40, PA_MAX = 600;

const int SMALL_MOVE_TICKS = 10;
const int FINE_WINDOW_TICKS = 40;
const int SMALL_OVER_UP_TICKS = 6;
const int SMALL_OVER_DOWN_TICKS = 2;
const int OVER_DELAY_MS_PER_TICK = 8;

const int COARSE_PV = 300;
const int COARSE_PA = 200;
const int COARSE_PG = 900;
const int COARSE_DG = 100;

const int FINE_PV = 80;
const int FINE_PA = 80;
const int FINE_PG = 1200;
const int FINE_DG = 10;
const int FINE_IG = 300;

const int FINISH_TOL_TICKS = 1;
const int MICROJOG_MAX_MS = 600;
const int MICROJOG_SAMPLE_DELAY = 8;

const uint32_t SMALL_MOVE_TIMEOUT_MS = 600;
const uint32_t LARGE_MOVE_TIMEOUT_MS = 3000;

const float TICKS_PER_REV = 4096.0f;
const float DEG_PER_TICK = 360.0f / TICKS_PER_REV;
const float PV_UNIT_RPM = 0.229f;
const float TIME_SAFETY_FACTOR = 1.20f;
const uint32_t MIN_WAIT_MS = 120;
const uint32_t EXTRA_SETTLE_MS = 50;

const int SMALL_THRESH_ABS = 20;
const int TEST_DEFAULT_COUNT = 40;
const int TEST_PAUSE_MS = 20;
const int STOW_TICK = 2000;
const int RAND_INCS[] = { 5, 19, 30, 50 };
const int RAND_INCS_N = sizeof(RAND_INCS) / sizeof(RAND_INCS[0]);

// ============================================================
//                      CALIBRATION FLAGS
// ============================================================

float arm_length_mm = 0;
int tick_zero_arm1 = -1;
int tick_zero_arm2 = -1;
int tick_zero_wrist = -1;
bool has_arm_len = false;
bool has_zero_arm1 = false;
bool has_zero_arm2 = false;
bool has_zero_wrist = false;

// ============================================================
//                      LED HELPERS
// ============================================================

void lOn(uint8_t id) {
  if (dxl.ping(id)) dxl.ledOn(id);
}
void lOff(uint8_t id) {
  if (dxl.ping(id)) dxl.ledOff(id);
}

// ============================================================
//                      STATUS & UTILS
// ============================================================

bool isInPosition(uint8_t id) {
  int s = dxl.readControlTableItem(ControlTableItem::MOVING_STATUS, id);
  return (s >= 0) && (s & 0x01);
}
bool isMoving(uint8_t id) {
  int s = dxl.readControlTableItem(ControlTableItem::MOVING_STATUS, id);
  return (s >= 0) && (s & 0x02);
}

// ---------------- Stall detection ----------------
bool checkStall(uint8_t id) {
  int curr = dxl.getPresentCurrent(id);
  int temp = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, id);
  if (temp >= TEMP_LIMIT_C || curr > STALL_CURRENT_mA) {
    dxl.torqueOff(id);
    lOn(id);
    serial_printf("STALL ID %d curr=%d temp=%d\n", id, curr, temp);
    return true;
  }
  return false;
}

// ============================================================
//                   TEST STATS HELPERS
// ============================================================

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

// ============================================================
//              PROFILE & TIMING UTILS (Dynamic Wait)
// ============================================================

static inline float pvToTicksPerSec(int pvLSB) {
  float rpm = (pvLSB <= 0 ? 0.0f : pvLSB * PV_UNIT_RPM);
  float rps = rpm / 60.0f;
  float tps = rps * TICKS_PER_REV;
  return max(tps, 1.0f);
}
static inline uint32_t estimateTravelTimeMs(uint8_t id, int deltaTicks) {
  int pv = dxl.readControlTableItem(ControlTableItem::PROFILE_VELOCITY, id);
  float tps = pvToTicksPerSec(pv);
  float moveMs = (fabs((float)deltaTicks) / tps) * 1000.0f * TIME_SAFETY_FACTOR;
  float total = MIN_WAIT_MS + moveMs + EXTRA_SETTLE_MS;
  return (uint32_t)total;
}

// ============================================================
//                     MOTION CONTROL CORE
// ============================================================

void moveServoSmart(uint8_t id, int rel_ticks) {
  if (!dxl.ping(id) || rel_ticks == 0) return;

  int start = dxl.getPresentPosition(id);
  int goal = start + rel_ticks;
  int absMove = abs(rel_ticks);
  bool smallMove = (absMove <= SMALL_MOVE_TICKS);

  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id, 885);
  dxl.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, id, 1);
  dxl.writeControlTableItem(ControlTableItem::DRIVE_MODE, id, 0);
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
    int over = (rel_ticks > 0 ? SMALL_OVER_UP_TICKS : -SMALL_OVER_DOWN_TICKS);
    dxl.setGoalPosition(id, goal + over);
    delay(abs(over) * OVER_DELAY_MS_PER_TICK);
  }

  dxl.setGoalPosition(id, goal);
  uint32_t maxTime = estimateTravelTimeMs(id, goal - start);
  uint32_t t0 = millis();

  while (millis() - t0 < maxTime) {
    if (checkStall(id)) return;
    if (!isMoving(id) && isInPosition(id)) break;
    delay(10);
  }

  // Micro-jog finisher
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

  if (verboseOn)
    serial_printf("MOVE END id=%d start=%d goal=%d final=%d err=%d\n",
                  id, start, goal, pos, err);
}

// ============================================================
//                  WRIST COMPENSATION (keep base orientation)
// ============================================================

void updateWristKeepBaseRef(int arm2_start, int wrist_start) {
  if (!has_zero_arm2 || !has_zero_wrist) return;

  int arm2_now = dxl.getPresentPosition(ID_ARM2);
  int delta_ref = (wrist_start - tick_zero_wrist) - (arm2_start - tick_zero_arm2);
  int wrist_goal = tick_zero_wrist + (arm2_now - tick_zero_arm2) + delta_ref;
  wrist_goal = constrain(wrist_goal, 0, 4095);

  dxl.setGoalPosition(ID_WRIST, wrist_goal);

  if (verboseOn)
    serial_printf("WRIST keep-base: arm2_now=%d wrist_goal=%d delta_ref=%d\n",
                  arm2_now, wrist_goal, delta_ref);
}

// ============================================================
//                   ADAPTIVE MOVE
// ============================================================

void cmdMoveAdaptive(uint8_t id, int goal, uint8_t depth = 0) {
  if (!dxl.ping(id)) return;
  if (depth > 2) return;

  int start = dxl.getPresentPosition(id);
  dxl.torqueOn(id);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id, 1023);
  lOn(id);

  dxl.setGoalPosition(id, goal);
  uint32_t maxTime = estimateTravelTimeMs(id, goal - start);
  uint32_t t0 = millis();
  while (isMoving(id) && (millis() - t0 < maxTime)) delay(10);

  int pos = dxl.getPresentPosition(id);
  int diff = goal - pos;
  if (abs(diff) > 5) {
    int trim = (diff > 0 ? 1 : -1);
    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, goal + trim);
    delay(15);
    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, goal);
  }

  delay(20);
  pos = dxl.getPresentPosition(id);
  diff = goal - pos;
  lOff(id);

  if (verboseOn) {
    for (int i = 0; i < depth; i++) Serial.print(" ");
    serial_printf("MOVE END id=%d start=%d goal=%d final=%d err=%d (ADAPTIVE d=%d)\n",
                  id, start, goal, pos, diff, depth);
  }

  if (abs(diff) > 6 && depth < 2) {
    int subGoal = goal + (diff > 0 ? 10 : -10);
    subGoal = constrain(subGoal, 0, 4095);
    cmdMoveAdaptive(id, subGoal, depth + 1);
    cmdMoveAdaptive(id, goal, depth + 1);
  }
}

// ============================================================
//                   MOVEY / MOVEX (with wrist)
// ============================================================

void cmdMoveY(int rel_y_ticks) {
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM1) || !dxl.ping(ID_WRIST)) {
    if (!dxl.ping(ID_ARM1)) Serial.println("ERR: servo 11 (arm1) not connected");
    if (!dxl.ping(ID_ARM2)) Serial.println("ERR: servo 12 (arm1) not connected");
    if (!dxl.ping(ID_WRIST)) Serial.println("ERR: servo 13 (wrist) not connected");
    return;
  }
  if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2 || !has_zero_wrist) {
    if (!has_arm_len) Serial.println("ERR: SETLEN missing");
    if (!has_zero_arm1) Serial.println("ERR: SETZERO 11 (arm1) missing");
    if (!has_zero_arm2) Serial.println("ERR: SETZERO 12 (arm2) missing");
    if (!has_zero_wrist) Serial.println("ERR: SETZERO 13 (wrist) missing");
    return;
  }

  int pos1 = dxl.getPresentPosition(ID_ARM1);
  int pos2 = dxl.getPresentPosition(ID_ARM2);
  int wrist_start = dxl.getPresentPosition(ID_WRIST);
  int arm2_start = pos2;
  int goal1 = pos1 + rel_y_ticks;
  int goal2 = pos2 - rel_y_ticks;

  const int steps = 10;
  const int slice1 = rel_y_ticks / steps;
  const int slice2 = -rel_y_ticks / steps;

  for (int i = 1; i <= steps; i++) {
    int subGoal1 = pos1 + slice1 * i;
    int subGoal2 = pos2 + slice2 * i;
    moveServoSmart(ID_ARM1, subGoal1 - dxl.getPresentPosition(ID_ARM1));
    moveServoSmart(ID_ARM2, subGoal2 - dxl.getPresentPosition(ID_ARM2));
    updateWristKeepBaseRef(arm2_start, wrist_start);
  }

  cmdMoveAdaptive(ID_ARM1, goal1, 0);
  cmdMoveAdaptive(ID_ARM2, goal2, 0);
  updateWristKeepBaseRef(arm2_start, wrist_start);
}

void cmdMoveX(int rel_ticks) {
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM1) || !dxl.ping(ID_WRIST)) {
    if (!dxl.ping(ID_ARM1)) Serial.println("ERR: servo 11 (arm1) not connected");
    if (!dxl.ping(ID_ARM2)) Serial.println("ERR: servo 12 (arm1) not connected");
    if (!dxl.ping(ID_WRIST)) Serial.println("ERR: servo 13 (wrist) not connected");
    return;
  }
  if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2 || !has_zero_wrist) {
    if (!has_arm_len) Serial.println("ERR: SETLEN missing");
    if (!has_zero_arm1) Serial.println("ERR: SETZERO 11 (arm1) missing");
    if (!has_zero_arm2) Serial.println("ERR: SETZERO 12 (arm2) missing");
    if (!has_zero_wrist) Serial.println("ERR: SETZERO 13 (wrist) missing");
    return;
  }

  int arm2_start = dxl.getPresentPosition(ID_ARM2);
  int wrist_start = dxl.getPresentPosition(ID_WRIST);
  moveServoSmart(ID_ARM1, +rel_ticks);
  moveServoSmart(ID_ARM2, +rel_ticks);
  updateWristKeepBaseRef(arm2_start, wrist_start);
}

// ============================================================
//                        READXY
// ============================================================

const float ARM_LENGTH_MM = 48.0f;
const float TICK_TO_RAD = (2.0f * M_PI) / 4096.0f;
const int TICK_ZERO = 2000;

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

void cmdReadXY() {
  if (!dxl.ping(ID_ARM1) || !dxl.ping(ID_ARM1) || !dxl.ping(ID_WRIST)) {
    if (!dxl.ping(ID_ARM1)) Serial.println("ERR: servo 11 (arm1) not connected");
    if (!dxl.ping(ID_ARM2)) Serial.println("ERR: servo 12 (arm1) not connected");
    if (!dxl.ping(ID_WRIST)) Serial.println("ERR: servo 13 (wrist) not connected");
    return;
  }
  if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2 || !has_zero_wrist) {
    if (!has_arm_len) Serial.println("ERR: SETLEN missing");
    if (!has_zero_arm1) Serial.println("ERR: SETZERO 11 (arm1) missing");
    if (!has_zero_arm2) Serial.println("ERR: SETZERO 12 (arm2) missing");
    if (!has_zero_wrist) Serial.println("ERR: SETZERO 13 (wrist) missing");
    return;
  }
  int a1 = dxl.getPresentPosition(ID_ARM1);
  int a2 = dxl.getPresentPosition(ID_ARM2);
  float x_mm, y_mm;
  computeXY(a1, a2, x_mm, y_mm);
  serial_printf("READXY Arm1=%d Arm2=%d | X=%.3f mm  Y=%.3f mm\n", a1, a2, x_mm, y_mm);
}

// ----------------------------------------------------
// READ: diagnostics table (position, current, temp)
// ----------------------------------------------------
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

// ----------------------------------------------------
// INFO <id> : print key control-table data for one servo
// ----------------------------------------------------
void cmdInfo(uint8_t id) {
  if (!dxl.ping(id)) {
    serial_printf("Servo %d not found\n", id);
    return;
  }

  int op = dxl.readControlTableItem(ControlTableItem::OPERATING_MODE, id);
  int drv = dxl.readControlTableItem(ControlTableItem::DRIVE_MODE, id);
  int pv = dxl.readControlTableItem(ControlTableItem::PROFILE_VELOCITY, id);
  int pa = dxl.readControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id);
  int pos = dxl.getPresentPosition(id);
  int minL = dxl.readControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id);
  int maxL = dxl.readControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id);

  float rpm = pv * PV_UNIT_RPM;
  float tps = pvToTicksPerSec(pv);
  float spanTicks = (maxL > minL) ? (float)(maxL - minL) : TICKS_PER_REV;
  float spanDeg = spanTicks * DEG_PER_TICK;

  serial_printf("INFO id=%d\n", id);
  serial_printf("  OperatingMode : %d\n", op);
  serial_printf("  DriveMode     : %d (bit0=%s-profile)\n", drv, (drv & 0x01) ? "TIME" : "VELOCITY");
  serial_printf("  Profile Vel   : %d  (≈ %.3f rpm, %.1f ticks/s)\n", pv, rpm, tps);
  serial_printf("  Profile Accel : %d\n", pa);
  serial_printf("  PosLimits     : min=%d  max=%d  (span≈ %.1f°)\n", minL, maxL, spanDeg);
  serial_printf("  Present Pos   : %d (≈ %.2f°)\n", pos, pos * DEG_PER_TICK);
}

// ============================================================
//                   SETLIMIT / SETLEN / SETZERO
// ============================================================

void cmdSetLimit(uint8_t id, int minL, int maxL) {
  if (!dxl.ping(id)) return;
  dxl.writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id, minL);
  dxl.writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id, maxL);
  serial_printf("SETLIMIT id=%d min=%d max=%d\n", id, minL, maxL);
}

// ----------------------------------------------------
// TESTMOVE  <id> [cycles]  – test single-servo adaptive moves
// ----------------------------------------------------
void cmdTestMove(uint8_t id, int count = TEST_DEFAULT_COUNT) {
  if (!dxl.ping(id)) {
    serial_printf("Servo %d not found\n", id);
    return;
  }
  if (count <= 0) count = TEST_DEFAULT_COUNT;

  TestStats stats;
  serial_printf("TESTMOVE start id=%d cycles=%d\n", id, count);

  for (int iter = 0; iter < count; ++iter) {
    // small ± moves
    cmdMoveAdaptive(id, STOW_TICK, 0);
    cmdMoveAdaptive(id, STOW_TICK, 0);
    int baseline = dxl.getPresentPosition(id);
    for (int i = 0; i < RAND_INCS_N; ++i) {
      int goalUp = baseline + RAND_INCS[i];
      int goalDn = baseline - RAND_INCS[i];
      cmdMoveAdaptive(id, goalUp, 0);
      cmdMoveAdaptive(id, goalDn, 0);
      int final = dxl.getPresentPosition(id);
      tst::addErr(stats, RAND_INCS[i], abs(final - goalDn));
    }
    // large excursion
    cmdMoveAdaptive(id, STOW_TICK + 1000, 0);
    cmdMoveAdaptive(id, STOW_TICK, 0);
  }

  tst::printStats("TESTMOVE", id, stats);
  serial_printf("TESTMOVE end id=%d\n", id);
}

// ----------------------------------------------------
// TESTMOVEX <ticks>  – move both arms laterally
// ----------------------------------------------------
void cmdTestMoveX(int rel_ticks) {
  Serial.println("=== TESTMOVEX ===");
  if (!has_arm_len || !has_zero_arm1 || !has_zero_arm2) {
    Serial.println("ERR: not calibrated");
    return;
  }
  int start1 = dxl.getPresentPosition(ID_ARM1);
  int start2 = dxl.getPresentPosition(ID_ARM2);
  cmdMoveX(rel_ticks);
  int end1 = dxl.getPresentPosition(ID_ARM1);
  int end2 = dxl.getPresentPosition(ID_ARM2);
  serial_printf("TESTMOVEX Δticks=%d | Arm1:%d→%d  Arm2:%d→%d\n",
                rel_ticks, start1, end1, start2, end2);
  Serial.println("=================\n");
}

// ----------------------------------------------------
// TESTMOVEY <y1_mm> <y2_mm> – geometric Y-axis test
// ----------------------------------------------------
void cmdTestMoveY(float yStart_mm, float yEnd_mm, int steps = 10) {
  Serial.println("=== TESTMOVEY (vertical geometry) ===");
  serial_printf("Arm length=%.1fmm  zeroTick=%d\n", ARM_LENGTH_MM, TICK_ZERO);

  float thetaStart = asin(constrain(yStart_mm / ARM_LENGTH_MM, -1.0f, 1.0f));
  float thetaEnd = asin(constrain(yEnd_mm / ARM_LENGTH_MM, -1.0f, 1.0f));
  int ticksStart = TICK_ZERO + (int)(thetaStart / TICK_TO_RAD);
  int ticksEnd = TICK_ZERO + (int)(thetaEnd / TICK_TO_RAD);
  int deltaTicks = ticksEnd - ticksStart;

  serial_printf("Ystart=%.2fmm (ticks=%d)  Yend=%.2fmm (ticks=%d)  Δticks=%d\n",
                yStart_mm, ticksStart, yEnd_mm, ticksEnd, deltaTicks);

  // bring both arms to start
  cmdMoveAdaptive(ID_ARM1, ticksStart, 0);
  cmdMoveAdaptive(ID_ARM2, ticksStart, 0);
  delay(300);

  int pos1 = dxl.getPresentPosition(ID_ARM1);
  int pos2 = dxl.getPresentPosition(ID_ARM2);
  float x0, y0;
  computeXY(pos1, pos2, x0, y0);
  serial_printf("Start measured: A1=%d A2=%d  X=%.2f Y=%.2f\n", pos1, pos2, x0, y0);

  int stepTicks = deltaTicks / steps;
  for (int i = 1; i <= steps; i++) {
    int goal1 = ticksStart + stepTicks * i;
    int goal2 = ticksStart - stepTicks * i;
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

  int f1 = dxl.getPresentPosition(ID_ARM1);
  int f2 = dxl.getPresentPosition(ID_ARM2);
  float xF, yF;
  computeXY(f1, f2, xF, yF);
  serial_printf("End measured: A1=%d A2=%d  X=%.3f Y=%.3f  ΔY=%.3f  X drift=%.3f\n",
                f1, f2, xF, yF, yF - y0, xF - x0);
  Serial.println("=== end test ===\n");
}

// ============================================================
//                          SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  dxl.begin(57600);
  dxl.setPortProtocolVersion(PROTOCOL);

  Serial.println("=== OpenRB Arm Motion Controller v10 (Adaptive + Verbose + WristComp) ===");
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    uint8_t id = servo_ids[i];
    if (dxl.ping(id)) {
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_POSITION);
      dxl.torqueOn(id);
      serial_printf("Servo %d OK\n", id);
    }
  }

  Serial.println("\nSupported Commands (VERBOSE ON):");
  Serial.println("  VERBOSEON / VERBOSEOFF         - toggle console verbosity");
  Serial.println("  SETLIMIT <id> <min> <max>      - set soft position limits");
  Serial.println("  SETLEN  <mm>                   - set arm length");
  Serial.println("  SETZERO <id> <ticks>           - set zero tick (include WRIST)");
  Serial.println("  MOVE  <id> <absgoal|±rel>      - adaptive move one servo");
  Serial.println("  MOVEY <ticks>                  - vertical (mirror arm1/arm2)");
  Serial.println("  MOVEX <ticks>                  - lateral (same dir)");
  Serial.println("  READ                           - diagnostics table");
  Serial.println("  READXY                         - compute current X/Y midpoint");
  Serial.println("  TESTMOVE  <id> [cycles]        - adaptive single-servo test");
  Serial.println("  TESTMOVEX <ticks>              - lateral X-axis test");
  Serial.println("  TESTMOVEY <y1_mm> <y2_mm>      - vertical Y-axis geometry test");
  Serial.println("  INFO  <id>                     - show servo mode/limits/profiles");
  Serial.println("  LEDON <id> / LEDOFF <id>");
  Serial.println("  (Boot default = VERBOSE ON)");
}

// ============================================================
//                            LOOP
// ============================================================

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  String U = line;
  U.toUpperCase();

  if (U.startsWith("VERBOSEON")) {
    verboseOn = true;
    Serial.println("Verbose ON");
  } else if (U.startsWith("VERBOSEOFF")) {
    verboseOn = false;
    Serial.println("Verbose OFF");
  }

  else if (U.startsWith("SETLIMIT")) {
    int id, minL, maxL;
    if (sscanf(line.c_str(), "SETLIMIT %d %d %d", &id, &minL, &maxL) == 3) cmdSetLimit(id, minL, maxL);
    else Serial.println("Usage: SETLIMIT <id> <min> <max>");
  }

  else if (U.startsWith("SETLEN")) {
    arm_length_mm = line.substring(6).toFloat();
    has_arm_len = arm_length_mm > 0;
    serial_printf("Arm length set to %.1f mm\n", arm_length_mm);
  }

  else if (U.startsWith("SETZERO")) {
    int id = 0, ticks = 0;
    if (sscanf(line.c_str(), "SETZERO %d %d", &id, &ticks) == 2) {
      if (id == ID_ARM1) {
        tick_zero_arm1 = ticks;
        has_zero_arm1 = true;
      } else if (id == ID_ARM2) {
        tick_zero_arm2 = ticks;
        has_zero_arm2 = true;
      } else if (id == ID_WRIST) {
        tick_zero_wrist = ticks;
        has_zero_wrist = true;
      }
      serial_printf("Zero tick for %d = %d\n", id, ticks);
    } else Serial.println("Usage: SETZERO <id> <ticks>");
  }

  else if (U.startsWith("MOVE ")) {
    int id = 0;
    char valStr[16] = { 0 };
    if (sscanf(line.c_str(), "MOVE %d %15s", &id, valStr) == 2) {
      int current = dxl.getPresentPosition(id);
      int goal = 0;
      if (valStr[0] == '+' || valStr[0] == '-') {
        goal = current + atoi(valStr);
        serial_printf("Relative move: start=%d rel=%d goal=%d\n", current, atoi(valStr), goal);
      } else goal = atoi(valStr);
      cmdMoveAdaptive((uint8_t)id, goal, 0);
    } else Serial.println("Usage: MOVE <id> <absgoal|±rel>");
  }

  else if (U.startsWith("MOVEY")) {
    int val = line.substring(6).toInt();
    cmdMoveY(val);
  }

  else if (U.startsWith("MOVEX")) {
    int val = line.substring(6).toInt();
    cmdMoveX(val);
  }

  else if (U.startsWith("READXY")) {
    cmdReadXY();
  }

  else if (U.startsWith("READ")) {
    print_status();
  }

  else if (U.startsWith("INFO")) {
    int id = 0;
    if (sscanf(line.c_str(), "INFO %d", &id) == 1) cmdInfo(id);
    else Serial.println("Usage: INFO <id>");
  }

  else if (U.startsWith("LEDON")) {
    int id = line.substring(6).toInt();
    dxl.ledOn(id);
    serial_printf("LED ON %d\n", id);
  }

  else if (U.startsWith("LEDOFF")) {
    int id = line.substring(7).toInt();
    dxl.ledOff(id);
    serial_printf("LED OFF %d\n", id);
  }

  else if (U.startsWith("TESTMOVEY")) {
    float y1 = 0, y2 = 0;
    if (sscanf(line.c_str(), "TESTMOVEY %f %f", &y1, &y2) == 2)
      cmdTestMoveY(y1, y2);
    else Serial.println("Usage: TESTMOVEY <yStart_mm> <yEnd_mm>");
  }

  else if (U.startsWith("TESTMOVEX")) {
    int v = line.substring(9).toInt();
    cmdTestMoveX(v);
  }

  else if (U.startsWith("TESTMOVE")) {
    int id = 0, cnt = TEST_DEFAULT_COUNT;
    if (sscanf(line.c_str(), "TESTMOVE %d %d", &id, &cnt) >= 1)
      cmdTestMove((uint8_t)id, cnt);
    else Serial.println("Usage: TESTMOVE <id> [cycles]");
  }

  else Serial.println("ERR: Unknown command");

  Serial.println();
}
