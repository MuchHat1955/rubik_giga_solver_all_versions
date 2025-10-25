#include <Dynamixel2Arduino.h>
#include <math.h>

// ---------------- HARDWARE ----------------
#define DXL_SERIAL Serial1
#define DXL_DIR_PIN -1  // OpenRB-150/half-duplex: no DIR pin
#define PROTOCOL 2.0f

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ---------------- CONFIGURABLE IDs ----------------
#define ID_LEFT 11
#define ID_RIGHT 12
#define ID_WRIST 13
uint8_t servo_ids[] = { ID_LEFT, ID_RIGHT, ID_WRIST, 14, 15, 16 };
const uint8_t SERVO_COUNT = sizeof(servo_ids) / sizeof(servo_ids[0]);

// ---------------- CONSTANTS ----------------
const int STALL_CURRENT_mA = 1000;  // stall if current exceeds this
const uint8_t CHECK_INTERVAL_MS = 20;

// For stall detection during smooth moves
const int LOAD_CURRENT_mA = 800;     // "high current" threshold
const int VELOCITY_MIN = 3;          // near-stopped if |vel| < this (ticks/s)
const uint16_t STALL_TIME_MS = 300;  // must persist this long

// ---------------- CALIBRATION ----------------
float arm_length_mm = 0;  // 0 = unset
int tick_zero_left = -1;  // -1 = unset
int tick_zero_right = -1;

bool has_L = false;
bool has_ZL = false;
bool has_ZR = false;

// ---------------- FLAGS ----------------
bool stalled_flags[6] = { false, false, false, false, false, false };
unsigned long last_check_ms = 0;

// ----------------------------------------------------
// helpers
// ----------------------------------------------------
int index_of_id(uint8_t id) {
  for (uint8_t i = 0; i < SERVO_COUNT; i++)
    if (servo_ids[i] == id) return i;
  return -1;
}
bool is_managed(uint8_t id) {
  return index_of_id(id) >= 0;
}
bool stalled_for_id(uint8_t id) {
  int i = index_of_id(id);
  return (i >= 0) ? stalled_flags[i] : false;
}

// ----------------------------------------------------
// LED helpers
// ----------------------------------------------------
void ledOn(uint8_t id) {
  if (dxl.ping(id)) dxl.ledOn(id);
}
void ledOff(uint8_t id) {
  if (dxl.ping(id)) dxl.ledOff(id);
}

void ledBlink(uint8_t id, uint16_t period_ms = 500) {
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

// ----------------------------------------------------
// Small single-servo micro-move with overshoot
// ----------------------------------------------------
void smallSingleServoMove(uint8_t id,
                          int rel_ticks,
                          uint32_t max_time_ms = 150,
                          int overshoot_ticks = 2) {
  if (!dxl.ping(id)) return;
  if (rel_ticks == 0) return;

  dxl.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, id, 1);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, 100);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, 30);

  int tq = dxl.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id);
  if (!tq) dxl.torqueOn(id);

  int start = dxl.getPresentPosition(id);
  Serial.print("STEP ID=");
  Serial.print(id);
  Serial.print(" rel=");
  Serial.print(rel_ticks);
  Serial.print(" goal=");
  Serial.println(start + rel_ticks);
  int goal = start + rel_ticks;
  if (rel_ticks > 10) goal = start + 10;
  if (rel_ticks < -10) goal = start - 10;

  int overshoot_goal = goal + (rel_ticks > 0 ? overshoot_ticks : -overshoot_ticks);
  dxl.setGoalPosition(id, overshoot_goal);
  delay((overshoot_ticks < 0 ? -overshoot_ticks : overshoot_ticks) * 8);
  dxl.setGoalPosition(id, goal);

  const int TOL = 1;
  uint32_t t0 = millis();
  while (millis() - t0 < max_time_ms) {
    int pos = dxl.getPresentPosition(id);
    if (abs(pos - goal) <= TOL) break;
    delay(5);
  }

  int finalPos = dxl.getPresentPosition(id);
  Serial.print("STEP done ID=");
  Serial.print(id);
  Serial.print(" final=");
  Serial.println(finalPos);
  Serial.println();
}

// ----------------------------------------------------
// Monitor stalls (subset)
// ----------------------------------------------------
void monitor_stalls_subset(const uint8_t* ids, uint8_t count) {
  if (millis() - last_check_ms < CHECK_INTERVAL_MS) return;
  last_check_ms = millis();

  for (uint8_t i = 0; i < count; i++) {
    uint8_t id = ids[i];
    if (!dxl.ping(id)) continue;

    int curr_mA = dxl.getPresentCurrent(id);  // mA (D2A converts units)
    int tempC = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, id);

    if (curr_mA > STALL_CURRENT_mA || tempC > 70) {
      int idx = index_of_id(id);
      if (idx >= 0) stalled_flags[idx] = true;
      dxl.torqueOff(id);
      ledBlink(id, 300);
    }
  }
}

// ----------------------------------------------------
// MOVEY: symmetric vertical move
// ----------------------------------------------------
void moveY(int rel_ticks) {
  if (!has_L || !has_ZL || !has_ZR) {
    Serial.println("ERR: arm not calibrated (L/Z missing)");
    return;
  }
  if (!dxl.ping(ID_LEFT) || !dxl.ping(ID_RIGHT)) return;

  dxl.torqueOn(ID_LEFT);
  dxl.torqueOn(ID_RIGHT);

  int tL = dxl.getPresentPosition(ID_LEFT);
  int tR = dxl.getPresentPosition(ID_RIGHT);
  int gL = tL + rel_ticks;
  int gR = tR - rel_ticks;

  dxl.setGoalPosition(ID_LEFT, gL);
  dxl.setGoalPosition(ID_RIGHT, gR);
  delay(10);
}

// ----------------------------------------------------
// MOVEX: differential motion for small lateral adjust
// ----------------------------------------------------
void moveX(int rel_ticks) {
  if (!has_L || !has_ZL || !has_ZR) {
    Serial.println("ERR: arm not calibrated (L/Z missing)");
    return;
  }
  if (!dxl.ping(ID_LEFT) || !dxl.ping(ID_RIGHT)) return;

  dxl.torqueOn(ID_LEFT);
  dxl.torqueOn(ID_RIGHT);

  int tL = dxl.getPresentPosition(ID_LEFT);
  int tR = dxl.getPresentPosition(ID_RIGHT);
  int gL = tL + rel_ticks;
  int gR = tR + rel_ticks;

  dxl.setGoalPosition(ID_LEFT, gL);
  dxl.setGoalPosition(ID_RIGHT, gR);
  delay(10);
}

// ----------------------------------------------------
// Reset motion parameters to Dynamixel defaults
// ----------------------------------------------------
void moveDefaults(uint8_t id, int goal = INT32_MIN) {
  if (!dxl.ping(id)) {
    Serial.print("Servo ");
    Serial.print(id);
    Serial.println(" not responding!");
    return;
  }

  Serial.print("MOVE default start. ID=");
  Serial.print(id);
  Serial.print(" goal=");

  // Restore factory-like motion behavior for XL-series
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, 0);      // unlimited
  dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, 0);  // unlimited
  dxl.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, id, 10);     // 10-tick window

  // If goal provided, move to it
  if (goal != INT32_MIN) {
    dxl.torqueOn(id);
    dxl.setGoalPosition(id, goal);
    Serial.print(" → moving to ");
    Serial.println(goal);

    // Wait until motion completes
    unsigned long t0 = millis();
    while (millis() - t0 < 3000) {
      int moving = dxl.readControlTableItem(ControlTableItem::MOVING, id);
      if (!moving) break;
      delay(10);
    }
    int pos = dxl.getPresentPosition(id);
    Serial.print("MOVE default done. ID=");
    Serial.print(id);
    Serial.print("  final=");
    Serial.println(pos);
  } else {
    Serial.println();
  }
}

// ----------------------------------------------------
// Print current status
// ----------------------------------------------------
void print_status(const uint8_t* ids, uint8_t count) {
  Serial.print("STATUS ");
  for (uint8_t i = 0; i < count; i++) {
    uint8_t id = ids[i];
    if (!dxl.ping(id)) {
      Serial.print(id);
      Serial.print(" X X X");
      if (i + 1 < count) Serial.print(' ');
      continue;
    }
    int pos = dxl.getPresentPosition(id);
    Serial.print(id);
    Serial.print(':');
    Serial.print(pos);
    if (i + 1 < count) Serial.print(' ');
  }
  Serial.println();
}

// ----------------------------------------------------
// Move one servo using Time-Profile mode
// ----------------------------------------------------
void moveTime(uint8_t id, int goal_ticks, uint32_t duration_ms) {
  if (!dxl.ping(id)) return;

  // Switch to Time-Profile (DRIVE_MODE bit2 = 1)
  uint8_t drive_mode = dxl.readControlTableItem(ControlTableItem::DRIVE_MODE, id);
  dxl.torqueOff(id);
  drive_mode |= 0b00000100;
  dxl.writeControlTableItem(ControlTableItem::DRIVE_MODE, id, drive_mode);
  dxl.torqueOn(id);

  // PROFILE_VELOCITY = total move time (ms)
  // PROFILE_ACCELERATION = accel time (ms)
  uint32_t accel_time = (uint32_t)(duration_ms * 0.2f);
  if (accel_time < 10) accel_time = 10;

  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, duration_ms);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, accel_time);

  dxl.setGoalPosition(id, goal_ticks);

  Serial.print("MOVETIME start ID=");
  Serial.print(id);
  Serial.print(" goal=");
  Serial.print(goal_ticks);
  Serial.print(" duration=");
  Serial.println(duration_ms);

  // Wait until finished or timeout
  uint32_t t0 = millis();
  while (millis() - t0 < (duration_ms + 200)) {
    int moving = dxl.readControlTableItem(ControlTableItem::MOVING, id);
    if (!moving) break;
    delay(5);
  }

  // Return to Velocity-Profile (bit2 = 0)
  dxl.torqueOff(id);
  drive_mode &= (uint8_t)~0b00000100;
  dxl.writeControlTableItem(ControlTableItem::DRIVE_MODE, id, drive_mode);
  dxl.torqueOn(id);

  int finalPos = dxl.getPresentPosition(id);
  Serial.print("MOVETIME done ID=");
  Serial.print(id);
  Serial.print("  final=");
  Serial.println(finalPos);
  Serial.println();
}

// ----------------------------------------------------
// Smooth stepped move with stall-stop
// ----------------------------------------------------
const uint32_t DEF_ACCEL = 50;   // smaller = smoother start/stop
const uint32_t DEF_VELOC = 150;  // smaller = slower motion

static inline void move_def(uint8_t id, int target_pos, uint32_t accel = DEF_ACCEL, uint32_t veloc = DEF_VELOC) {
  if (!dxl.ping(id)) return;
  dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, accel);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, veloc);
  dxl.setGoalPosition(id, target_pos);
}

// Perform a smooth multi-step move; aborts if stall is detected
void moveSmooth(uint8_t id, int goal_ticks, int steps, uint16_t step_delay_ms) {
  if (!dxl.ping(id)) return;
  if (steps < 1) steps = 1;

  Serial.print("MOVE smooth start. ID=");
  Serial.print(id);
  Serial.print(" goal=");
  Serial.println(goal_ticks);

  dxl.torqueOn(id);
  dxl.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, id, 4);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, DEF_ACCEL);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, DEF_VELOC);

  int start = dxl.getPresentPosition(id);
  int delta = goal_ticks - start;

  unsigned long stall_start = 0;

  for (int i = 1; i <= steps; i++) {
    int interm = start + (int)((long)delta * i / steps);
    dxl.setGoalPosition(id, interm);

    // Wait a bit while checking for stall
    unsigned long seg_t0 = millis();
    while (millis() - seg_t0 < step_delay_ms) {
      int vel = dxl.getPresentVelocity(id);
      int curr = dxl.getPresentCurrent(id);

      if (abs(vel) < VELOCITY_MIN && abs(curr) > LOAD_CURRENT_mA) {
        if (stall_start == 0) stall_start = millis();
        if (millis() - stall_start > STALL_TIME_MS) {
          Serial.println("⚠️ Stall detected! Torque OFF.");
          dxl.torqueOff(id);
          return;
        }
      } else {
        stall_start = 0;
      }
      delay(5);
    }
  }

  // Final settle check (reach within small tolerance)
  const int TOL = 5;
  unsigned long t0 = millis();
  while (millis() - t0 < 300) {
    int pos = dxl.getPresentPosition(id);
    if (abs(pos - goal_ticks) <= TOL) break;
    delay(5);
  }

  Serial.print("MOVE smooth done. ID=");
  Serial.print(id);
  Serial.print(" final=");
  Serial.print((int)dxl.getPresentPosition(id));
  Serial.print(" err=");
  Serial.print((int)dxl.getPresentPosition(id) - goal_ticks);
  Serial.println();
}

// ----------------------------------------------------
// Arduino setup / loop
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB */
  }

  dxl.begin(57600);
  dxl.setPortProtocolVersion(PROTOCOL);

  Serial.println("=== OpenRB Arm Motion Controller (with calibration safety) ===");

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    uint8_t id = servo_ids[i];
    if (dxl.ping(id)) {
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_POSITION);
      dxl.torqueOn(id);
      stalled_flags[i] = false;
      ledOff(id);
      Serial.print("Servo ");
      Serial.print(id);
      Serial.println(" OK");
    } else {
      Serial.print("Servo ");
      Serial.print(id);
      Serial.println(" not found!");
    }
  }

  Serial.println("Supported Commands:");
  Serial.println("  MOVE     <id> <goal> [steps]     (smooth stepped move)");
  Serial.println("  MOVEDEF  <id>                    (restore default motion params)");
  Serial.println("  MOVETIME <id> <goal> <ms>        (time-profile move)");
  Serial.println("  MOVEY    <ticks>                 (vertical, symmetric)");
  Serial.println("  MOVEX    <ticks>                 (lateral adjust, same direction)");
  Serial.println("  STEP     <id> <rel> [ms] [ov]    (micro step with overshoot)");
  Serial.println("  SETL     <mm>                    (set arm length)");
  Serial.println("  SETZ     <id> <ticks>            (set zero tick for id)");
  Serial.println("  READ                             (show current ticks)");
  Serial.println();
}


void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("MOVEY")) {
    int val = line.substring(6).toInt();
    moveY(val);

  } else if (line.startsWith("MOVEX")) {
    int val = line.substring(6).toInt();
    moveX(val);

  } else if (line.startsWith("STEP")) {
    int id = 0, rel = 0, ms = 150, ov = 2;
    sscanf(line.c_str(), "STEP %d %d %d %d", &id, &rel, &ms, &ov);
    smallSingleServoMove((uint8_t)id, rel, (uint32_t)ms, ov);

  } else if (line.startsWith("SETL")) {
    arm_length_mm = line.substring(5).toFloat();
    has_L = arm_length_mm > 0;
    Serial.print("Arm length set to ");
    Serial.print(arm_length_mm);
    Serial.println(" mm");

  } else if (line.startsWith("SETZ")) {
    int id = 0, ticks = 0;
    sscanf(line.c_str(), "SETZ %d %d", &id, &ticks);
    if (id == ID_LEFT) {
      tick_zero_left = ticks;
      has_ZL = true;
    } else if (id == ID_RIGHT) {
      tick_zero_right = ticks;
      has_ZR = true;
    }
    Serial.print("Zero tick for ");
    Serial.print(id);
    Serial.print(" = ");
    Serial.println(ticks);

  } else if (line.startsWith("READ")) {
    print_status(servo_ids, SERVO_COUNT);

  } else if (line.startsWith("MOVEDEF")) {
    int id = 0, goal = INT32_MIN;
    int n = sscanf(line.c_str(), "MOVEDEF %d %d", &id, &goal);
    if (n == 1) moveDefaults((uint8_t)id);
    else if (n == 2) moveDefaults((uint8_t)id, goal);
  } else if (line.startsWith("MOVETIME")) {
    int id = 0, goal = 0, ms = 1000;
    sscanf(line.c_str(), "MOVETIME %d %d %d", &id, &goal, &ms);
    moveTime((uint8_t)id, goal, (uint32_t)ms);

  } else if (line.startsWith("MOVE")) {
    int id = 0, goal = 0, steps = 10;
    sscanf(line.c_str(), "MOVE %d %d %d", &id, &goal, &steps);
    moveSmooth((uint8_t)id, goal, steps, 50);  // 50 ms between steps

  } else {
    Serial.println("ERR CMD");
  }

  // Optional background stall monitor
  monitor_stalls_subset(servo_ids, SERVO_COUNT);

  delay(20);
}
