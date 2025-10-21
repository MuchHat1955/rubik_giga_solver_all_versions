/********************************************************************
 * SERVO BASIC FUNCTIONS
 ********************************************************************/

#include <Dynamixel2Arduino.h>

// Adjust stall threshold (mA)
const int STALL_THRESHOLD = 600;

// Forward declaration
void moveServoSmooth(int id, int target, int minv, int maxv);

// Servo name → ID mapping
struct ServoMap {
  const char *name;
  uint8_t id;
};

ServoMap servoTable[] = {
  { "arm1", 1 },
  { "arm2", 2 },
  { "wrist", 3 },
  { "grip1", 4 },
  { "grip2", 5 },
  { "base", 6 }
};

const int NUM_SERVOS = sizeof(servoTable) / sizeof(servoTable[0]);

// ------------------------------------------------------------
// Check if a string corresponds to a valid servo name
// ------------------------------------------------------------
bool isServo(const String &name) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (name.equalsIgnoreCase(servoTable[i].name))
      return true;
  }
  return false;
}

// ------------------------------------------------------------
// Lookup servo ID from name
// ------------------------------------------------------------
int getServoId(const String &name) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (name.equalsIgnoreCase(servoTable[i].name))
      return servoTable[i].id;
  }
  return -1;
}

// ------------------------------------------------------------
// Convert angle to servo ticks (linear interpolation)
// ------------------------------------------------------------
int angle_to_ticks(int ticks_0, int ticks_90, int angle_deg) {
  // assume linear mapping; direction based on whether 90° > 0°
  float slope = (float)(ticks_90 - ticks_0) / 90.0f;
  return (int)(ticks_0 + slope * angle_deg);
}

// ------------------------------------------------------------
// Rotate base servo to a given angle (deg)
// ------------------------------------------------------------
void rotateBaseToAngle(int angle_deg) {
  int id = getServoId("base");
  if (id < 0) return;

  int base_0 = getParamValue("base_0");
  int base_90 = getParamValue("base_90");
  int base_min = getParamValue("base_min");
  int base_max = getParamValue("base_max");

  int target_ticks = angle_to_ticks(base_0, base_90, angle_deg);

  moveServoSmooth(id, target_ticks, base_min, base_max);
}

// ------------------------------------------------------------
// Smooth move with stall monitoring
// Used mainly for raw calibration and fine control
// ------------------------------------------------------------
void moveServoSmooth(int id, int target, int minv, int maxv) {
  // Clamp target to range
  target = constrain(target, minv, maxv);

  // Read current position
  int curPos = dxl.getPresentPosition(id);

  // Determine step direction
  int step = (target > curPos) ? 10 : -10;

  while ((step > 0 && curPos < target) || (step < 0 && curPos > target)) {
    curPos += step;

    // Prevent overshoot
    if ((step > 0 && curPos > target) || (step < 0 && curPos < target))
      curPos = target;

    dxl.setGoalPosition(id, curPos);

    // Stall detection
    int current_mA = dxl.getPresentCurrent(id);  // returns mA
    if (abs(current_mA) > STALL_THRESHOLD) {
      Serial.printf("⚠️ Stall detected on servo %d — current=%d mA\n", id, current_mA);
      dxl.ledOn(id);  // Optional: turn LED red
      delay(100);
      dxl.ledOff(id);
      break;
    }

    delay(20);
  }

  Serial.printf("✅ Servo %d moved to %d ticks\n", id, target);
}

// ------------------------------------------------------------
// Update servo by name and save to Preferences if changed
// ------------------------------------------------------------
void updateServo(const String &name, int target) {
  if (!isServo(name)) {
    Serial.println("❌ Invalid servo: " + name);
    return;
  }

  int id = getServoId(name);

  // Read min/max constraints
  String minKey = name + "_min";
  String maxKey = name + "_max";
  int minv = getParamValue(minKey);
  int maxv = getParamValue(maxKey);

  // Clamp target
  target = constrain(target, minv, maxv);

  // Move the servo
  moveServoSmooth(id, target, minv, maxv);

  // Update param store
  setParamValue(name, target);

  // Write to NVS (Preferences) only if changed
  int stored = preferences.getInt(name.c_str(), -99999);
  if (stored != target) {
    preferences.putInt(name.c_str(), target);
    Serial.printf("💾 Saved %s = %d\n", name.c_str(), target);
  }
}

