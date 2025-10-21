#include <Dynamixel2Arduino.h>
#include <Preferences.h>

extern Dynamixel2Arduino dxl; // assumed already initialized
Preferences preferences;

const int STALL_THRESHOLD = 600; // mA

// Servo name → ID mapping
struct ServoMap {
  const char* name;
  uint8_t id;
};

ServoMap servoTable[] = {
  {"arm1", 1},
  {"arm2", 2},
  {"wrist", 3},
  {"grip1", 4},
  {"grip2", 5},
  {"base", 6}
};

const int NUM_SERVOS = sizeof(servoTable) / sizeof(servoTable[0]);

// Check if name is a valid servo
bool isServo(const String &name) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (name.equalsIgnoreCase(servoTable[i].name)) return true;
  }
  return false;
}

// Get servo ID from name (or -1 if not found)
int getServoId(const String &name) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (name.equalsIgnoreCase(servoTable[i].name)) return servoTable[i].id;
  }
  return -1;
}

// Smooth move with stall monitoring
void moveServoSmooth(int id, int target, int minv, int maxv) {
  // Clamp target
  if (target < minv) target = minv;
  if (target > maxv) target = maxv;

  // Read current pos
  int curPos = dxl.getPresentPosition(id);

  // Step size (tune for speed/smoothness)
  int step = (target > curPos) ? 10 : -10;

  while ((step > 0 && curPos < target) || (step < 0 && curPos > target)) {
    curPos += step;
    if ((step > 0 && curPos > target) || (step < 0 && curPos < target)) {
      curPos = target;
    }

    dxl.setGoalPosition(id, curPos);

    // Stall detection (tune threshold)
    int torque = dxl.getPresentCurrent(id);
    if (abs(torque) > STALL_THRESHOLD) {
      Serial.println("Stall detected, stopping servo " + String(id));
      break;
    }

    delay(20);
  }
}

// Update servo: check valid, move, save to preferences
void updateServo(const String &name, int target) {
  if (!isServo(name)) {
    Serial.println("Invalid servo: " + name);
    return;
  }

  int id = getServoId(name);

  // min/max keys
  String minKey = name + "_min";
  String maxKey = name + "_max";

  int minv = getParamValue(minKey);
  int maxv = getParamValue(maxKey);

  // Clamp target to param limits
  if (target < minv) target = minv;
  if (target > maxv) target = maxv;

  // Move servo
  moveServoSmooth(id, target, minv, maxv);

  // Update paramStore
  setParamValue(name, target);

  // Save to Preferences (only if changed)
  int stored = preferences.getInt(name.c_str(), -99999);
  if (stored != target) {
    preferences.putInt(name.c_str(), target);
    Serial.println("Saved " + name + " = " + String(target));
  }
}
