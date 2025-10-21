// ==========================================
// GROUP POSE MOTION CONTROLLER
// ==========================================

#include <Dynamixel2Arduino.h>
#include <vector>
#include <map>

#define SERVO_SPEED_DEFAULT   60        // Base speed factor
#define SERVO_ACCEL_TIME_MS   250
#define SERVO_DECEL_TIME_MS   250
#define SYNC_INTERVAL_MS      20

#define STALL_THRESHOLD       150        // PWM or load threshold
#define STALL_CHECK_INTERVAL  50         // ms
#define STALL_DEBOUNCE_COUNT  3          // consecutive high load readings before declaring stall

#define LED_ON_MOVING         1
#define LED_OFF_IDLE          0

extern Dynamixel2Arduino dxl;

// Provided externally:
// struct ServoMap { const char* name; uint8_t id; };
// extern ServoMap servoTable[];
// int getParamValue(const String &key);

int getServoId(const String &name) {
  for (auto &entry : servoTable) {
    if (name.equalsIgnoreCase(entry.name)) return entry.id;
  }
  return -1;
}

// LED control (safe for any XL-430)
void setServoLED(int id, uint8_t r, uint8_t g, uint8_t b) {
  dxl.ledOn(id);
  dxl.setLEDColor(id, r, g, b);
}

// Read position in ticks
int readServoTicks(const String &name) {
  int id = getServoId(name);
  if (id < 0) return 0;
  return dxl.getPresentPosition(id);
}

// Read torque/load (XL430 → PWM or effort)
int readServoLoad(int id) {
  if (id < 0) return 0;
  return abs(dxl.getPresentPWM(id));   // use PWM since Load is deprecated
}

// Cosine ease-in-out profile
float easeInOut(float t) {
  return (1 - cos(PI * t)) * 0.5;
}

// Emergency stop (flashing all LEDs)
void flashAllServosRed(const std::vector<int> &ids) {
  for (int i = 0; i < 6; i++) {
    for (auto id : ids) setServoLED(id, 255, 0, 0);
    delay(100);
    for (auto id : ids) setServoLED(id, 0, 0, 0);
    delay(100);
  }
}

// --- Group Motion ---
bool runGroupPoseServos(const std::vector<String> &servos, const std::vector<int> &targets) {
  if (servos.empty() || servos.size() != targets.size()) return false;

  const int n = servos.size();
  std::vector<int> ids(n), startPos(n), stallCount(n, 0);
  bool stalled = false;

  for (int i = 0; i < n; i++) {
    ids[i] = getServoId(servos[i]);
    startPos[i] = dxl.getPresentPosition(ids[i]);
    setServoLED(ids[i], 255, 0, 0); // red (moving)
  }

  int maxDelta = 0;
  for (int i = 0; i < n; i++) {
    int d = abs(targets[i] - startPos[i]);
    if (d > maxDelta) maxDelta = d;
  }
  if (maxDelta == 0) return true;

  int totalTime = SERVO_ACCEL_TIME_MS + SERVO_DECEL_TIME_MS + maxDelta / SERVO_SPEED_DEFAULT;
  unsigned long t0 = millis(), lastStallCheck = 0;

  while (true) {
    float elapsed = millis() - t0;
    if (elapsed > totalTime) break;
    float phase = elapsed / (float)totalTime;
    if (phase > 1.0) phase = 1.0;
    float eased = easeInOut(phase);

    // --- Check for stall every interval ---
    if (millis() - lastStallCheck > STALL_CHECK_INTERVAL) {
      lastStallCheck = millis();
      for (int i = 0; i < n; i++) {
        int load = readServoLoad(ids[i]);
        if (load > STALL_THRESHOLD) stallCount[i]++; else stallCount[i] = 0;
        if (stallCount[i] >= STALL_DEBOUNCE_COUNT) stalled = true;
      }
    }

    if (stalled) {
      Serial.println("[STALL] Motion aborted!");
      flashAllServosRed(ids);
      return false;
    }

    // --- Send intermediate positions ---
    for (int i = 0; i < n; i++) {
      int pos = startPos[i] + (int)((targets[i] - startPos[i]) * eased);
      dxl.setGoalPosition(ids[i], pos);
    }

    delay(SYNC_INTERVAL_MS);
  }

  // Finalize
  for (int i = 0; i < n; i++) {
    dxl.setGoalPosition(ids[i], targets[i]);
    setServoLED(ids[i], 0, 255, 0); // green (done)
  }

  return true;
}

// --- Group Pose Executor ---
void runActionGroupPose(const std::vector<String> &actionKeys) {
  std::vector<String> servos;
  std::vector<int> targets;

  for (const auto &key : actionKeys) {
    int underscore = key.indexOf('_');
    if (underscore < 0) continue;
    String servo = key.substring(0, underscore);
    int val = getParamValue(key);
    servos.push_back(servo);
    targets.push_back(val);
    Serial.printf("[group] %s -> %d\n", servo.c_str(), val);
  }

  bool ok = runGroupPoseServos(servos, targets);
  if (!ok) Serial.println("[group_pose] Stalled!");
}


