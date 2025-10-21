// --- ACTION STORE ---

enum ActionType {
  ACTION_PRESET,
  ACTION_SERVO_MOVE,
  ACTION_CUBE_MOVE
};

class ActionDef {
public:
  ActionType type;
  String key;                    // For PRESET or SERVO MOVE
  std::vector<String> sequence;  // For KEY MOVE sequences

  ActionDef() {}
  ActionDef(ActionType t, const String &k)
    : type(t), key(k) {}
  ActionDef(ActionType t, const std::vector<String> &s)
    : type(t), sequence(s) {}
};

std::map<String, ActionDef> actionStore;

// --- Initialize Actions ---
void initActions() {
  // Preset actions
  actionStore["arm1_0"] = ActionDef(ACTION_PRESET, "arm1_0");
  actionStore["arm1_1"] = ActionDef(ACTION_PRESET, "arm1_1");
  actionStore["arm1_2"] = ActionDef(ACTION_PRESET, "arm1_2");
  actionStore["arm2_0"] = ActionDef(ACTION_PRESET, "arm2_0");
  actionStore["arm2_1"] = ActionDef(ACTION_PRESET, "arm2_1");
  actionStore["arm2_2"] = ActionDef(ACTION_PRESET, "arm2_2");
  actionStore["wrist_0"] = ActionDef(ACTION_PRESET, "wrist_0");
  actionStore["wrist_90"] = ActionDef(ACTION_PRESET, "wrist_90");
  actionStore["wrist_180"] = ActionDef(ACTION_PRESET, "wrist_180");
  actionStore["wrist_minus90"] = ActionDef(ACTION_PRESET, "wrist_minus90");
  actionStore["grip_0"] = ActionDef(ACTION_PRESET, "grip_0");
  actionStore["grip_1"] = ActionDef(ACTION_PRESET, "grip_1");
  actionStore["base_0"] = ActionDef(ACTION_PRESET, "base_0");
  actionStore["base_90"] = ActionDef(ACTION_PRESET, "base_90");
  actionStore["base_180"] = ActionDef(ACTION_PRESET, "base_180");
  actionStore["base_minus90"] = ActionDef(ACTION_PRESET, "base_minus90");

  // Servo move actions
  actionStore["arms_low"] = ActionDef(ACTION_SERVO_MOVE, "arms_low");
  actionStore["arms_mid"] = ActionDef(ACTION_SERVO_MOVE, "arms_mid");
  actionStore["arms_top"] = ActionDef(ACTION_SERVO_MOVE, "arms_top");
  actionStore["grip_open"] = ActionDef(ACTION_SERVO_MOVE, "grip_open");
  actionStore["grip_closed"] = ActionDef(ACTION_SERVO_MOVE, "grip_closed");

  // Cube move actions (KEY_MOVE) as sequences of servo moves
  actionStore["bottom_plus"] = ActionDef(ACTION_CUBE_MOVE, { "grip_open", "arms_mid", "grip_close", "base_90", "grip_open", "base_0", "grip_open" });
  actionStore["bottom_minus"] = ActionDef(ACTION_CUBE_MOVE, { "grip_open", "arms_mid", "grip_close", "base_minus90", "grip_open", "base_0", "grip_open" });
  actionStore["bottom_180"] = ActionDef(ACTION_CUBE_MOVE, { "grip_open", "arms_mid", "grip_close", "base_180", "grip_open", "base_0", "grip_open" });
  actionStore["z_plus"] = ActionDef(ACTION_CUBE_MOVE, { "grip_open", "base_90", "grip_close", "arms_high", "base_0", "arms_mid", "grip_close" });
  actionStore["z_minus"] = ActionDef(ACTION_CUBE_MOVE, { "grip_open", "base_minus90", "grip_close", "arms_high", "base_0", "arms_mid", "grip_close" });
  actionStore["z_180"] = ActionDef(ACTION_CUBE_MOVE, { "grip_open", "base_180", "grip_close", "arms_high", "base_0", "arms_mid", "grip_close" });
  actionStore["y_plus"] = ActionDef(ACTION_CUBE_MOVE, { "grip_open", "wrist_0", "arms_mid", "grip_close", "arms_high", "wrist_90", "arms_mid", "grip_open", "wrist_0" });
  actionStore["y_minus"] = ActionDef(ACTION_CUBE_MOVE, { "grip_open", "wrist_0", "arms_mid", "grip_close", "arms_high", "wrist_minus90", "arms_mid", "grip_open", "wrist_0" });
  actionStore["y_180"] = ActionDef(ACTION_CUBE_MOVE, { "grip_open", "wrist_0", "arms_mid", "grip_close", "arms_high", "wrist_180", "arms_mid", "grip_open", "wrist_0" });
  actionStore["read_cube"] = ActionDef(ACTION_CUBE_MOVE, { "grip_open", "arms_read", "base_minus90", "base_0", "base_90", "base_180" });
}

// Convert Dynamixel present position into ticks (0–4095)
// key = "arm1", "arm2", "wrist", "grip1", "grip2", "base"
int readServoTicks(const String &key) {
  if (!isServo(key)) return 0;

  int id = getServoId(key);
  // Dynamixel returns position in 0–4095 ticks
  int pos = dxl.getPresentPosition(id, UNIT_RAW);
  if (pos < 0) pos = 0;  // safety
  if (pos > 4095) pos = 4095;

  return pos;
}

// --- Execute an Action ---
void runAction(const String &actionKey) {
  if (actionStore.find(actionKey) == actionStore.end()) {
    Serial.println("Unknown action: " + actionKey);
    return;
  }
  ActionDef &a = actionStore[actionKey];
  switch (a.type) {
    case ACTION_PRESET:
      Serial.println("Running PRESET action: " + a.key);
      moveServoToPreset(a.key);
      break;

    case ACTION_SERVO_MOVE:
      Serial.println("Running SERVO MOVE action: " + a.key);
      updateServo(a.key, getParamValue(a.key));  // placeholder
      break;

    case ACTION_CUBE_MOVE:
      Serial.println("Running KEY MOVE action: " + actionKey);
      for (auto &step : a.sequence) {
        Serial.println(" -> Step: " + step);
        runAction(step);  // recursion: run servo moves inside sequence
      }
      break;
  }
}

// --- Utility: degree <-> ticks ---
inline int degToTicks(float deg) {
  return (int)(deg * 4096.0 / 360.0);  // XL430 full resolution
}
inline float ticksToDeg(int ticks) {
  return (ticks * 360.0 / 4096.0);
}

// --- Compare with tolerance ---
bool withinTolerance(int current, int target, int toleranceTicks) {
  return abs(current - target) <= toleranceTicks;
}

// Compare a servo's actual position with a preset key value
bool matchServoPreset(const String &servoKey, const String &presetKey, int tolerance = 35) {
  if (!isServo(servoKey)) return false;
  if (paramStore.find(presetKey) == paramStore.end()) return false;

  int actualTicks = readServoTicks(servoKey);
  int targetTicks = getParamValue(presetKey);

  return abs(actualTicks - targetTicks) <= tolerance;
}

// --- Check if servos match a preset or servo move ---
String matchServoAction() {
  const int tolTicks = degToTicks(3.0);  // ±3 degrees

  // Iterate through PRESET and SERVO_MOVE actions
  for (auto &kv : actionStore) {
    const String &name = kv.first;
    ActionDef &a = actionStore[name];

    if (a.type == ACTION_PRESET || a.type == ACTION_SERVO_MOVE) {
      bool match = true;

      // A preset key is e.g. "arm1_0", extract servo name
      if (a.key.length() == 0) continue;

      String baseKey = a.key;
      // If it's a PRESET: lookup param value
      if (a.type == ACTION_PRESET) {
        if (paramStore.find(baseKey) == paramStore.end()) continue;
        int targetTicks = paramStore[baseKey].value;
        int currentTicks = readServoTicks(baseKey);  // <-- your Dynamixel read
        if (!withinTolerance(currentTicks, targetTicks, tolTicks)) {
          match = false;
        }
      }

      // If it's a SERVO_MOVE: you can define them as sets of servo preset combos
      if (a.type == ACTION_SERVO_MOVE) {
        // Example: "arms_low" → means arm1_0 + arm2_0
        if (name == "arms_low") {
          int t1 = paramStore["arm1_0"].value;
          int t2 = paramStore["arm2_0"].value;
          if (!withinTolerance(readServoTicks("arm1"), t1, tolTicks)) match = false;
          if (!withinTolerance(readServoTicks("arm2"), t2, tolTicks)) match = false;
        } else if (name == "arms_mid") {
          int t1 = paramStore["arm1_1"].value;
          int t2 = paramStore["arm2_1"].value;
          if (!withinTolerance(readServoTicks("arm1"), t1, tolTicks)) match = false;
          if (!withinTolerance(readServoTicks("arm2"), t2, tolTicks)) match = false;
        } else if (name == "arms_top") {
          int t1 = paramStore["arm1_2"].value;
          int t2 = paramStore["arm2_2"].value;
          if (!withinTolerance(readServoTicks("arm1"), t1, tolTicks)) match = false;
          if (!withinTolerance(readServoTicks("arm2"), t2, tolTicks)) match = false;
        } else if (name == "arms_read") {
          int t1 = paramStore["arm1_read"].value;
          int t2 = paramStore["arm2_read"].value;
          if (!withinTolerance(readServoTicks("arm1"), t1, tolTicks)) match = false;
          if (!withinTolerance(readServoTicks("arm2"), t2, tolTicks)) match = false;
        } else if (name == "grip_open") {
          int t = paramStore["grip_1"].value;
          if (!withinTolerance(readServoTicks("grip"), t, tolTicks)) match = false;
        } else if (name == "grip_closed") {
          int t = paramStore["grip_0"].value;
          if (!withinTolerance(readServoTicks("grip"), t, tolTicks)) match = false;
        }
      }

      if (match) {
        return name;  // Found matching state
      }
    }
  }

  return "none";  // No match found
}

// Helper: ensure grip is open before continuing
void ensureGripOpen() {
  int gripOpen = paramStore["grip_1"].value;  // "open" preset
  if (!withinTolerance(readServoTicks("grip"), gripOpen, degToTicks(3))) {
    updateServo("grip", gripOpen);
  }
}

void safeServoMove(const String &servo, int targetTicks) {
  // --- Constraint 1: Wrist rotation ---
  if (servo == "wrist") {
    if (matchServoPreset("arm1", "arm1_0") ||     // arm1 LOW
        matchServoPreset("arm1", "arm1_1") ||     // arm1 MID
        matchServoPreset("arm1", "arm1_read") ||  // arm1 READ
        matchServoPreset("arm2", "arm2_0") ||     // arm2 LOW
        matchServoPreset("arm2", "arm2_1") ||     // arm2 MID
        matchServoPreset("arm2", "arm2_read")) {  // arm2 READ
      ensureGripOpen();
    }
  }

  // --- Constraint 2: Arm low<->mid transitions ---
  if (servo == "arm1" || servo == "arm2") {
    String lowKey = servo + "_0";
    String midKey = servo + "_1";
    String readKey = servo + "_1";

    if ((matchServoPreset(servo, lowKey) && targetTicks == getParamValue(lowKey)) ||    //
        (matchServoPreset(servo, midKey) && targetTicks == getParamValue(midKey)) ||  //
        (matchServoPreset(servo, readKey) && targetTicks == getParamValue(readKey))) {
      ensureGripOpen();
    }
  }

  // --- Constraint 3: Base rotation ---
  if (servo == "base") {
    if (matchServoPreset("arm1", "arm1_0") ||     // arm1 LOW
        matchServoPreset("arm1", "arm1_21") ||    // arm1 MID
        matchServoPreset("arm1", "arm1_read") ||  // arm1 READ
        matchServoPreset("arm2", "arm2_0") ||     // arm2 LOW
        matchServoPreset("arm2", "arm2_1") ||     // arm2 MID
        matchServoPreset("arm2", "arm2_read")) {  // arm2 HIGH
      ensureGripOpen();
    }
  }

  // --- Finally do the move safely ---

  //TODO add here the case of a servomove like arm_low and 2 servos need to be moved at the same time
  // there will be a move 2 servos etc
  updateServo(servo, targetTicks);
}

// Executes a single action safely (preset, servo move, cube move)
void safeActionExecute(const String &actionKey) {
  if (actionStore.find(actionKey) == actionStore.end()) {
    Serial.println("Unknown action: " + actionKey);
    return;
  }

  ActionDef &act = actionStore[actionKey];

  switch (act.type) {
    case ACTION_PRESET:
      {
        // Example: arm1_0
        String servo = getServoNameFromKey(actionKey);  // e.g. "arm1"
        int target = getParamValue(actionKey);          // stored param
        safeServoMove(servo, target);
        break;
      }

    case ACTION_SERVO_MOVE:
      {
        // Example: wrist_90
        String servo = getServoNameFromKey(actionKey);
        int target = getParamValue(actionKey);
        safeServoMove(servo, target);
        break;
      }

    case ACTION_CUBE_MOVE:
      {
        // Example: front_plus is a sequence of servo moves
        for (const auto &stepKey : act.sequence) {
          if (actionStore.find(stepKey) != actionStore.end()) {
            safeActionExecute(stepKey);  // recurse for each servo move
          }
        }
        break;
      }

    default:
      Serial.println("Unknown action type for " + actionKey);
      break;
  }
}
