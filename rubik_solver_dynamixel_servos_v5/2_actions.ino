
void actionExecute(const String &key);
void runSingleServo(const String &servoName, long target);

// =====================================================
// ACTION STORE SYSTEM
// =====================================================

enum ActionType {
  ACTION_POSE,
  ACTION_GROUP,
  ACTION_SEQUENCE,
  ACTION_UNKNOWN
};

class ActionDef {
public:
  ActionType type;
  std::vector<String> keys;  // list of servo/pose/sequence keys
  int delay_ms;
  void (*custom)();  // optional direct function

  ActionDef()
    : type(ACTION_UNKNOWN), delay_ms(0), custom(nullptr) {}

  ActionDef(ActionType t, std::initializer_list<String> k, int d = 0)
    : type(t), keys(k), delay_ms(d), custom(nullptr) {}

  ActionDef(ActionType t, std::vector<String> k, int d = 0)
    : type(t), keys(k), delay_ms(d), custom(nullptr) {}
};

std::map<String, ActionDef> actionStore;

// ------------------------------------------
// Execute any action recursively
// ------------------------------------------

String getServoBaseName(const String &poseKey) {
  int idx = poseKey.indexOf('_');
  if (idx == -1) return poseKey;  // no underscore
  return poseKey.substring(0, idx);
}

void actionExecute(const String &key) {
  if (actionStore.find(key) == actionStore.end()) {
    Serial.printf("Unknown action: %s\n", key.c_str());
    return;
  }

  ActionDef &act = actionStore[key];

  switch (act.type) {
    case ACTION_POSE:
      {
        // Single servo pose (e.g., "arm1_0")
        String servoName = getServoBaseName(act.key);
        int target = getParamValue(act.key);
        runGroupPoseServos({ servoName }, { target });
        break;
      }

    case ACTION_GROUP:
      {
        // Multiple servos at once (e.g., "arms_0" → {"arm1_0", "arm2_0"})
        std::vector<String> servos;
        std::vector<int> targets;

        for (const auto &poseKey : act.keys) {
          String servoName = getServoBaseName(poseKey);
          int target = getParamValue(poseKey);

          servos.push_back(servoName);
          targets.push_back(target);
        }
        // THIS CHECKS IF NOT ALREADY IN THE END POSITION
        if (key == "base_reset" && isPoseActive("base_0")) return;
        if (key == "wrist_reset" && isPoseActive("wrist_0")) return;
        if (isGroupPoseActive(key)) return;

        // SIMPLIFIED BASE_RESET IF CUBE IS OFF THE BASE
        if (key == "base_reset") {  // base reset needs cube lifted if arms not at 2
          if (isPoseActive("arms_1")) {
            actionExecute("base_0");
            return;
          }
        }
        runGroupPoseServos(servos, targets);
        break;
      }

    case ACTION_SEQUENCE:
      {
        // Sequence of other actions
        for (const auto &stepKey : act.keys) {
          actionExecute(stepKey);
        }
        break;
      }

    case ACTION_SEQUENCE:
      {
        // Sequence of other actions
        for (const auto &stepKey : act.keys) {
          actionExecute(stepKey);
        }
        break;
      }

    default:
      Serial.printf("⚠️ Unknown action type for %s\n", key.c_str());
      break;
  }
}

void runSingleServo(const String &servoName, long target) {
  // Wraps one servo into vectors and delegates to the group version
  std::vector<String> servos = { servoName };
  std::vector<int> targets = { static_cast<int>(target) };

  runGroupPoseServos(servos, targets);
}

// ------------------------------------------
// Initialize all actions
// ------------------------------------------
void initActionStore() {
  actionStore.clear();

  // --- Individual servo poses ---
  String servos[] = { "arm1", "arm2", "wrist", "grip", "base" };
  String suffixes[] = { "_0", "_1", "_2", "_90", "_180", "_minus90", "_read", "_open", "_closed" };

  for (const auto &s : servos) {
    for (const auto &suf : suffixes) {
      String k = s + suf;
      if (paramStore.find(k) != paramStore.end()) {
        actionStore[k] = ActionDef(ACTION_POSE, { k });
      }
    }
  }

  // --- Group poses ---
  actionStore["arms_0"] = ActionDef(ACTION_GROUP, { "arm1_0", "arm2_0" });
  actionStore["arms_row1"] = ActionDef(ACTION_GROUP, { "arm1_1", "arm2_1" });
  actionStore["arms_row2"] = ActionDef(ACTION_GROUP, { "arm1_2", "arm2_2" });
  actionStore["arms_read1"] = ActionDef(ACTION_GROUP, { "arm1_read1", "arm2_read1" });
  actionStore["arms_read2"] = ActionDef(ACTION_GROUP, { "arm1_read2", "arm2_read2" });
  actionStore["grip_open"] = ActionDef(ACTION_GROUP, { "grip1", "grip2" });
  actionStore["grip_closed"] = ActionDef(ACTION_GROUP, { "grip1", "grip2" });

  // --- Sequences ---
  // TODO add base_reset each time cube is off the case, arms at row2, to optimize for later
  actionStore["bottom_plus"] = ActionDef(ACTION_SEQUENCE, { "base_reset", "grip_closed", "base_90", "grip_open" }, 500);
  actionStore["bottom_minus"] = ActionDef(ACTION_SEQUENCE, { "base_reset", "grip_closed", "base_90", "grip_open" }, 500);
  actionStore["front_to_base"] = ActionDef(ACTION_SEQUENCE, { "wrist_reset", "arms_0", "wrist_180" }, 400);
  actionStore["back_to_base"] = ActionDef(ACTION_SEQUENCE, { "wrist_reset", "arms_0", "wrist_180" }, 400);
  actionStore["top_to_base"] = ActionDef(ACTION_SEQUENCE, { "wrist_reset", "arms_0", "wrist_180" }, 400);
  actionStore["left_to_base"] = ActionDef(ACTION_SEQUENCE, { "wrist_reset", "arms_0", "wrist_180" }, 400);
  actionStore["right_to_base"] = ActionDef(ACTION_SEQUENCE, { "wrist_reset", "arms_0", "wrist_180" }, 400);
  actionStore["read_top"] = ActionDef(ACTION_SEQUENCE, { "arms_row1", "base_90" }, 400);  // get the arms in read top, and rotate the base
  actionStore["read_level2"] = ActionDef(ACTION_SEQUENCE, { "base_0" }, 400);
  actionStore["read_level3"] = ActionDef(ACTION_SEQUENCE, { "base_0" }, 400);
  actionStore["base_reset"] = ActionDef(ACTION_SEQUENCE, { "base_0" }, 400);
  actionStore["wrist_reset"] = ActionDef(ACTION_SEQUENCE, { "base_0" }, 400);

  // --- Cube moves ---
  actionStore["f+"] = ActionDef(ACTION_SEQUENCE, { "arms_row1", "wrist_90", "grip_closed" }, 400);
  actionStore["f-"] = ActionDef(ACTION_SEQUENCE, { "grip_closed", "base_90", "grip_open" }, 500);
  actionStore["b+"] = ActionDef(ACTION_SEQUENCE, { "arms_row1", "wrist_90", "grip_closed" }, 400);
  actionStore["b-"] = ActionDef(ACTION_SEQUENCE, { "grip_closed", "base_90", "grip_open" }, 500);

  actionStore["l+"] = ActionDef(ACTION_SEQUENCE, { "arms_row1", "wrist_90", "grip_closed" }, 400);
  actionStore["l-"] = ActionDef(ACTION_SEQUENCE, { "grip_closed", "base_90", "grip_open" }, 500);
  actionStore["r+"] = ActionDef(ACTION_SEQUENCE, { "arms_row1", "wrist_90", "grip_closed" }, 400);
  actionStore["r-"] = ActionDef(ACTION_SEQUENCE, { "grip_closed", "base_90", "grip_open" }, 500);


  actionStore["u+"] = ActionDef(ACTION_SEQUENCE, { "arms_row1", "wrist_90", "grip_closed" }, 400);
  actionStore["u-"] = ActionDef(ACTION_SEQUENCE, { "grip_closed", "base_90", "grip_open" }, 500);
  actionStore["d+"] = ActionDef(ACTION_SEQUENCE, { "arms_row1", "wrist_90", "grip_closed" }, 400);
  actionStore["d-"] = ctionDef(ACTION_SEQUENCE, { "grip_closed", "base_90", "grip_open" }, 500);

  actionStore["f++"] = ActionDef(ACTION_SEQUENCE, { "arms_row1", "wrist_90", "grip_closed" }, 400);
  actionStore["b++"] = ActionDef(ACTION_SEQUENCE, { "grip_closed", "base_90", "grip_open" }, 500);
  actionStore["l++"] = ActionDef(ACTION_SEQUENCE, { "arms_row1", "wrist_90", "grip_closed" }, 400);
  actionStore["r++"] = ctionDef(ACTION_SEQUENCE, { "grip_closed", "base_90", "grip_open" }, 500);
  actionStore["u++"] = ActionDef(ACTION_SEQUENCE, { "arms_row1", "wrist_90", "grip_closed" }, 400);
  actionStore["d++"] = ctionDef(ACTION_SEQUENCE, { "grip_closed", "base_90", "grip_open" }, 500);

  Serial.println("[ActionStore] Initialized actions:");
  for (auto &kv : actionStore) {
    Serial.printf("  - %s (type=%d, %d keys)\n", kv.first.c_str(), kv.second.type, kv.second.keys.size());
  }
}
