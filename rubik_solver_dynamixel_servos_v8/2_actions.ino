// =====================================================
// ACTION STORE SYSTEM
// =====================================================

/*
main functions used outside of this file:
void actionExecute(const String &key);
void runSingleServo(const String &servoName, long target);
*/

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

  ActionDef(ActionType t, const std::vector<String> &k, int d = 0)
    : type(t), keys(k), delay_ms(d), custom(nullptr) {}
};

std::map<String, ActionDef> actionStore;

// =====================================================
// Helpers
// =====================================================

String getServoBaseName(const String &poseKey) {
  int idx = poseKey.indexOf('_');
  if (idx == -1) return poseKey;  // no underscore
  return poseKey.substring(0, idx);
}

// =====================================================
// Execute any action recursively
// =====================================================

void actionExecute(const String &key) {
  if (actionStore.find(key) == actionStore.end()) {
    Serial.printf("❌ Unknown action: %s\n", key.c_str());
    return;
  }

  ActionDef &act = actionStore[key];

  switch (act.type) {
    case ACTION_POSE: {
      // Single servo pose (e.g., "arm1_0")
      String servoName = getServoBaseName(act.keys[0]);
      int target = getParamValue(act.keys[0]);
      runGroupPoseServos({ servoName }, { target });
      break;
    }

    case ACTION_GROUP: {
      // Multiple servos at once (e.g., "arms_0" → {"arm1_0", "arm2_0"})
      std::vector<String> servos;
      std::vector<int> targets;

      for (const auto &poseKey : act.keys) {
        String servoName = getServoBaseName(poseKey);
        int target = getParamValue(poseKey);
        servos.push_back(servoName);
        targets.push_back(target);
      }

      // Skip redundant moves
      if (key == "base_reset" && isPoseActive("base_0")) return;
      if (key == "wrist_reset" && isPoseActive("wrist_0")) return;
      if (isGroupPoseActive(key)) return;

      // Safety: simplify base reset if cube is lifted
      if (key == "base_reset" && isPoseActive("arms_1")) {
        actionExecute("base_0");
        return;
      }

      runGroupPoseServos(servos, targets);
      break;
    }

    case ACTION_SEQUENCE: {
      // Sequence of other actions
      for (const auto &stepKey : act.keys) {
        actionExecute(stepKey);
        if (act.delay_ms > 0) delay(act.delay_ms);
      }
      // Optional cube orientation tracking
      if (key.length() > 0)
        cubeOri.rotateCube(key);
      break;
    }

    default:
      Serial.printf("⚠️ Unknown action type for %s\n", key.c_str());
      break;
  }
}

// =====================================================
// Initialize all actions
// =====================================================

void initActionStore() {
  actionStore.clear();

  // --- Individual servo poses ---
  String servos[] = { "arm1", "arm2", "wrist", "grip1", "grip2", "base" };
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
  actionStore["arms_0"]     = ActionDef(ACTION_GROUP, { "arm1_0", "arm2_0" });
  actionStore["arms_row1"]  = ActionDef(ACTION_GROUP, { "arm1_1", "arm2_1" });
  actionStore["arms_row2"]  = ActionDef(ACTION_GROUP, { "arm1_2", "arm2_2" });
  actionStore["arms_read1"] = ActionDef(ACTION_GROUP, { "arm1_read1", "arm2_read1" });
  actionStore["arms_read2"] = ActionDef(ACTION_GROUP, { "arm1_read2", "arm2_read2" });

  actionStore["grip_open"]   = ActionDef(ACTION_GROUP, { "grip1_open", "grip2_open" });
  actionStore["grip_closed"] = ActionDef(ACTION_GROUP, { "grip1_closed", "grip2_closed" });

  // --- Sequences ---
  actionStore["base_reset"]   = ActionDef(ACTION_SEQUENCE, { "base_0" }, 400);
  actionStore["wrist_reset"]  = ActionDef(ACTION_SEQUENCE, { "wrist_0" }, 400);

  actionStore["front_to_base"] = ActionDef(ACTION_SEQUENCE, { "wrist_reset", "arms_0", "wrist_180" }, 400);
  actionStore["back_to_base"]  = ActionDef(ACTION_SEQUENCE, { "wrist_reset", "arms_0", "wrist_180" }, 400);
  actionStore["top_to_base"]   = ActionDef(ACTION_SEQUENCE, { "wrist_reset", "arms_0", "wrist_180" }, 400);
  actionStore["left_to_base"]  = ActionDef(ACTION_SEQUENCE, { "wrist_reset", "arms_0", "wrist_180" }, 400);
  actionStore["right_to_base"] = ActionDef(ACTION_SEQUENCE, { "wrist_reset", "arms_0", "wrist_180" }, 400);

  actionStore["read_top"]    = ActionDef(ACTION_SEQUENCE, { "arms_read1", "base_90" }, 400);
  actionStore["read_middle"] = ActionDef(ACTION_SEQUENCE, { "arms_read2", "base_90" }, 400);

  // --- Cube moves (placeholders for now) ---
  const std::vector<String> basicMove = { "arms_row1", "wrist_90", "grip_closed" };
  const std::vector<String> reverseMove = { "grip_closed", "base_90", "grip_open" };

  for (auto f : {"f", "b", "l", "r", "u", "d"}) {
    String plus = String(f) + "+";
    String minus = String(f) + "-";
    String twice = String(f) + "++";
    actionStore[plus]  = ActionDef(ACTION_SEQUENCE, basicMove, 400);
    actionStore[minus] = ActionDef(ACTION_SEQUENCE, reverseMove, 500);
    actionStore[twice] = ActionDef(ACTION_SEQUENCE, basicMove, 400);
  }

  Serial.println("[ActionStore] Initialized actions:");
  for (auto &kv : actionStore) {
    Serial.printf("  - %s (type=%d, %d keys)\n", kv.first.c_str(), kv.second.type, (int)kv.second.keys.size());
  }
}

/*
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
  actionStore["read_top"] = ActionDef(ACTION_SEQUENCE, { "arms_read1", "base_90" }, 400);     // get the arms in read top, and rotate the base
  actionStore["read_middle"] = ActionDef(ACTION_SEQUENCE, { "arms_read2", "base_90" }, 400);  // get the arms in read top, and rotate the base
  actionStore["base_reset"] = ActionDef(ACTION_SEQUENCE, { "base_0" }, 400);
  actionStore["wrist_reset"] = ActionDef(ACTION_SEQUENCE, { "base_0" }, 400);

  //TODO all below are place holders
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
  actionStore["d++"] = ActionDef(ACTION_SEQUENCE, { "grip_closed", "base_90", "grip_open" }, 500);

  Serial.println("[ActionStore] Initialized actions:");
  for (auto &kv : actionStore) {
    Serial.printf("  - %s (type=%d, %d keys)\n", kv.first.c_str(), kv.second.type, kv.second.keys.size());
  }
}
*/
