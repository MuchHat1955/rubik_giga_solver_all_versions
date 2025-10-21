// --- PRESET HANDLING FOR SERVOS ---

// Extract base servo name (substring before "_")
String getServoNameFromKey(const String &presetKey) {
  int idx = presetKey.indexOf('_');
  if (idx == -1) return presetKey;  // no "_", just return full name
  return presetKey.substring(0, idx);
}

// Move servo to preset position
void moveServoToPreset(const String &presetKey) {
  if (paramStore.find(presetKey) == paramStore.end()) {
    Serial.println("Preset key not found: " + presetKey);
    return;
  }

  // get servo name ("arm1" from "arm1_0")
  String servoName = getServoNameFromKey(presetKey);

  if (!isServo(servoName)) {
    Serial.println("Not a valid servo for preset: " + servoName);
    return;
  }

  int target = getParamValue(presetKey);

  // Clamp with the main servo's min/max (not the preset’s)
  String minKey = servoName + "_min";
  String maxKey = servoName + "_max";
  int minv = getParamValue(minKey);
  int maxv = getParamValue(maxKey);

  // Do the move
  int id = getServoId(servoName);
  moveServoSmooth(id, target, minv, maxv);

  // Update current value for the servo
  setParamValue(servoName, target);

  // Save updated position
  int stored = preferences.getInt(servoName.c_str(), -99999);
  if (stored != target) {
    preferences.putInt(servoName.c_str(), target);
    Serial.println("Preset applied & saved: " + servoName + " -> " + String(target));
  }
}

// Run multiple presets in sequence (list of keys)
void runPresetSequence(const std::vector<String> &presetKeys) {
  for (const auto &key : presetKeys) {
    moveServoToPreset(key);
    delay(200); // small delay between moves
  }
}
