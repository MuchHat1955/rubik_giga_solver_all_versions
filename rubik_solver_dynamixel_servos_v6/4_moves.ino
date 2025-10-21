// ==========================================
// RUN GROUP POSES WITH RESETS + CUBE ORIENTATION + ACTION
// ==========================================
// Requires: matchServoState(), readServoTicks(), runAction().
// Integrates with: servoMap[], paramStore, and actionStore.

#include <map>
#include <vector>
#include <Arduino.h>

void runCubeMove(const String &moveName);

// -------------------------------
// 1️⃣ Cube Orientation Tracker
// -------------------------------
struct CubeOrientation {
  String up = "top";
  String down = "bottom";
  String front = "front";
  String back = "back";
  String left = "left";
  String right = "right";
};

CubeOrientation cube;

// -------------------------------
// 2️⃣ Logical Move Resolver
// -------------------------------
// Converts a symbolic cube move (like "top+") into its real-world equivalent
String resolveLogicalMove(const String &move) {
  String face = move.substring(0, move.length() - 1);  // e.g., "top"
  String dir = move.substring(move.length() - 1);      // e.g., "+"

  if (face == "top") face = cube.up;
  else if (face == "bottom") face = cube.down;
  else if (face == "front") face = cube.front;
  else if (face == "back") face = cube.back;
  else if (face == "left") face = cube.left;
  else if (face == "right") face = cube.right;

  return face + dir;
}

// -------------------------------
// 3️⃣ Orientation Update Table
// -------------------------------
void updateCubeOrientation(const String &seq) {
  // You can extend this table as needed
  if (seq == "top_to_base") {
    std::swap(cube.up, cube.down);
  } 
  else if (seq == "left_to_cradle") {
    std::swap(cube.front, cube.left);
    std::swap(cube.back, cube.right);
  } 
  else if (seq == "flip_180") {
    std::swap(cube.front, cube.back);
  }

  Serial.printf("[orientation] Updated after %s:\n  up=%s  down=%s  front=%s  back=%s  left=%s  right=%s\n",
                seq.c_str(), cube.up.c_str(), cube.down.c_str(),
                cube.front.c_str(), cube.back.c_str(),
                cube.left.c_str(), cube.right.c_str());
}

// -------------------------------
// 4️⃣ Reset Rule Definition
// -------------------------------
struct ResetRule {
  String resetAction;      // Action name, e.g. "reset_base"
  String servoToCheck;     // Servo key, e.g. "base"
  String presetKeyToMatch; // Expected preset, e.g. "base_0"
};

// -------------------------------
// 5️⃣ Reset Table per Sequence
// -------------------------------
std::map<String, std::vector<ResetRule>> sequenceResets = {
  {"top_to_cradle",   {{"reset_base",  "base",  "base_0"}}},
  {"front_to_cradle", {{"reset_wrist", "wrist", "wrist_90"}}},
  {"rotate_base_180", {{"reset_base",  "base",  "base_0"}}},
  {"lift_to_reset",   {{"reset_wrist", "wrist", "wrist_0"},
                       {"reset_base",  "base",  "base_0"}}}
};

// -------------------------------
// 6️⃣ Smart Reset Evaluation
// -------------------------------
// Uses matchServoState() to decide whether to execute a reset
std::vector<String> collectNeededResets(const String &seqName) {
  std::vector<String> needed;
  if (!sequenceResets.count(seqName)) return needed;

  for (const auto &rule : sequenceResets[seqName]) {
    bool needsReset = !matchServoState(rule.servoToCheck, rule.presetKeyToMatch);
    if (needsReset) {
      needed.push_back(rule.resetAction);
      Serial.printf("[reset] Will run %s (servo %s not at %s)\n",
                    rule.resetAction.c_str(),
                    rule.servoToCheck.c_str(),
                    rule.presetKeyToMatch.c_str());
    } else {
      Serial.printf("[reset] Skipping %s (servo %s already at %s)\n",
                    rule.resetAction.c_str(),
                    rule.servoToCheck.c_str(),
                    rule.presetKeyToMatch.c_str());
    }
  }
  return needed;
}

// -------------------------------
// 7️⃣ Run Multiple Resets in Sync
// -------------------------------
// Merges several reset actions into one "group pose" call
void runGroupResetsIfNeeded(const String &seqName) {
  auto needed = collectNeededResets(seqName);
  if (needed.empty()) return;

  Serial.printf("[reset] Running %d resets in sync before %s\n", needed.size(), seqName.c_str());

  // Build a combined ActionDef dynamically
  std::vector<String> combined;
  for (auto &r : needed) combined.push_back(r);

  // Execute them together
  runActionGroupPose(combined);
}

// -------------------------------
// 8️⃣ Unified Sequence Executor
// -------------------------------
// Runs resets (in sync if needed), then the sequence, then updates orientation
void runSequenceWithResets(const String &seqName) {
  runGroupResetsIfNeeded(seqName);
  Serial.printf("[seq] Running main sequence: %s\n", seqName.c_str());
  runAction(seqName);
  updateCubeOrientation(seqName);
}

// -------------------------------
// 9️⃣ Cube Move Executor
// -------------------------------
// Handles high-level cube-move logic like “f+”, resolving to current orientation
void runCubeMove(const String &moveName) {
  String resolved = resolveLogicalMove(moveName);
  Serial.printf("[cube] %s resolved to physical %s\n", moveName.c_str(), resolved.c_str());
  runSequenceWithResets(resolved);
}
