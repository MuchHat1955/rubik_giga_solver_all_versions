#include "sequence_store.h"
#include "logging.h"

extern PoseStore pose_store;

SequenceStore sequence_store;

// ---------------------------------------------------------------------------
// Available poses
// ---------------------------------------------------------------------------
/*
  // y poses
        y_zero | y_1st | y_2nd | y_3rd
        y_c2  | y_c3

  // x poses
        x_center | x_left | x_right

  //grippers
        grippers_open | grippers_close
        gripper1_open | gripper1_close
        gripper2_open gripper2_close",

  // wrist
        wrist_vert | wrist_horiz_left | wrist_horiz_right 

  // base
        base_front | base_right | base_left 
*/

// ---------------------------------------------------------------------------
// Constructor: define all sequences here
// ---------------------------------------------------------------------------

//TODO fix below with the actual moves
SequenceStore::SequenceStore() {
  // Example — replace pose names with actual ones from your PoseStore keys
  sequences = {
    { "bottom_plus", "bottom+", { "grips_open", "move_down", "grip1_close" } },
    { "bottom_minus", "bottom-", { "grip1_open", "move_up", "grip1_close" } },

    { "front_to_base", "front to base", { "front_align", "base_move", "release" } },
    { "back_to_base", "back to base", { "back_align", "base_move", "release" } },

    { "left_to_base", "left to base", { "left_align", "base_move", "release" } },
    { "right_to_base", "right to base", { "right_align", "base_move", "release" } },

    { "top_to_base", "top to base", { "top_align", "base_move", "release" } },

    { "rotate_down_90", "rotate down face+", { "down_grip", "rotate_90", "release" } },
    { "rotate_down_90minus", "rotate down face-", { "down_grip", "rotate_-90", "release" } },
  };
}

// ---------------------------------------------------------------------------
// Check if a key corresponds to a valid sequence
// ---------------------------------------------------------------------------
bool SequenceStore::is_key_for_sequence(const char* key) const {
  if (!key || !*key) return false;
  for (auto& seq : sequences)
    if (strcmp(seq.key, key) == 0)
      return true;
  return false;
}

// ---------------------------------------------------------------------------
// Run a sequence by name
// ---------------------------------------------------------------------------
bool SequenceStore::run_sequence_by_key(const char* key) {
  LOG_SECTION_START_PRINTF("runSequence", "| key{%s}", key);

  for (auto& seq : sequences) {
    if (strcmp(seq.key, key) != 0) continue;

    LOG_PRINTF("Running sequence: %s (%s)", seq.key, seq.text);
    for (auto pose : seq.poses) {
      LOG_PRINTF("  pose: %s", pose);
      if (!pose_store.run_pose(pose)) {
        LOG_ERROR_RB("run sequence {%s} pose failed {%s}", key, pose);
        LOG_SECTION_END();
        return false;
      }
      delay(100);  // small pause between poses
    }

    LOG_SECTION_END();
    return true;
  }

  LOG_ERROR_RB("Sequence not found {%s}", key);
  LOG_SECTION_END();
  return false;
}

// ---------------------------------------------------------------------------
// List all sequences (for menu/debug)
// ---------------------------------------------------------------------------
String SequenceStore::listAllSequences() const {
  String out;
  for (auto& s : sequences) {
    out += String(s.key) + " (" + s.text + ")\n";
  }
  return out;
}
