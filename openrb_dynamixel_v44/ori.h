#pragma once
#include <Arduino.h>

// ============================================================
// Callback type for performing a physical robot move.
// Returns false if hardware move fails.
// ============================================================
typedef bool (*robot_move_cb_t)(const String &move_str);

// ============================================================
// CubeOri - handles cube orientation and maps cube moves to
// robot moves using lookup tables.
// ============================================================
class CubeOri {
public:
  CubeOri(robot_move_cb_t cb = nullptr);

  // Reset orientation to identity (U,R,F,D,L,B) and clear log
  void reset();

  // Set hardware callback
  void set_robot_callback(robot_move_cb_t cb) {
    robot_cb_ = cb;
  }

  // Execute a single robot move (y+, y', z+, z', z2, d+, d', d2).
  // - Calls the callback
  // - Updates orientation using k_ori_move_table
  // - Appends to move log
  bool robot_move(const String &move_str);

  // Execute a sequence of cube logical moves, e.g. "F R' U2".
  // Uses k_cube_to_robot_table and current orientation.
  bool cube_move(const String &moves_str);

  // Return mapping of original logical faces to current physical faces.
  // out[0] = "U->X"
  // out[1] = "R->X"
  // out[2] = "F->X"
  // out[3] = "D->X"
  // out[4] = "L->X"
  // out[5] = "B->X"
  void get_face_mapping(String out[6]) const;

  // Convenience: one-line orientation string:
  // "U->F R->R F->D D->B L->U B->L"
  String get_orientation_string() const;

  // Log of all robot moves applied (space separated)
  String get_move_log() const {
    return orientation_log_;
  }
  void clear_move_log() {
    orientation_log_ = "";
  }

private:
  // Orientation: for each *physical* face, which *logical* face is there.
  // At identity:
  //   U='U', R='R', F='F', D='D', L='L', B='B'
  struct Orientation {
    char U;
    char R;
    char F;
    char D;
    char L;
    char B;
  };

  Orientation ori_;
  robot_move_cb_t robot_cb_;
  String orientation_log_;

  // ----------------- Helpers -----------------

  // Normalize robot move to canonical form: "y+", "y'", "z2", "d+"
  // Returns "" on error.
  String normalize_robot_move_token_(const String &in);

  // Split "a b c" or "a,b,c" style lists into tokens
  void split_moves_(const String &in, String *out, int &count, int max_count) const;

  // Apply orientation change from the table for a given robot move
  void apply_ori_table_(const String &robot_move);

  // Execute one cube move given the *logical* face and quarter turns:
  // qt = +1, -1, or 2
  bool execute_single_cube_move_(char logical_face, int qt);

  // Parse cube token "F", "R'", "U2" into (face, quarter_turns)
  bool parse_cube_token_(const String &tok, char &face, int &qt) const;

  // Find which physical face currently holds a given logical face.
  // Returns 'U','R','F','D','L','B' or '\0' if not found.
  char find_physical_dir_for_logical_(char logical_face) const;
};
