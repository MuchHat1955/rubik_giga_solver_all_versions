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
  void clear_orientation_data();

  // Set hardware callback
  void set_robot_callback(robot_move_cb_t cb) {
    robot_cb_ = cb;
  }

  // ------------------------------------------------------------
  // PUBLIC API
  // ------------------------------------------------------------

  // Execute a single robot move (y+, y', z+, z', z2, d+, d', d2).
  bool robot_move(const String &move_str);

  // Execute a sequence of cube logical moves, e.g. "F R' U2".
  bool cube_move(const String &moves_str);

  // Restore cube orientation back to identity: U->U, R->R ...
  bool restore_cube_orientation();  // <-- You were missing this

  // Return mapping of original logical faces to current physical faces.
  void get_face_mapping(String out[6]) const;

  // One-line orientation string like:
  // "U->F R->R F->D D->B L->U B->L"
  String get_orientation_string() const;

  void print_orientation_string() const;

  // Move-log helpers
  String get_move_log() const {
    return orientation_log_;
  }
  void clear_move_log() {
    orientation_log_ = "";
  }

private:

  // ============================================================
  // Orientation struct
  //
  // Stores, for each PHYSICAL face (U,R,F,D,L,B),
  // which LOGICAL face is currently sitting there.
  //
  // Identity:
  //   ori_.U='U'
  //   ori_.R='R'
  //   ori_.F='F'
  //   ori_.D='D'
  //   ori_.L='L'
  //   ori_.B='B'
  // ============================================================
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

  // ------------------------------------------------------------
  // Private helpers
  // ------------------------------------------------------------

  // Split "a b c" or "a,b,c" etc
  void split_moves_(const String &in, String *out, int &count, int max_count) const;

  // Update orientation using lookup table
  void apply_ori_table_(const String &robot_move);

  // Parse cube token "F", "R'", "U2"
  bool parse_cube_token_(const String &tok, char &face, int &qt) const;

  // Find which PHYSICAL direction holds a logical face:
  // returns 'U','R','F','D','L','B'
  char find_physical_dir_for_logical_(char logical_face) const;

  // Execute one cube move (already parsed)
  bool execute_single_cube_move_(char logical_face, int qt);

  bool orientations_equal_(const Orientation &a, const Orientation &b) const;

  Orientation apply_rotation_to_orientation_(const Orientation &o, const String &move) const;
};
