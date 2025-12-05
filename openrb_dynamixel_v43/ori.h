#pragma once

#include <Arduino.h>

// Simple orientation / movement controller.
// Tracks which logical faces (U,D,F,B,L,R) are currently at each physical
// direction (up, down, front, back, left, right). Uses only y/z rotations
// and d turns for hardware.

class CubeOri {
public:
  // Callback type for sending a single physical move to the robot.
  // The move string is always canonical: "y+", "y'", "z+", "z'", "z2",
  // "d+", "d'", "d2".
  typedef bool (*robot_move_cb_t)(const String &move);

  CubeOri(robot_move_cb_t cb = nullptr);

  // Reset orientation to identity: U up, D down, F front, etc.
  void reset();

  // Set / change the robot callback.
  void set_robot_callback(robot_move_cb_t cb);

  // ------------------------------------------------------------
  // PUBLIC API
  // ------------------------------------------------------------

  // Execute a sequence of logical cube moves, e.g.:
  // "R U R' U'" or "f2 d' l".
  //
  // Supported (case-insensitive):
  //   F, F', F2
  //   B, B', B2
  //   R, R', R2
  //   L, L', L2
  //   U, U', U2
  //   D, D', D2
  //
  // Returns true if *all* moves were valid and executed.
  // On first invalid token, stops and returns false.
  bool cube_move(const String &moves_str);

  // Execute a single physical robot move (no parsing of sequences).
  //
  // Accepted (case-insensitive, ' or - for prime):
  //   y+, y'
  //   z+, z', z2
  //   d+, d', d2
  //
  // Returns false on invalid move; orientation is unchanged in that case.
  bool robot_move(const String &move_str);

  // Physically rotate the cube so that the logical axes align with the
  // original reference:
  //   Up -> physical up
  //   Down -> physical down
  //   Front -> physical front
  //   ...
  //
  // Uses only y+, y', z+, z' via robot_move().
  // Returns true if a path is found (should always be true).
  bool restore_orientation();

  // Human-readable description of current orientation:
  // "U=<face> D=<face> F=<face> B=<face> L=<face> R=<face>"
  String get_orientation_string() const;

  // Returns the physical move log (all canonical robot moves, space-separated).
  String get_orientation_log() const {
    return orientation_log_str_;
  }

private:
  struct orientation_t {
    char up;
    char down;
    char front;
    char back;
    char left;
    char right;
  };

  orientation_t ori_;
  robot_move_cb_t robot_cb_;
  String orientation_log_str_;

  // Directions indices for internal use.
  enum dir_index_t {
    DIR_U = 0,
    DIR_D = 1,
    DIR_F = 2,
    DIR_B = 3,
    DIR_L = 4,
    DIR_R = 5
  };

  // ------------------------------------------------------------
  // INTERNAL HELPERS
  // ------------------------------------------------------------

  // Normalize and validate a physical move token.
  // Returns "" on invalid.
  static String normalize_robot_move_token_(const String &in);

  // Parse a single logical move token (e.g. "F", "r'", "u2").
  // face_char: 'F','B','R','L','U','D'
  // quarter_turns: +1 (CW), -1 (CCW), +2 (180 deg).
  // Returns false on invalid token.
  static bool parse_cube_move_token_(const String &token,
                                     char &face_char,
                                     int &quarter_turns);

  // Execute one logical face move (already parsed).
  bool execute_single_cube_move_(char face_char, int quarter_turns);

  // Find which direction currently holds the given logical face.
  // Returns DIR_* or -1 if not found (should not happen).
  int find_face_direction_index_(char face_char) const;

  // Rotate cube so that the given direction index (DIR_*) becomes down.
  // Uses only y+, y', z+, z', z2 via robot_move().
  bool bring_direction_to_down_(int dir_index);

  // Apply orientation change for a physical rotation move
  // (y+, y', z+, z', z2) to ori_.
  void apply_rotation_in_place_(const String &move_str);

  // Rotation primitives on a generic orientation_t.
  static void rotate_y_plus_(orientation_t &o);
  static void rotate_y_minus_(orientation_t &o);
  static void rotate_z_plus_(orientation_t &o);
  static void rotate_z_minus_(orientation_t &o);

  // Apply a rotation move to a *copy* of an orientation (used by BFS).
  static orientation_t apply_rotation_to_orientation_(const orientation_t &in,
                                                      const String &move_str);

  // Comparison helper
  static bool orientations_equal_(const orientation_t &a,
                                  const orientation_t &b);

  // Split a sequence string into tokens separated by whitespace / commas.
  static void split_moves_(const String &moves_str,
                           String tokens[], int &count, int max_tokens);
};
