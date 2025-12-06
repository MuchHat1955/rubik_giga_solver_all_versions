#pragma once

#include <Arduino.h>
#include "ori.h"

// Class that manages reading all cube colors using CubeOri and a callback.
//
// - It moves the cube so that each logical face (U, R, F, D, L, B)
//   becomes the *front* face in turn.
// - For each face, it calls a user-provided callback that returns
//   a 9-character string representing that face's colors in row-major
//   order (top-left -> bottom-right).
// - Unread stickers must be marked as 'x' by the callback.
// - The class merges multiple reads, preferring non-'x' over 'x'.
// - At the end, it restores the cube to the reference orientation
//   using ori.restore_cube_orientation() and provides a 54-character string
//   in URFDLB order.
class CubeColorReader {
public:
  // Callback type:
  //   read_two_rows == true  -> typically read more (e.g., top+middle rows)
  //   read_two_rows == false -> typically read the remaining band
  //
  // Return value:
  //   String of length 9 (row-major 3x3, 'x' for unread stickers).
  using read_color_cb_t = String (*)(bool read_two_rows);

  // Constructor
  CubeColorReader(CubeOri &ori, read_color_cb_t cb);

  // Read all faces.
  //
  // Returns true if all robot movements and callback calls succeeded.
  // It does NOT guarantee that there are no 'x' characters left in
  // the final 54-char string; you can check for that separately if
  // you want to assert full coverage.
  bool read_full_cube();

  // Get the final 54-character string in URFDLB face order:
  //
  //   U (0..8), R(9..17), F(18..26),
  //   D(27..35), L(36..44), B(45..53)
  //
  // Unread stickers (if any) are 'x'.
  String get_cube_colors_string() const;

  // Clear internal colors buffer back to 'x'.
  void clear();

private:
  CubeOri &ori_;
  read_color_cb_t cb_;
  char colors_[54];

  // Map logical face char to index 0..5 (U,R,F,D,L,B)
  static int face_index_(char face);

  // Bring the given logical face to the physical FRONT using only
  // y+, y', z+, z', z2 and ori.robot_move().
  //
  // Implementation assumption: ori.restore_cube_orientation() brings the
  // cube to a known reference where:
  //   - logical U is physical up
  //   - logical F is physical front
  //   - etc.
  //
  // From that reference, we use fixed sequences:
  //   F: (none)
  //   R: z+
  //   B: z2
  //   L: z'
  //   U: y+ then z'
  //   D: y+ then z+
  bool bring_face_to_front_(char face);

  // Merge one 9-char string into the internal buffer for a given face.
  // For each position:
  //   - if new_char != 'x', it overwrites the stored char
  //   - if new_char == 'x', the stored char is preserved
  void merge_face_colors_(char face, const String &s);

  // Helper: set all 54 entries to 'x'.
  void fill_unknown_();
};
