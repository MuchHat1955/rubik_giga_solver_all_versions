#include "color_reader.h"
#include "utils.h"
#include "cmd_parser.h"  // if you have serial_printf; otherwise remove

extern CubeOri ori;

// ------------------------------------------------------------
// Constructor
// ------------------------------------------------------------

CubeColorReader::CubeColorReader(CubeOri &ori, read_color_cb_t cb)
  : ori_(ori), cb_(cb) {
  fill_unknown_();
}

// ------------------------------------------------------------
// Private helpers
// ------------------------------------------------------------

void CubeColorReader::fill_unknown_() {
  for (int i = 0; i < 54; ++i) {
    colors_[i] = 'x';
  }
}

void CubeColorReader::clear() {
  fill_unknown_();
}

int CubeColorReader::face_index_(char face) {
  switch (face) {
    case 'U': return 0;
    case 'R': return 1;
    case 'F': return 2;
    case 'D': return 3;
    case 'L': return 4;
    case 'B': return 5;
    default:  return -1;
  }
}

bool CubeColorReader::bring_face_to_front_(char face) {
  // Ensure we start from reference orientation.
  if (!ori_.restore_cube_orientation()) {
    serial_printf("ERR CubeColorReader: restore_cube_orientation() failed\n");
    return false;
  }

  // From the reference, apply a fixed sequence to put the requested
  // logical face at the physical FRONT.
  //
  // These were derived from the orientation tables, assuming
  // rotations:
  //   y+, y', z+, z', z2
  //
  // and starting from identity.
  bool ok = true;

  switch (face) {
    case 'F':
      // Already at front in the reference orientation.
      break;

    case 'R':
      ok = ori_.robot_move("z+");
      break;

    case 'B':
      // z2 is allowed directly in your robot move table.
      ok = ori_.robot_move("z2");
      break;

    case 'L':
      ok = ori_.robot_move("z'");
      break;

    case 'U':
      // Minimal sequence bringing original U to FRONT:
      //   y+ then z'
      ok = ori_.robot_move("y+");
      if (ok) ok = ori_.robot_move("z'");
      break;

    case 'D':
      // Minimal sequence bringing original D to FRONT:
      //   y+ then z+
      ok = ori_.robot_move("y+");
      if (ok) ok = ori_.robot_move("z+");
      break;

    default:
      serial_printf("ERR CubeColorReader: invalid face %c\n", face);
      return false;
  }

  if (!ok) {
    serial_printf("ERR CubeColorReader: robot_move sequence failed for face %c\n", face);
  }
  return ok;
}

void CubeColorReader::merge_face_colors_(char face, const String &s) {
  int idx = face_index_(face);
  if (idx < 0) {
    serial_printf("ERR CubeColorReader: merge_face_colors invalid face %c\n", face);
    return;
  }

  if (s.length() < 9) {
    serial_printf("WARN CubeColorReader: string len %d < 9 for face %c\n", s.length(), face);
  }

  int base = idx * 9;
  for (int i = 0; i < 9; ++i) {
    char new_c = (i < s.length()) ? s.charAt(i) : 'x';
    if (new_c == '\0') new_c = 'x';

    // Only overwrite if we got an actual color (non 'x')
    if (new_c != 'x') {
      colors_[base + i] = new_c;
    }
  }
}

// ------------------------------------------------------------
// Public API: read_full_cube()
// ------------------------------------------------------------

bool CubeColorReader::read_full_cube() {
  if (!cb_) {
    serial_printf("ERR CubeColorReader: no callback set\n");
    return false;
  }

  // Start fresh
  fill_unknown_();

  // Desired face order
  const char faces[6] = { 'U', 'R', 'F', 'D', 'L', 'B' };

  for (int fi = 0; fi < 6; ++fi) {
    char f = faces[fi];

    // Bring that logical face to the front
    if (!bring_face_to_front_(f)) {
      serial_printf("ERR CubeColorReader: bring_face_to_front_ failed for %c\n", f);
      ori_.restore_cube_orientation();  // best effort to leave cube sane
      return false;
    }

    // First pass: typically "top + middle rows"
    String s1 = cb_(true);
    if (s1.length() != 9) {
      serial_printf("WARN CubeColorReader: cb(true) len=%d for face %c\n", s1.length(), f);
    }
    merge_face_colors_(f, s1);

    // Second pass: typically "remaining band" or refined read
    String s2 = cb_(false);
    if (s2.length() != 9) {
      serial_printf("WARN CubeColorReader: cb(false) len=%d for face %c\n", s2.length(), f);
    }
    merge_face_colors_(f, s2);
  }

  // Restore cube to reference orientation at the end
  if (!ori_.restore_cube_orientation()) {
    serial_printf("ERR CubeColorReader: final restore_cube_orientation() failed\n");
    return false;
  }

  return true;
}

// ------------------------------------------------------------
// Public API: get_cube_colors_string()
// ------------------------------------------------------------

String CubeColorReader::get_cube_colors_string() const {
  String s;
  s.reserve(54);
  for (int i = 0; i < 54; ++i) {
    s += colors_[i];
  }
  return s;
}

// Somewhere during setup:
CubeColorReader color_reader(ori, read_color_rows_cb);