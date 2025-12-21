#include "color_reader.h"
#include "utils.h"
#include "cmd_parser.h"
#include "ori.h"

extern CubeOri ori;

// ============================================================
// Constructor
// ============================================================
CubeColorReader::CubeColorReader(CubeOri &ori, read_color_cb_t cb)
  : ori_(ori), color_sensor_cb_(cb) {
  fill_unknown_();
}

// ============================================================
// Helpers
// ============================================================
void CubeColorReader::fill_unknown_() {
  for (int i = 0; i < 54; i++) colors_[i] = '.';
}

void CubeColorReader::clear() {
  fill_unknown_();
}

// Return base index in colors_[ ] for a face letter
int CubeColorReader::face_base_index_(char face) const {
  switch (face) {
    case 'u': return 0;
    case 'r': return 9;
    case 'f': return 18;
    case 'd': return 27;
    case 'l': return 36;
    case 'b': return 45;
  }
  return -1;
}

// Apply a read color for a single slot of a face:
// slot = 1..6, band semantics:
//   non-mirrored: 1,2,3 = top row (L,C,R), 4,5,6 = middle row (L,C,R)
//   mirrored:     1,2,3 = bottom row (L,C,R)
void CubeColorReader::apply_slot_to_face_(char face, int slot, char color, bool mirrored) {
  int base = face_base_index_(face);
  if (base < 0) return;

  if (slot < 1 || slot > 6) {
    LOG_ERR(MOD_COLORSCAN, "error", "invalid color reader slot");
    LOG_VAR("slot", slot);
    LOG_VAR("face", face);
    return;
  }

  int offset = -1;

  if (!mirrored) {
    // Normal reading: top row + middle row
    switch (slot) {
      case 1: offset = 0; break;  // top-left
      case 2: offset = 1; break;  // top-center
      case 3: offset = 2; break;  // top-right
      case 4: offset = 3; break;  // mid-left
      case 5: offset = 4; break;  // mid-center
      case 6: offset = 5; break;  // mid-right
    }
  } else {
    // Mirrored bottom band:
    // 1 → bottom-left, 2 → bottom-center, 3 → bottom-right
    switch (slot) {
      case 1: offset = 8; break;  // bottom-left
      case 2: offset = 7; break;  // bottom-center
      case 3: offset = 6; break;  // bottom-right
      default:
        // slots 4,5,6 shouldn't be used in mirrored mode
        LOG_ERR(MOD_COLORSCAN, "skipping mirrored slot", slot);
        LOG_VAR("face", face);
        return;
    }
  }
  update_color_string(face, offset, color);

  if (offset >= 0) {
    colors_[base + offset] = color;
    LOG_INFO(MOD_COLORSCAN, "read face", face);
    LOG_VAR("slot", offset + 1);
    LOG_VAR("color", color);
  }
}

// Print compact face state
String CubeColorReader::get_color_string_face(char face) const {
  int base = face_base_index_(face);
  if (base < 0) return "";
  String return_str = String(face) + "=";
  for (int i = 0; i < 9; i++) {
    return_str += String(colors_[base + i]);
  }
  return return_str;
}

void CubeColorReader::rotate_face(char face, char dir) {
  bool cw = (dir == '+');  // clockwise
  face = tolower(face);

  int base = face_base_index_(face);
  if (base < 0) {
    LOG_ERR(MOD_COLORSCAN, "invalid face", face);
    return;
  }

  // ------------------------------------------------------------
  // 1. Rotate the face (3x3 matrix) - This part was correct.
  // ------------------------------------------------------------
  char t[9];
  memcpy(t, &colors_[base], 9);

  if (cw) {
    // clockwise: 0->6, 1->3, 2->0, 3->7, 4->4, 5->1, 6->8, 7->5, 8->2
    colors_[base + 0] = t[6];
    colors_[base + 1] = t[3];
    colors_[base + 2] = t[0];
    colors_[base + 3] = t[7];
    colors_[base + 4] = t[4];
    colors_[base + 5] = t[1];
    colors_[base + 6] = t[8];
    colors_[base + 7] = t[5];
    colors_[base + 8] = t[2];
  } else {
    // counter-clockwise: 0->2, 1->5, 2->8, 3->1, 4->4, 5->7, 6->0, 7->3, 8->6
    colors_[base + 0] = t[2];
    colors_[base + 1] = t[5];
    colors_[base + 2] = t[8];
    colors_[base + 3] = t[1];
    colors_[base + 4] = t[4];
    colors_[base + 5] = t[7];
    colors_[base + 6] = t[0];
    colors_[base + 7] = t[3];
    colors_[base + 8] = t[6];
  }

  // ------------------------------------------------------------
  // 2. Surrounding edges (the "ring")
  // ------------------------------------------------------------
  int idx[4][3];

  auto set3 = [&](int i, int a, int b, int c) {
    idx[i][0] = a;
    idx[i][1] = b;
    idx[i][2] = c;
  };

  // Assuming the standard U=0, R=9, F=18, D=27, L=36, B=45 index map.
  // Neighbors are listed in clockwise order around the face.

  if (face == 'u') {
    // F top → R top → B top → L top
    set3(0, 18, 19, 20);  // F0, F1, F2
    set3(1, 9, 10, 11);   // R0, R1, R2
    set3(2, 45, 46, 47);  // B0, B1, B2
    set3(3, 36, 37, 38);  // L0, L1, L2
  } else if (face == 'd') {
    // F bottom → L bottom → B bottom → R bottom
    set3(0, 24, 25, 26);  // F6, F7, F8
    set3(1, 42, 43, 44);  // L6, L7, L8
    set3(2, 51, 52, 53);  // B6, B7, B8
    set3(3, 15, 16, 17);  // R6, R7, R8
  } else if (face == 'f') {
    // U bottom → R left column → D top (reversed) → L right column
    set3(0, 6, 7, 8);     // U6, U7, U8
    set3(1, 9, 12, 15);   // R0, R3, R6
    set3(2, 29, 28, 27);  // D2, D1, D0 (Reversed)
    set3(3, 38, 41, 44);  // L2, L5, L8
  } else if (face == 'b') {
    // U top (reversed) → L left column → D bottom → R right column (reversed)
    set3(0, 2, 1, 0);     // U2, U1, U0 (Reversed)
    set3(1, 36, 39, 42);  // L0, L3, L6
    set3(2, 33, 34, 35);  // D6, D7, D8
    set3(3, 17, 14, 11);  // R8, R5, R2 (Reversed)
  } else if (face == 'l') {
    // U left col → B right col (reversed) → D left col → F left col
    set3(0, 0, 3, 6);     // U0, U3, U6
    set3(1, 53, 50, 47);  // B8, B5, B2 (Reversed)
    set3(2, 27, 30, 33);  // D0, D3, D6
    set3(3, 18, 21, 24);  // F0, F3, F6
  } else if (face == 'r') {
    // U right col → F right col → D right col → B left col (reversed)
    set3(0, 2, 5, 8);     // U2, U5, U8
    set3(1, 20, 23, 26);  // F2, F5, F8
    set3(2, 29, 32, 35);  // D2, D5, D8
    set3(3, 51, 48, 45);  // B6, B3, B0 (Reversed)
  } else {
    return;
  }

  // ------------------------------------------------------------
  // 3. Perform the 4x3 edge cycle - This part was correct.
  // ------------------------------------------------------------
  char buf[12];

  // Store stickers *before* the cycle
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
      buf[i * 3 + j] = colors_[idx[i][j]];

  // Move stickers: source (src) to destination (i)
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++) {
      // Source is (i - 1) mod 4 for CW, (i + 1) mod 4 for CCW
      int src = cw ? (i + 3) % 4 : (i + 1) % 4;
      colors_[idx[i][j]] = buf[src * 3 + j];
    }
}

// ============================================================
// adjust the color string for standard list of one or more moves f+ etc
// ============================================================

// List of all allowed robot moves.
// Use whatever notation you actually use: f+, f', f2, etc.
static const char *k_valid_moves[] = {
  "f+", "f-", "f2",
  "b+", "b-", "b2",
  "u+", "u-", "u2",
  "d+", "d-", "d2",
  "l+", "l-", "l2",
  "r+", "r-", "r2",
  // add any others here (cube rotations, etc.)
};

bool is_valid_move(const String &token) {
  for (const char *m : k_valid_moves) {
    if (token == m) {
      return true;
    }
  }
  return false;
}

void CubeColorReader::apply_moves(const String &moves) {
  int len = moves.length();
  int i = 0;

  while (i < len) {
    // Skip leading spaces
    while (i < len && isspace(moves[i])) i++;

    // Find end of token
    int start = i;
    while (i < len && !isspace(moves[i])) i++;
    int end = i;

    if (start == end) continue;  // empty segment

    // Extract token (ex: "f+", "u2", "r'")
    String token = moves.substring(start, end);
    token.trim();

    if (token.length() == 0) continue;

    // Validate token using your single source of truth
    if (!is_valid_move(token)) {

      LOG_ERR(MOD_COLORSCAN, "invalid move", token.c_str());

      continue;
    }

    // Token is valid → parse
    // token = "<face><dir>" where dir is '+', '-', '\'', or '2'
    char face = tolower(token[0]);
    char dir = token[1];

    rotate_face(tolower(face), dir);
    //serial_printf_verbose("applied move: %c%c", tolower(face), dir);
    // print_cube_colors_diagram();
  }
}

/*
if (move.equalsIgnoreCase("z_plus")) {
    n.U = o.L;
    n.R = o.U;
    n.D = o.R;
    n.L = o.D;
    // unchanged
    n.F = o.F;
    n.B = o.B;
  } else if (move.equalsIgnoreCase("z_minus")) {
    n.U = o.R;
    n.L = o.U;
    n.D = o.L;
    n.R = o.D;
    // unchanged
    n.F = o.F;
    n.B = o.B;
  } else if (move.equalsIgnoreCase("y_plus")) {
    n.F = o.L;
    n.L = o.B;
    n.B = o.R;
    n.R = o.F;
    // unchanged
    n.U = o.U;
    n.D = o.D;
  } else if (move.equalsIgnoreCase("y_minus")) {
    n.F = o.R;
    n.R = o.B;
    n.B = o.L;
    n.L = o.F;
    // unchanged
    n.U = o.U;
    n.D = o.D;
  } else if (move.equalsIgnoreCase("z_180")) {
    n.U = o.D;
    n.D = o.U;
    n.R = o.L;
    n.L = o.R;
    // unchanged
    n.F = o.F;
    n.B = o.B;
  } else if (move.equalsIgnoreCase("y_180")) {
    n.F = o.B;
    n.R = o.L;
    n.B = o.F;
    n.L = o.R;
    // unchanged
    n.U = o.U;
    n.D = o.D;}
*/

// ============================================================
// Mapping table (with explicit mirrored flag)
// ============================================================

bool not_inverted = false;
bool inverted = true;

static const color_map_step_t k_color_map_steps_all[] = {

  // -----------------------------------------------------------
  // 0) none
  //      U
  //   L  F  R  B  [not inverted] F-U edge is up
  //      D
  // -----------------------------------------------------------
  { "none", "f", not_inverted, "145632" },

  // -----------------------------------------------------------
  // 1) y_minus
  //
  // --- orintentation after the step ---
  //      U
  //   F  R  B  L  [not inverted] R-U edge is up
  //      D
  // -----------------------------------------------------------
  { "y_plus", "r", not_inverted, "145632" },

  // -----------------------------------------------------------
  // 2) y_minus
  //
  // --- orintentation after the step ---
  //      U
  //   R  B  L  F  [not inverted] B-U edge is up
  //      D
  // -----------------------------------------------------------
  { "y_plus", "b", not_inverted, "145632" },

  // -----------------------------------------------------------
  // 3) y_minus
  //
  // --- orintentation after the step ---
  //      U
  //   B  L  F  R  [not inverted] L-U edge is up
  //      D
  // -----------------------------------------------------------
  { "y_plus", "l", not_inverted, "145632" },

  // -----------------------------------------------------------
  // 4) z_180
  //
  // --- orintentation after the step ---
  //      D
  //   F  L  B  R  [inverted] L-U edge is down
  //      U
  // -----------------------------------------------------------
  { "z_180", "l", inverted, "231" },

  // -----------------------------------------------------------
  // 5) y_minus
  //
  // --- orintentation after the step --------------------------
  //      D
  //   L  B  R  F  [inverted] B-U edge is down
  //      U
  // -----------------------------------------------------------
  { "y_plus", "b", inverted, "231" },

  // -----------------------------------------------------------
  // 6) y_minus
  //
  // --- orintentation after the step --------------------------
  //      D
  //   B  R  F  L  [inverted] R-U edge is down
  //      U
  // -----------------------------------------------------------
  { "y_plus", "r", inverted, "231" },

  // -----------------------------------------------------------
  // 7) y_minus
  //
  // --- orintentation after the step --------------------------
  //      D
  //   R  F  L  B  [inverted] F-U edge is down
  //      U
  // -----------------------------------------------------------
  { "y_plus", "f", inverted, "231" },

  // -----------------------------------------------------------
  // 8) z_plus
  //
  // --- orintentation after the step --------------------------
  //      R
  //   U  F  D  B  [reposition]
  //      L
  // -----------------------------------------------------------
  { "z_plus", "", not_inverted, "" },

  // -----------------------------------------------------------
  // 9) y_minus
  //      R
  //   F  D  B  U  [reposition]
  //      L
  // -----------------------------------------------------------
  { "y_plus", "", not_inverted, "" },

  // -----------------------------------------------------------
  // 9) z_plus,
  //
  // --- orintentation after the step --------------------------
  //      F
  //   L  D  R  U  [not inverted] D-F edge is up
  //      B
  //   -----------------------------------------------------------
  { "z_plus", "d", not_inverted, "145632" },

  // -----------------------------------------------------------
  // 10) y_180
  //
  // --- orintentation after the step --------------------------
  //      F
  //   R  U  L  D  [inverted] U-B edge is down
  //      B
  // -----------------------------------------------------------
  { "y_180", "u", inverted, "231" },

  // -----------------------------------------------------------
  // 11) z_180
  //
  // --- orintentation after the step --------------------------
  //      B
  //   L  U  R  D  [not inverted] U-B edge is up
  //      F
  // -----------------------------------------------------------
  { "z_180", "u", not_inverted, "145632" },

  // -----------------------------------------------------------
  // 12) y_180
  //
  // --- orintentation after the step --------------------------
  //      B
  //   R  D  L  U [inverted] D-F edge is down
  //      F
  // -----------------------------------------------------------
  { "y_180", "d", inverted, "231" }  // end
};

static const int k_num_color_map_steps_all =
  sizeof(k_color_map_steps_all) / sizeof(k_color_map_steps_all[0]);

static const color_map_step_t k_color_map_steps_bottom[] = {

  // -----------------------------------------------------------
  // 1) z_180
  //
  // --- orintentation after the step --------------------------
  //      D
  //   R  F  L  B  [inverted] F-U edge is down
  //      U
  //
  // -----------------------------------------------------------
  { "z_180", "f", inverted, "132" },

  // -----------------------------------------------------------
  // 2) y_plus
  //
  // --- orintentation after the step --------------------------
  //      D
  //   F  L  B  R  [inverted] U-L edge is down
  //      U
  //
  // -----------------------------------------------------------
  { "y_plus", "l", inverted, "132" },

  // -----------------------------------------------------------
  // 3) y_plus
  //
  // --- orintentation after the step --------------------------
  //      D
  //   L  B  R  F  [inverted] U-B edge is down
  //      U
  //
  // -----------------------------------------------------------
  { "y_plus", "b", inverted, "132" },

  // -----------------------------------------------------------
  // 4) y_plus
  //
  // --- orintentation after the step --------------------------
  //      D
  //   B  R  F  L  [inverted] U-R edge is down
  //      U
  //
  // -----------------------------------------------------------
  { "y_plus", "r", inverted, "132" },

  // -----------------------------------------------------------
  // 4) z_plus
  //
  // --- orintentation after the step --------------------------
  //      B
  //   U  R  D  L  [reposition]
  //      F
  //
  // -----------------------------------------------------------
  { "z_plus", "", not_inverted, "" },

  // -----------------------------------------------------------
  // 5) y_plus
  //
  // --- orintentation after the step --------------------------
  //      B
  //   R  D  L  U   [inverted] D-F edge is down
  //      F
  //
  // -----------------------------------------------------------
  { "y_plus", "d", inverted, "132" },

  // -----------------------------------------------------------
  // 5) z_180
  //
  // --- orintentation after the step --------------------------
  //      F
  //   L  D  R  U   [not inverted] D-F edge is up
  //      B
  //
  // -----------------------------------------------------------
  { "z_180", "d", not_inverted, "145632" }  // end
};

static const int k_num_color_map_steps_bottom =
  sizeof(k_color_map_steps_bottom) / sizeof(k_color_map_steps_bottom[0]);

static const color_map_step_t k_color_map_steps_centers[] = {

  // -----------------------------------------------------------
  // 1) ""
  //
  // --- orintentation after the step --------------------------
  //      U
  //   R  F  L  B  []
  //      D
  //
  // -----------------------------------------------------------
  { "none", "f", not_inverted, "1" },

  // -----------------------------------------------------------
  // 2) y_plus
  //
  // --- orintentation after the step --------------------------
  //      U
  //   F  L  B  R  []
  //      D
  //
  // -----------------------------------------------------------
  { "y_plus", "l", not_inverted, "1" }
};

static const int k_num_color_map_steps_centers =
  sizeof(k_color_map_steps_centers) / sizeof(k_color_map_steps_centers[0]);

// ============================================================
// One step processor
// ============================================================
bool CubeColorReader::process_color_scan_step_(int step_index,
                                               const char *robot_move,
                                               const char *face,
                                               bool mirrored,
                                               const char *order) {

  // Robot move (if any)
  if (robot_move && robot_move[0] != '\0' && strcmp(robot_move, "none") != 0) {
    if (!ori_.robot_move(robot_move)) {
      LOG_ERR(MOD_COLORSCAN, "robot move failed", step_index);
      LOG_VAR("robot_move", robot_move);
      return false;
    }
  }

  // No read on this step?
  if (!face || face[0] == '\0' || !order || order[0] == '\0')
    return true;

  char F = tolower(face[0]);  // ensure lowercase for face_base_index_()

  // Read all slots in this step
  for (int i = 0; order[i] != '\0'; i++) {

    char d = order[i];
    if (d < '1' || d > '6') continue;

    int slot = d - '0';
    char color = color_sensor_cb_ ? color_sensor_cb_(slot) : '.';

    apply_slot_to_face_(F, slot, color, mirrored);
    LOG_INFO(MOD_COLORSCAN, "color_string_curr_face", String(F));
    LOG_VAR("color", get_color_string_face(F));
  }

  // log completion of this step for this face
  LOG_INFO(MOD_COLORSCAN, "color scan step", step_index);
  LOG_VAR("robot_move", robot_move ? robot_move : "");

  get_color_string_face(F);
  LOG_INFO(MOD_COLORSCAN, "cube_color_string_54", get_color_string_54().c_str());
  LOG_INFO(MOD_COLORSCAN, "cube_color_string_faces", get_color_string_faces().c_str());

  return true;
}

#define SCAN_MODE_FULL 0
#define SCAN_MODE_BOTTOM 0
#define SCAN_MODE_CENTERS 0

// ============================================================
// Perform  scan
// ============================================================
bool CubeColorReader::read_cube_full() {
  return read_cube(SCAN_MODE_FULL);
}
bool CubeColorReader::read_cube_bottom() {
  return read_cube(SCAN_MODE_BOTTOM);
}
bool CubeColorReader::read_cube_centers() {
  return read_cube(SCAN_MODE_CENTERS);
}

bool CubeColorReader::read_cube(int scan_mode) {
  if (scan_mode != SCAN_MODE_FULL &&    //
      scan_mode != SCAN_MODE_BOTTOM &&  //
      scan_mode != SCAN_MODE_CENTERS) {
    LOG_ERR(MOD_COLORSCAN, "invalid scan mode", scan_mode);
    return false;
  }

  if (!color_sensor_cb_) {
    LOG_ERR(MOD_COLORSCAN, "error", "no callback");
    return false;
  }
  int total_steps = k_num_color_map_steps_all;
  if (scan_mode == SCAN_MODE_BOTTOM) total_steps = k_num_color_map_steps_bottom;
  if (scan_mode == SCAN_MODE_CENTERS) total_steps = k_num_color_map_steps_centers;

  if (total_steps < 1) {
    LOG_ERR(MOD_COLORSCAN, "step count invalid", total_steps);
    return false;
  }
  String mode_string =
    (scan_mode == SCAN_MODE_BOTTOM) ? "bottom" :    //
      (scan_mode == SCAN_MODE_CENTERS) ? "centers"  //
                                       : "full";

  LOG_INFO(MOD_COLORSCAN, "color scan start", mode_string);
  LOG_VAR("total_steps", total_steps);

  if (scan_mode == SCAN_MODE_FULL) fill_unknown_();
  if (scan_mode == SCAN_MODE_BOTTOM) fill_solved_cube_top2layers_();

  // LOG_INFO(MOD_COLORSCAN, "info","color reader: orientation cleared");
  // Ensure orientation is clear
  if (scan_mode != SCAN_MODE_CENTERS) ori_.clear_orientation_data();
  const color_map_step_t *orientation_map_ptr = k_color_map_steps_all;
  if (scan_mode == SCAN_MODE_BOTTOM) orientation_map_ptr = k_color_map_steps_bottom;
  if (scan_mode == SCAN_MODE_CENTERS) orientation_map_ptr = k_color_map_steps_centers;

  // ~~~~~~~~~~~~~~~~ start scan ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (int i = 0; i < total_steps; i++) {
    const auto &s = orientation_map_ptr[i];
    if (!process_color_scan_step_(i, s.robot_move, s.face, s.mirrored, s.order)) {
      LOG_ERR(MOD_COLORSCAN, "step failed", i);
      LOG_INFO(MOD_COLORSCAN, "color scan failed", "restoring_ori");
      if (scan_mode != SCAN_MODE_CENTERS) ori_.restore_cube_orientation();
      return false;
    }
    // print_cube_colors_diagram();
  }
  // ~~~~~~~~~~~~~~~~ end scan ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  LOG_INFO(MOD_COLORSCAN, "info", "color scan completed");

  if (scan_mode != SCAN_MODE_CENTERS) {
    LOG_INFO(MOD_COLORSCAN, "info", "start orientation restore after color scan");
    // Final restore
    if (!ori_.restore_cube_orientation()) {
      LOG_ERR(MOD_COLORSCAN, "error", "final restore failed");
      return false;
    }
  }
  LOG_INFO(MOD_COLORSCAN, "color_string_54", get_color_string_54().c_str());
  LOG_INFO(MOD_COLORSCAN, "color_string_faces", get_color_string_faces().c_str());
  LOG_INFO(MOD_COLORSCAN, "orientation", ori.get_orientation_string().c_str());

  return true;
}

// eg update_color_string('f', 0, 'r');
void CubeColorReader::update_color_string(char face, int offset, char color) {
  if (offset < 0 || offset >= 9) {
    LOG_ERR(MOD_COLORSCAN, "error", "update_color_string_invalid_params");
    LOG_VAR("face", face);
    LOG_VAR("offset", offset);
    LOG_VAR("color", color);
    return;
  }

  int base = -1;
  switch (tolower(face)) {
    case 'u': base = 0; break;
    case 'r': base = 9; break;
    case 'f': base = 18; break;
    case 'd': base = 27; break;
    case 'l': base = 36; break;
    case 'b': base = 45; break;
    default:
      LOG_ERR(MOD_COLORSCAN, "error", "update_color_string_invalid_face");
      LOG_VAR("face", face);
      return;
  }

  colors_[base + offset] = color;
}

// ============================================================
// Return full cube string
// ============================================================
String CubeColorReader::get_color_string_54() const {
  String s;
  s.reserve(55);
  for (int i = 0; i < 54; i++)
    s += colors_[i];
  return s;
}

String CubeColorReader::get_color_string_faces() const {
  String color_str_faces;

  color_str_faces.reserve(73);
  color_str_faces = get_color_string_face('u');
  color_str_faces += String(" ") + get_color_string_face('u');
  color_str_faces += String(" ") + get_color_string_face('r');
  color_str_faces += String(" ") + get_color_string_face('f');
  color_str_faces += String(" ") + get_color_string_face('d');
  color_str_faces += String(" ") + get_color_string_face('l');
  color_str_faces += String(" ") + get_color_string_face('b');
  return color_str_faces;
}

//TODO not used
void CubeColorReader::print_cube_colors_diagram() {
  String cube = "";
  cube.reserve(54);
  for (int i = 0; i < 54; i++)
    cube += colors_[i];
  Serial.println();
  // Print U face (top)
  for (int i = 0; i < 3; i++) {
    Serial.print("       ");  // Padding
    for (int j = 0; j < 3; j++) {
      Serial.print(cube[i * 3 + j]);
      Serial.print(" ");
    }
    Serial.println();
  }

  // Print L, F, R, B faces (middle layer)
  for (int i = 0; i < 3; i++) {
    // L face
    for (int j = 0; j < 3; j++) {
      Serial.print(cube[36 + i * 3 + j]);
      Serial.print(" ");
    }
    Serial.print(" ");  // Separator

    // F face
    for (int j = 0; j < 3; j++) {
      Serial.print(cube[18 + i * 3 + j]);
      Serial.print(" ");
    }
    Serial.print(" ");  // Separator

    // R face
    for (int j = 0; j < 3; j++) {
      Serial.print(cube[9 + i * 3 + j]);
      Serial.print(" ");
    }
    Serial.print(" ");  // Separator

    // B face
    for (int j = 0; j < 3; j++) {
      Serial.print(cube[45 + i * 3 + j]);
      Serial.print(" ");
    }

    Serial.println();
  }

  // Print D face (bottom)
  for (int i = 0; i < 3; i++) {
    Serial.print("       ");  // Padding
    for (int j = 0; j < 3; j++) {
      Serial.print(cube[27 + i * 3 + j]);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println();
}

// ============================================================
// SOLVED cube template
// ============================================================

// Faces:
//   U: indices  0.. 8
//   R: indices  9..17
//   F: indices 18..26
//   D: indices 27..35
//   L: indices 36..44
//   B: indices 45..53

static const char solved_54[55] =
  "WWWWWWWWW"
  "RRRRRRRRR"
  "GGGGGGGGG"
  "YYYYYYYYY"
  "OOOOOOOOO"
  "BBBBBBBBB";

static const char solved_top2layers_54[55] =
  "WWWWWWWWW"
  "RRRRRR..."
  "GGGGGG..."
  "........."
  "OOOOOO..."
  "BBBBBB...";

void CubeColorReader::fill_solved_cube() {
  memcpy(colors_, solved_54, 54);
}
void CubeColorReader::fill_solved_cube_top2layers_() {
  memcpy(colors_, solved_top2layers_54, 54);
}

String CubeColorReader::get_color_string_centers() const {
  // Center indices for URFDLB
  static const int center_idx[6] = { 4, 13, 22, 31, 40, 49 };

  String out;
  out.reserve(6);

  for (int i = 0; i < 6; i++) {
    out += colors_[center_idx[i]];
  }

  return out;
}

CubeColorReader color_reader(ori, read_one_color_cb);
