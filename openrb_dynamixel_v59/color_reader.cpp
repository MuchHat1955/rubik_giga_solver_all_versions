#include "color_reader.h"
#include "utils.h"
#include "cmd_parser.h"
#include "ori.h"

extern CubeOri ori;

/*
COLORSCAN logging (emitted by CubeColorReader)

RB_INFO COLORSCAN (cNN)
  info=scan_start            total_steps, mode
  info=step_start            step, face, mirrored, order
  info=robot_move_start      step, robot_move
  info=robot_move_end        step, robot_move
  info=slot_read             step, face, slot, color
  info=step_complete         step, total_steps
  info=scan_complete         total_steps, mode
  info=orientation_restore_start
  info=orientation_restore_end

RB_ERR COLORSCAN (cNN)
  err=no_callback
  err=invalid_slot           face, slot
  err=invalid_move           move
  err=robot_move_failed      step, robot_move
  err=step_failed            step, total_steps
  err=step_count_invalid     step_count
  err=final_restore_failed
*/

// ============================================================
// Constructor
// ============================================================
CubeColorReader::CubeColorReader(CubeOri &ori, read_color_cb_t cb)
  : ori_(ori), cb_(cb) {
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
    RB_ERR_COLORSCAN("invalid_slot", "(na)",
                     "face=%c slot=%d",
                     face, slot);
    return;
  }

  int offset = -1;

  if (!mirrored) {
    switch (slot) {
      case 1: offset = 0; break;
      case 2: offset = 1; break;
      case 3: offset = 2; break;
      case 4: offset = 3; break;
      case 5: offset = 4; break;
      case 6: offset = 5; break;
    }
  } else {
    switch (slot) {
      case 1: offset = 8; break;
      case 2: offset = 7; break;
      case 3: offset = 6; break;
      default:
        RB_ERR_COLORSCAN("invalid_slot_mirrored", "(na)",
                         "face=%c slot=%d",
                         face, slot);
        return;
    }
  }

  colors_[base + offset] = color;
  update_color_string(face, offset, color);

  RB_INFO_COLORSCAN("slot_read", "(na)",
                    "face=%c slot=%d offset=%d color=%c",
                    face, slot, offset, color);
}

// Print compact face state
void CubeColorReader::print_face_compact(char face) const {
  int base = face_base_index_(face);
  if (base < 0) return;

  serial_printf_verbose("%c[", face);
  for (int i = 0; i < 9; i++)
    serial_printf_verbose("%c", colors_[base + i]);
  serial_printf_verbose("]\n");
}

void CubeColorReader::rotate_face(char face, char dir) {
  bool cw = (dir == '+');  // clockwise
  face = tolower(face);

  int base = face_base_index_(face);
  if (base < 0) {
    RB_ERR_COLORSCAN("rotate_face_invalid_face", rb_id_,
                 "face=%c", face);
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
    while (i < len && isspace(moves[i])) i++;

    int start = i;
    while (i < len && !isspace(moves[i])) i++;
    if (start == i) continue;

    String token = moves.substring(start, i);
    token.trim();

    if (!is_valid_move(token)) {
      RB_ERR_COLORSCAN("invalid_move", "(na)",
                       "move=%s",
                       token.c_str());
      continue;
    }

    char face = tolower(token[0]);
    char dir = token[1];

    rotate_face(face, dir);

    RB_INFO_COLORSCAN("apply_move", "(na)",
                      "move=%c%c",
                      face, dir);
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

// ============================================================
// One step processor
// ============================================================
bool CubeColorReader::process_step_(int step_index,
                                    const char *robot_move,
                                    const char *face,
                                    bool mirrored,
                                    const char *order) {

  // --- robot move ---
  if (robot_move && robot_move[0] && strcmp(robot_move, "none") != 0) {

    RB_INFO_COLORSCAN("robot_move_start", "(na)",
                      "step=%d robot_move=%s",
                      step_index, robot_move);

    if (!ori_.robot_move(robot_move)) {
      RB_ERR_COLORSCAN("robot_move_failed", "(na)",
                       "step=%d robot_move=%s",
                       step_index, robot_move);
      return false;
    }

    RB_INFO_COLORSCAN("robot_move_end", "(na)",
                      "step=%d robot_move=%s",
                      step_index, robot_move);
  }

  if (!face || !order || !face[0] || !order[0])
    return true;

  char F = tolower(face[0]);

  RB_INFO_COLORSCAN("step_start", "(na)",
                    "step=%d face=%c mirrored=%d order=%s",
                    step_index, F, mirrored ? 1 : 0, order);

  for (int i = 0; order[i]; i++) {
    char d = order[i];
    if (d < '1' || d > '6') continue;

    int slot = d - '0';
    char color = cb_ ? cb_(slot) : '.';

    apply_slot_to_face_(F, slot, color, mirrored);
  }

  RB_INFO_COLORSCAN("step_complete", "(na)",
                    "step=%d",
                    step_index);

  return true;
}


// ============================================================
// Perform full scan
// ============================================================
bool CubeColorReader::read_cube_full() {
  return read_cube(true);
}
bool CubeColorReader::read_cube_bottom() {
  return read_cube(false);
}

bool CubeColorReader::read_cube(bool mode_all_vs_bottom) {

  if (!cb_) {
    RB_ERR_COLORSCAN("no_callback", "(na)", "");
    return false;
  }

  int total_steps = mode_all_vs_bottom
                      ? k_num_color_map_steps_all
                      : k_num_color_map_steps_bottom;

  if (total_steps < 1) {
    RB_ERR_COLORSCAN("step_count_invalid", "(na)",
                     "step_count=%d",
                     total_steps);
    return false;
  }

  RB_INFO_COLORSCAN("scan_start", "(na)",
                    "total_steps=%d mode=%s",
                    total_steps,
                    mode_all_vs_bottom ? "all" : "bottom");

  if (mode_all_vs_bottom) fill_unknown_();
  else fill_solved_cube_top2layers_();

  ori_.clear_orientation_data();

  const auto *steps = mode_all_vs_bottom
                        ? k_color_map_steps_all
                        : k_color_map_steps_bottom;

  for (int i = 0; i < total_steps; i++) {
    if (!process_step_(i,
                       steps[i].robot_move,
                       steps[i].face,
                       steps[i].mirrored,
                       steps[i].order)) {

      RB_ERR_COLORSCAN("step_failed", "(na)",
                       "step=%d total_steps=%d",
                       i, total_steps);

      ori_.restore_cube_orientation();
      return false;
    }
  }

  RB_INFO_COLORSCAN("scan_complete", "(na)",
                    "total_steps=%d mode=%s",
                    total_steps,
                    mode_all_vs_bottom ? "all" : "bottom");

  RB_INFO_COLORSCAN("orientation_restore_start", "(na)", "");

  if (!ori_.restore_cube_orientation()) {
    RB_ERR_COLORSCAN("final_restore_failed", "(na)", "");
    return false;
  }

  RB_INFO_COLORSCAN("orientation_restore_end", "(na)", "");

  return true;
}

// ============================================================
// Return full cube string
// ============================================================
String CubeColorReader::get_cube_colors_string() const {
  String s;
  s.reserve(54);
  for (int i = 0; i < 54; i++)
    s += colors_[i];
  return s;
}

// eg update_color_string('f', 0, 'r');
void CubeColorReader::update_color_string(char face, int offset, char color) {
  if (offset < 0 || offset >= 9) {
    serial_printf_verbose("DEBUG COLORSREAD update_color_string invalid_offset=%d\n\n", offset);
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
      serial_printf_verbose("invalid face %c\n", face);
      return;
  }

  colors_[base + offset] = color;
}

void CubeColorReader::print_cube_colors_string() {

  if (!verboseOn) return;

  String cube = "";
  cube.reserve(54);
  for (int i = 0; i < 54; i++)
    cube += colors_[i];

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

CubeColorReader color_reader(ori, read_one_color_cb);
