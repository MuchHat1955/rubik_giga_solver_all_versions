#include "color_reader.h"
#include "utils.h"
#include "cmd_parser.h"
#include "ori.h"

extern CubeOri ori;
extern char crrColorChar;

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
    serial_printf("ERRcolor reader: invalid slot %d for face %c\n", slot, face);
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
        serial_printf("WARNcolor reader: mirrored with slot %d on face %c\n",
                      slot, face);
        return;
    }
  }

  if (offset >= 0) {
    colors_[base + offset] = color;
    serial_printf("COLORSREAD %c%d=%c\n", face, offset + 1, color);
  }
}

// Print compact face state
void CubeColorReader::print_face_compact(char face) const {
  int base = face_base_index_(face);
  if (base < 0) return;

  serial_printf("%c[", face);
  for (int i = 0; i < 9; i++)
    serial_printf("%c", colors_[base + i]);
  serial_printf("]\n");
}

void CubeColorReader::rotate_face(char face, char dir) {
  bool cw = (dir == '+');  // clockwise
  face = tolower(face);

  int base = face_base_index_(face);
  if (base < 0) {
    serial_printf("ERR invalid face %c\n", face);
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
      serial_printf("ERR invalid move: %s\n", token.c_str());
      continue;
    }

    // Token is valid → parse
    // token = "<face><dir>" where dir is '+', '-', '\'', or '2'
    char face = tolower(token[0]);
    char dir = token[1];

    rotate_face(tolower(face), dir);
    serial_printf_verbose("applied move: %c%c\n", tolower(face), dir);
    print_cube_colors_string();
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
struct color_map_step_t {
  const char *robot_move;  // movement: "y_plus", "y_minus", "z_minus", "y_180", …
  const char *face;        // face to read: "f","r","u","", …
  bool mirrored;           // true = bottom band (mirror), false = normal
  const char *order;       // slot order: "236541" or "231"
};

bool not_inverted = false;
bool inverted = true;

static const color_map_step_t k_color_map_steps[] = {

  // -----------------------------------------------------------
  // 0) none
  //      U
  //   L  F  R  B  [not inverted] F-U edge is up
  //      D
  // -----------------------------------------------------------
  { "none", "f", not_inverted, "236541" },

  // -----------------------------------------------------------
  // 1) y_minus
  //
  // --- orintentation after the step ---
  //      U
  //   F  R  B  L  [not inverted] R-U edge is up
  //      D
  // -----------------------------------------------------------
  { "y_plus", "r", not_inverted, "236541" },

  // -----------------------------------------------------------
  // 2) y_minus
  //
  // --- orintentation after the step ---
  //      U
  //   R  B  L  F  [not inverted] B-U edge is up
  //      D
  // -----------------------------------------------------------
  { "y_plus", "b", not_inverted, "236541" },

  // -----------------------------------------------------------
  // 3) y_minus
  //
  // --- orintentation after the step ---
  //      U
  //   B  L  F  R  [not inverted] L-U edge is up
  //      D
  // -----------------------------------------------------------
  { "y_plus", "l", not_inverted, "236541" },

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
  { "z_plus", "d", not_inverted, "236541" },

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
  { "z_180", "u", not_inverted, "236541" },

  // -----------------------------------------------------------
  // 12) y_180
  //
  // --- orintentation after the step --------------------------
  //      B
  //   R  D  L  U [inverted] D-F edge is down
  //      F
  // -----------------------------------------------------------
  { "y_180", "d", inverted, "231" },

  // -----------------------------------------------------------
  // 13) y_minus
  //
  // --- orintentation after the step --------------------------
  //      B
  //   D  L  U  R  [reposition]
  //      F
  // -----------------------------------------------------------
  { "y_plus", "", not_inverted, "" },

  // -----------------------------------------------------------
  //
  // --- orintentation after the step --------------------------
  // 14) z_minus
  //      U
  //   B  L  F  R  [reposition]
  //      D
  // -----------------------------------------------------------
  { "z_minus", "", not_inverted, "" },

  // -----------------------------------------------------------
  // 15) y_minus
  //
  // --- orintentation after the step --------------------------
  //      U
  //   L  F  R  B  [reposition]
  //      D
  // -----------------------------------------------------------
  { "y_plus", "", not_inverted, "" },
};

static const int k_num_color_map_steps =
  sizeof(k_color_map_steps) / sizeof(k_color_map_steps[0]);

// ============================================================
// One step processor
// ============================================================
bool CubeColorReader::process_step_(int step_index,
                                    const char *robot_move,
                                    const char *face,
                                    bool mirrored,
                                    const char *order) {

  // Robot move (if any)
  if (robot_move && robot_move[0] != '\0' && strcmp(robot_move, "none") != 0) {
    if (!ori_.robot_move(robot_move)) {
      serial_printf("err step %d: robot move %s failed\n",
                    step_index, robot_move);
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
    char color = cb_ ? cb_(slot) : '.';

    apply_slot_to_face_(F, slot, color, mirrored);

    // log single read
    serial_printf_verbose("   ---%d %c%d c=%c\n",
                          step_index,
                          F,
                          slot,
                          color);
    print_face_compact(F);
  }

  // log completion of this step for this face
  serial_printf("[step %d] move=%s\n",
                step_index,
                robot_move ? robot_move : "",
                F);
  print_face_compact(F);
  serial_printf("COLORSREAD cube_colors=%s\n", get_cube_colors_string().c_str());

  return true;
}

// ============================================================
// Perform full scan
// ============================================================
bool CubeColorReader::read_full_cube() {
  if (!cb_) {
    serial_printf("ERR color reader: no callback\n");
    return false;
  }

  fill_unknown_();

  serial_printf("color reader start - orientation restore\n");
  // Ensure starting orientation
  if (!ori_.restore_cube_orientation()) {
    serial_printf("ERR color reader: initial restore failed\n");
    return false;
  }

  for (int i = 0; i < k_num_color_map_steps; i++) {
    const auto &s = k_color_map_steps[i];
    if (!process_step_(i, s.robot_move, s.face, s.mirrored, s.order)) {
      serial_printf("ERR color reader: step %d failed\n", i);
      ori_.restore_cube_orientation();
      return false;
    }
    serial_printf("[step %d] completed\n", i);
    ori.print_orientation_string();
    print_cube_colors_string();
  }
  serial_printf("color read completed\n");
  serial_printf("color reader end - orientation restore\n");

  // Final restore
  if (!ori_.restore_cube_orientation()) {
    serial_printf("ERR color reader: final restore failed\n");
    return false;
  }
  ori.print_orientation_string();
  print_cube_colors_string();

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

void CubeColorReader::print_cube_colors_string() {
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

void CubeColorReader::fill_solved_cube() {
  static const char solved[54] = {
    // U: white
    'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W',
    // R: red
    'R', 'R', 'R', 'R', 'R', 'R', 'R', 'R', 'R',
    // F: green
    'G', 'G', 'G', 'G', 'G', 'G', 'G', 'G', 'G',
    // D: yellow
    'Y', 'Y', 'Y', 'Y', 'Y', 'Y', 'Y', 'Y', 'Y',
    // L: orange
    'O', 'O', 'O', 'O', 'O', 'O', 'O', 'O', 'O',
    // B: blue
    'B', 'B', 'B', 'B', 'B', 'B', 'B', 'B', 'B'
  };

  for (int i = 0; i < 54; i++)
    colors_[i] = solved[i];
}

// ------------------------------------------------------------
// Predefined solved cube (URFDLB orientation)
// ------------------------------------------------------------
static const char SOLVED_54[55] =
  "UUUUUUUUU"   // U = white
  "RRRRRRRRR"   // R = red
  "GGGGGGGGG"   // F = green
  "YYYYYYYYY"   // D = yellow
  "OOOOOOOOO"   // L = orange
  "BBBBBBBBB";  // B = blue

// ------------------------------------------------------------
// Helper: copy solved stickers into selected face positions
// ------------------------------------------------------------
void CubeColorReader::fill_face_from_solved_(char face, int row_start, int row_end) {
  int base = face_base_index_(face);
  int sbase = base;  // same layout in SOLVED_54

  for (int row = row_start; row <= row_end; row++) {
    for (int col = 0; col < 3; col++) {
      int idx = base + row * 3 + col;
      colors_[idx] = SOLVED_54[sbase + row * 3 + col];
    }
  }
}

// ------------------------------------------------------------
// Validate robot orientation before partial read
// Expectation: CubeOri must be identity (U=up, F=front, R=right)
// ------------------------------------------------------------
bool CubeColorReader::check_alignment_for_partial_() {
  CubeOri::Orientation ideal;
  ideal.U = 'u';
  ideal.R = 'r';
  ideal.F = 'f';
  ideal.D = 'd';
  ideal.L = 'l';
  ideal.B = 'b';

  // get current CubeOri orientation
  CubeOri::Orientation cur = ori_.get_orientation();

  if (!ori_.is_orientation_equal(ideal)) {
    serial_printf("ERR: cube misalignment detected before partial read\n");
    serial_printf("Current ori = %s\n", ori_.get_orientation_string().c_str());
    return false;
  }

  return true;
}

// ------------------------------------------------------------
// MAIN ENTRY: partial cube read
// read_bottom: read row 6,7,8 on all faces (D layer, bottom ring)
// read_mid:    read row 3,4,5 on all faces (middle layer ring)
// missing rows are filled with solved values
// ------------------------------------------------------------
bool CubeColorReader::read_partial_cube(bool read_bottom, bool read_mid) {
  fill_unknown_();

  // Detect impossible combinations (misaligned cube)
  if (!check_alignment_for_partial_()) {
    return false;
  }

  // If NOTHING requested → fill full solved cube
  if (!read_bottom && !read_mid) {
    memcpy(colors_, SOLVED_54, 54);
    serial_printf("partial read: no scan → using solved cube\n");
    return true;
  }

  // Start with solved cube as base
  memcpy(colors_, SOLVED_54, 54);

  // --------------------------------------------------------
  // PHYSICAL SCANNING
  // The robot will read only the desired layers.
  // After each face read, we place readings in correct stickers.
  // --------------------------------------------------------
  auto read_face_layer = [&](char face, bool bottom, bool mid) {
    String faceColors = "";

    // 6 stickers readable from sensor: slots 1..6
    // Order used by your cmd_read_one_face_colors:
    // Desired order = 1,2,3,6,5,4
    const int readOrder[6] = { 1, 2, 3, 6, 5, 4 };
    char tmp[7];  // tmp[1..6]

    // Perform the reading
    for (int i = 0; i < 6; i++) {
      double slot = readOrder[i];
      crrColorChar = '.';
      bool ok = cmd_read_one_color(1, &slot);
      tmp[readOrder[i]] = ok ? crrColorChar : '.';
    }

    // Convert into faceColors final 123456 ordering
    faceColors = "";
    for (int i = 1; i <= 6; i++) faceColors += tmp[i];

    // Map to cube array:
    // For mid: rows 1 & 2 (face indices 3..8 except bottom row)
    // For bottom: only row 2 (indices 6..8)
    int base = face_base_index_(face);

    if (mid) {
      // Mid layer = rows 1 and 2 (slots 4,5,6 correspond to row 1-right mapping)
      // map slot 1..3 to row 0 → IGNORE (we want solved)
      // map slot 4..6 to row 1 positions 3..5
      colors_[base + 3] = faceColors[3];
      colors_[base + 4] = faceColors[4];
      colors_[base + 5] = faceColors[5];
    }

    if (bottom) {
      // bottom layer maps slots 1,2,3 to stickers 6,7,8 of the face
      colors_[base + 6] = faceColors[0];
      colors_[base + 7] = faceColors[1];
      colors_[base + 8] = faceColors[2];
    }
  };

  // --------------------------------------------------------
  // Which faces can be scanned by your robot?
  // Typically only the FRONT face can be placed in scanning position.
  // If orientation is restored → scanning always reads FRONT.
  // --------------------------------------------------------
  char f = 'f';
  char r = 'r';
  char b = 'b';
  char l = 'l';
  char u = 'u';
  char d = 'd';

  // Robot can scan only FRONT face physically.
  // You already rotate cube using `apply_moves()` logic so other faces become front.
  auto scan_face = [&](char face) {
    // Move face to front using CubeOri (user’s existing apply_moves(cubeMove))
    // But simplest is: assume cube is already oriented physically by your k_color_map_steps.
    // Use full read_one_face_colors routine:
    int readOrder[6] = { 1, 2, 3, 6, 5, 4 };
    char facevals[9] = { 0 };

    // Actually read the front face stencil
    cmd_read_one_face_colors(0, nullptr);

    // Now faceColors is printed and stored in `faceColors` inside function.
    // But your API does not return it → WE ADD a getter!

    String fc = get_last_face_colors();  // You must add this storage in class.

    // Fill bottom / mid as appropriate
    int base = face_base_index_(face);

    if (read_mid) {
      colors_[base + 3] = fc[3];
      colors_[base + 4] = fc[4];
      colors_[base + 5] = fc[5];
    }
    if (read_bottom) {
      colors_[base + 6] = fc[0];
      colors_[base + 7] = fc[1];
      colors_[base + 8] = fc[2];
    }
  };

  // --------------------------------------------------------
  // YOUR ROBOT SHOULD NOW ROTATE cube for each face you want
  // For partial read, you only need the ring around bottom or mid layers.
  // --------------------------------------------------------

  // U face never scanned (top layer); already solved.

  // Middle layer faces: F, R, B, L
  if (read_mid) {
    // Move F to front → scan_face('f')
    scan_face('f');

    // rotate cube right
    ori_.robot_move("y_plus");
    scan_face('r');

    ori_.robot_move("y_plus");
    scan_face('b');

    ori_.robot_move("y_plus");
    scan_face('l');

    // Restore orientation
    ori_.robot_move("y_plus");
  }

  // Bottom layer: row 2 of F, R, B, L and full D face
  if (read_bottom) {
    scan_face('f');

    ori_.robot_move("y_plus");
    scan_face('r');

    ori_.robot_move("y_plus");
    scan_face('b');

    ori_.robot_move("y_plus");
    scan_face('l');

    // Restore
    ori_.robot_move("y_plus");

    // D face = rotate cube 180 on X-axis (z_180)
    ori_.robot_move("z_180");
    scan_face('d');
    ori_.robot_move("z_180");
  }

  return true;
}

CubeColorReader color_reader(ori, read_one_color_cb);
