#include "color_reader.h"
#include "utils.h"
#include "cmd_parser.h"
#include "ori.h"

extern CubeOri ori;

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

  if (offset >= 0)
    colors_[base + offset] = color;
}

// Print compact face state
void CubeColorReader::print_face_compact_(char face) const {
  int base = face_base_index_(face);
  if (base < 0) return;

  serial_printf("%c[", face);
  for (int i = 0; i < 9; i++)
    serial_printf("%c", colors_[base + i]);
  serial_printf("]");
}

// ============================================================
// adjust the color string for rotate a face and its edges
// ============================================================
void CubeColorReader::rotate_face(char face, char dir) {
  bool clockwise = (dir == '+');
  face = tolower(face);
  int base = face_base_index_(face);
  if (base < 0) return;

  // Rotate face itself
  char temp[9];
  memcpy(temp, &colors_[base], 9);
  if (clockwise) {
    colors_[base + 0] = temp[6];
    colors_[base + 1] = temp[3];
    colors_[base + 2] = temp[0];
    colors_[base + 3] = temp[7];
    colors_[base + 4] = temp[4];
    colors_[base + 5] = temp[1];
    colors_[base + 6] = temp[8];
    colors_[base + 7] = temp[5];
    colors_[base + 8] = temp[2];
  } else {
    colors_[base + 0] = temp[2];
    colors_[base + 1] = temp[5];
    colors_[base + 2] = temp[8];
    colors_[base + 3] = temp[1];
    colors_[base + 4] = temp[4];
    colors_[base + 5] = temp[7];
    colors_[base + 6] = temp[0];
    colors_[base + 7] = temp[3];
    colors_[base + 8] = temp[6];
  }

  // Rotate surrounding edge stickers
  int idx[4][3];
  char buffer[12];

  auto set_edge = [&](int a, int b, int c, int i) {
    idx[i][0] = a;
    idx[i][1] = b;
    idx[i][2] = c;
  };

  // Define edges based on face
  if (face == 'u') {
    set_edge(36, 37, 38, 0);
    set_edge(18, 19, 20, 1);
    set_edge(9, 10, 11, 2);
    set_edge(45, 46, 47, 3);
  } else if (face == 'd') {
    set_edge(24, 25, 26, 0);
    set_edge(42, 43, 44, 1);
    set_edge(51, 52, 53, 2);
    set_edge(15, 16, 17, 3);
  } else if (face == 'f') {
    set_edge(6, 7, 8, 0);
    set_edge(36 + 8, 36 + 5, 36 + 2, 1);
    set_edge(27 + 2, 27 + 1, 27 + 0, 2);
    set_edge(9 + 0, 9 + 3, 9 + 6, 3);
  } else if (face == 'b') {
    set_edge(0, 1, 2, 0);
    set_edge(9 + 8, 9 + 5, 9 + 2, 1);
    set_edge(27 + 8, 27 + 7, 27 + 6, 2);
    set_edge(36 + 0, 36 + 3, 36 + 6, 3);
  } else if (face == 'l') {
    set_edge(0, 3, 6, 0);
    set_edge(18 + 0, 18 + 3, 18 + 6, 1);
    set_edge(27 + 0, 27 + 3, 27 + 6, 2);
    set_edge(45 + 8, 45 + 5, 45 + 2, 3);
  } else if (face == 'r') {
    set_edge(2, 5, 8, 0);
    set_edge(45 + 6, 45 + 3, 45 + 0, 1);
    set_edge(27 + 2, 27 + 5, 27 + 8, 2);
    set_edge(18 + 2, 18 + 5, 18 + 8, 3);
  } else {
    return;
  }

  // Backup
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
      buffer[i * 3 + j] = colors_[idx[i][j]];

  // Apply rotation
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++) {
      int src = clockwise ? (i + 3) % 4 : (i + 1) % 4;
      colors_[idx[i][j]] = buffer[src * 3 + j];
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
      serial_printf("Invalid move: %s\n", token.c_str());
      continue;
    }

    // Token is valid → parse
    // token = "<face><dir>" where dir is '+', '-', '\'', or '2'
    char face = tolower(token[0]);
    char dir = token[1];

    rotate_face(face, dir);
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
    serial_printf("   ---%d %c%d c=%c  ",
                  step_index,
                  F,
                  slot,
                  color);
    print_face_compact_(F);
    serial_printf("\n");
  }

  // log completion of this step for this face
  serial_printf("[step %d] move=%s ",
                step_index,
                robot_move ? robot_move : "",
                F);
  print_face_compact_(F);
  serial_printf("\n");

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
    serial_printf("[step %d] completed\n");
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


CubeColorReader color_reader(ori, read_one_color_cb);
