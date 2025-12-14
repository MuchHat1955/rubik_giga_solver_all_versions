#include "color_analyzer.h"

extern uint32_t start_ms;

// ============================================================
// Static cubie index definitions (URFDLB, row-major per face)
// ============================================================
//
// Faces:
//   U: indices  0.. 8
//   R: indices  9..17
//   F: indices 18..26
//   D: indices 27..35
//   L: indices 36..44
//   B: indices 45..53
//
// Net layout:
//
//           U0 U1 U2
//           U3 U4 U5
//           U6 U7 U8
// L0 L1 L2  F0 F1 F2  R0 R1 R2  B0 B1 B2
// L3 L4 L5  F3 F4 F5  R3 R4 R5  B3 B4 B5
// L6 L7 L8  F6 F7 F8  R6 R7 R8  B6 B7 B8
//           D0 D1 D2
//           D3 D4 D5
//           D6 D7 D8
//
// Cubie sticker indices below are consistent with this layout.

struct EdgeDef {
  uint8_t a;
  uint8_t b;
};

struct CornerDef {
  uint8_t a;
  uint8_t b;
  uint8_t c;
};

// Edges: UR, UF, UL, UB, DR, DF, DL, DB, FR, FL, BR, BL
static const EdgeDef k_edge_defs[12] = {
  { 5, 10 },  // UR  (U mid-right, R mid-top)
  { 7, 19 },  // UF  (U mid-bottom, F mid-top)
  { 3, 39 },  // UL  (U mid-left, L mid-top)  <-- FIX
  { 1, 46 },  // UB  (U mid-top, B mid-top)

  { 32, 16 },  // DR  (D mid-right, R mid-bottom)
  { 28, 25 },  // DF  (D mid-top, F mid-bottom)
  { 30, 43 },  // DL  (D mid-left, L mid-bottom)
  { 34, 52 },  // DB  (D mid-bottom, B mid-bottom)

  { 23, 12 },  // FR  (F mid-right, R mid-left)
  { 21, 41 },  // FL  (F mid-left, L mid-right)
  { 48, 14 },  // BR  (B mid-left, R mid-right)
  { 50, 39 }   // BL  (B mid-right, L mid-left)
};

// Corners: URF, UFL, ULB, UBR, DRF, DFL, DLB, DBR
static const CornerDef k_corner_defs[8] = {
  { 8, 20, 9 },    // URF
  { 6, 18, 38 },   // UFL
  { 0, 36, 47 },   // ULB
  { 2, 45, 11 },   // UBR
  { 29, 26, 15 },  // DRF
  { 27, 24, 44 },  // DFL
  { 33, 42, 53 },  // DLB
  { 35, 17, 51 }   // DBR
};

// Face adjacency for each edge and corner
static const char k_edge_adj[12][2] = {
  { 'u', 'r' }, { 'u', 'f' }, { 'u', 'l' }, { 'u', 'b' }, { 'd', 'r' }, { 'd', 'f' }, { 'd', 'l' }, { 'd', 'b' }, { 'f', 'r' }, { 'f', 'l' }, { 'b', 'r' }, { 'b', 'l' }
};

static const char k_corner_adj[8][3] = {
  { 'u', 'r', 'f' },
  { 'u', 'f', 'l' },
  { 'u', 'l', 'b' },
  { 'u', 'b', 'r' },
  { 'd', 'f', 'r' },
  { 'd', 'l', 'f' },
  { 'd', 'b', 'l' },
  { 'd', 'r', 'b' }
};

static const char *k_edge_names[12] = {
  "UR", "UF", "UL", "UB", "DR", "DF", "DL", "DB", "FR", "FL", "BR", "BL"
};

static const char *k_corner_names[8] = {
  "URF", "UFL", "ULB", "UBR", "DRF", "DFL", "DLB", "DBR"
};

// ============================================================
// Constructor
// ============================================================
ColorAnalyzer::ColorAnalyzer() {
  colors_.reserve(54);
  colors_ = "......................................................";  // 54 dots
  last_error_ = "";
}

// ============================================================
// Index helpers
// ============================================================
int ColorAnalyzer::base_index(char face) const {
  switch (face) {
    case 'u': return 0;
    case 'r': return 9;
    case 'f': return 18;
    case 'd': return 27;
    case 'l': return 36;
    case 'b': return 45;
  }
  return 0;
}

void ColorAnalyzer::sort_pair(char &a, char &b) const {
  if (a > b) {
    char t = a;
    a = b;
    b = t;
  }
}

void ColorAnalyzer::sort_triple(char &a, char &b, char &c) const {
  if (a > b) {
    char t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    char t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    char t = a;
    a = b;
    b = t;
  }
}

// center color from an arbitrary color string
char ColorAnalyzer::face_center_color_from(const String &s, char face) const {
  int idx = base_index(face);
  if (s.length() < idx + 5) return '.';
  return s[idx + 4];  // center is always index base+4
}

// ============================================================
// Set colors
// ============================================================
void ColorAnalyzer::set_colors(const String &colors) {
  if (colors.length() == 54) {
    colors_ = colors;
  }
}

// ============================================================
// Stage names
// ============================================================
const char *ColorAnalyzer::get_stage_name(int id) const {
  switch (id) {
    case 0: return "top face";
    case 1: return "top layer";
    case 2: return "middle layer";
    case 3: return "bottom cross";
    case 4: return "bottom layer";
    case 5: return "bottom face";
    case 6: return "cube solved";
  }
  return "unknown";
}

// ============================================================
// Face solved
// ============================================================
bool ColorAnalyzer::face_solved_bool(char face) const {
  int idx = base_index(face);
  char c = colors_[idx + 4];
  for (int i = 0; i < 9; i++) {
    if (colors_[idx + i] != c) return false;
  }
  return true;
}

// ============================================================
// Stage detection (visual band-based checks)
// ============================================================
bool ColorAnalyzer::top_layer_solved_bool() const {
  // U face solid
  if (!face_solved_bool('u')) return false;

  // Top rows of F, R, B, L match their centers
  char cf = face_center_color_from(colors_, 'f');
  char cr = face_center_color_from(colors_, 'r');
  char cb = face_center_color_from(colors_, 'b');
  char cl = face_center_color_from(colors_, 'l');

  int bf = base_index('f');
  int br = base_index('r');
  int bb = base_index('b');
  int bl = base_index('l');

  for (int i = 0; i < 3; i++) {
    if (colors_[bf + i] != cf) return false;
    if (colors_[br + i] != cr) return false;
    if (colors_[bb + i] != cb) return false;
    if (colors_[bl + i] != cl) return false;
  }
  return true;
}

bool ColorAnalyzer::middle_layer_solved_bool() const {
  if (!top_layer_solved_bool()) return false;

  char cf = face_center_color_from(colors_, 'f');
  char cr = face_center_color_from(colors_, 'r');
  char cb = face_center_color_from(colors_, 'b');
  char cl = face_center_color_from(colors_, 'l');

  int bf = base_index('f');
  int br = base_index('r');
  int bb = base_index('b');
  int bl = base_index('l');

  // middle row left+right on each side
  if (colors_[bf + 3] != cf || colors_[bf + 5] != cf) return false;
  if (colors_[br + 3] != cr || colors_[br + 5] != cr) return false;
  if (colors_[bb + 3] != cb || colors_[bb + 5] != cb) return false;
  if (colors_[bl + 3] != cl || colors_[bl + 5] != cl) return false;

  return true;
}

bool ColorAnalyzer::bottom_cross_solved_bool() const {
  if (!middle_layer_solved_bool()) return false;

  int bd = base_index('d');
  char cd = face_center_color_from(colors_, 'd');

  // cross on D face: positions 1,3,5,7
  if (colors_[bd + 1] != cd) return false;
  if (colors_[bd + 3] != cd) return false;
  if (colors_[bd + 5] != cd) return false;
  if (colors_[bd + 7] != cd) return false;

  // side alignment at bottom centers
  char cf = face_center_color_from(colors_, 'f');
  char cr = face_center_color_from(colors_, 'r');
  char cb = face_center_color_from(colors_, 'b');
  char cl = face_center_color_from(colors_, 'l');

  int bf = base_index('f');
  int br = base_index('r');
  int bb = base_index('b');
  int bl = base_index('l');

  if (colors_[bf + 7] != cf) return false;
  if (colors_[br + 7] != cr) return false;
  if (colors_[bb + 7] != cb) return false;
  if (colors_[bl + 7] != cl) return false;

  return true;
}

bool ColorAnalyzer::bottom_layer_solved_bool() const {
  if (!bottom_cross_solved_bool()) return false;

  int bd = base_index('d');
  char cd = face_center_color_from(colors_, 'd');

  // D face solid
  for (int i = 0; i < 9; i++) {
    if (colors_[bd + i] != cd) return false;
  }

  // bottom rows of side faces solid
  char cf = face_center_color_from(colors_, 'f');
  char cr = face_center_color_from(colors_, 'r');
  char cb = face_center_color_from(colors_, 'b');
  char cl = face_center_color_from(colors_, 'l');

  int bf = base_index('f');
  int br = base_index('r');
  int bb = base_index('b');
  int bl = base_index('l');

  for (int i = 6; i < 9; i++) {
    if (colors_[bf + i] != cf) return false;
    if (colors_[br + i] != cr) return false;
    if (colors_[bb + i] != cb) return false;
    if (colors_[bl + i] != cl) return false;
  }

  return true;
}

// ============================================================
// Stage query
// ============================================================
bool ColorAnalyzer::is_stage_done_bool(int id) const {
  switch (id) {
    case 0: return face_solved_bool('u');
    case 1: return top_layer_solved_bool();
    case 2: return middle_layer_solved_bool();
    case 3: return bottom_cross_solved_bool();
    case 4: return bottom_layer_solved_bool();
    case 5: return face_solved_bool('d');
    case 6:
      return face_solved_bool('u') && face_solved_bool('r') && face_solved_bool('f') && face_solved_bool('d') && face_solved_bool('l') && face_solved_bool('b');
  }
  return false;
}

// "partial" = previous stage done, this stage not done yet
bool ColorAnalyzer::is_stage_partial_bool(int id) const {
  if (id < 0 || id >= get_stage_count()) return false;
  if (is_stage_done_bool(id)) return false;

  if (id == 0) {
    // top face partial: any U sticker matches center, but face not solved
    int bu = base_index('u');
    char cu = face_center_color_from(colors_, 'u');
    for (int i = 0; i < 9; i++) {
      if (colors_[bu + i] == cu) return true;
    }
    return false;
  }

  // for id>0: previous stage done, this one not yet
  return is_stage_done_bool(id - 1);
}

// ============================================================
// Validation - public wrappers
// ============================================================
bool ColorAnalyzer::is_color_string_valid_bool() const {
  last_error_ = "";
  return is_color_string_valid_impl(colors_);
}

// Already valid OR 1-sticker-change fixable?
bool ColorAnalyzer::is_string_fixable_bool() const {
  if (is_color_string_valid_bool()) return true;
  if (colors_.length() != 54) return false;

  String tmp = colors_;

  char centers[6] = {
    face_center_color_from(colors_, 'u'),
    face_center_color_from(colors_, 'r'),
    face_center_color_from(colors_, 'f'),
    face_center_color_from(colors_, 'd'),
    face_center_color_from(colors_, 'l'),
    face_center_color_from(colors_, 'b')
  };

  for (int i = 0; i < 54; i++) {
    char orig = tmp[i];
    for (int k = 0; k < 6; k++) {
      char c = centers[k];
      if (c == orig) continue;
      tmp[i] = c;
      last_error_ = "";
      if (is_color_string_valid_impl(tmp)) {
        return true;
      }
    }
    tmp[i] = orig;
  }
  return false;
}

bool ColorAnalyzer::try_fix_color_string(String &fixed_out) const {
  if (is_color_string_valid_bool()) {
    fixed_out = colors_;
    return true;
  }
  if (colors_.length() != 54) return false;

  String tmp = colors_;

  char centers[6] = {
    face_center_color_from(colors_, 'u'),
    face_center_color_from(colors_, 'r'),
    face_center_color_from(colors_, 'f'),
    face_center_color_from(colors_, 'd'),
    face_center_color_from(colors_, 'l'),
    face_center_color_from(colors_, 'b')
  };

  for (int i = 0; i < 54; i++) {
    char orig = tmp[i];
    for (int k = 0; k < 6; k++) {
      char c = centers[k];
      if (c == orig) continue;
      tmp[i] = c;
      last_error_ = "";
      if (is_color_string_valid_impl(tmp)) {
        fixed_out = tmp;
        return true;
      }
    }
    tmp[i] = orig;
  }
  return false;
}

// ============================================================
// Validation - diagnostics
// ============================================================
String ColorAnalyzer::get_string_check_log() const {
  const String &s = colors_;
  last_error_ = "";

  if (s.length() != 54) {
    last_error_ = "invalid length";
    return "invalid length: must be 54 characters";
  }

  if (!centers_correct_from(s)) {
    if (last_error_.length() == 0)
      last_error_ = "center colors invalid (must be 6 distinct colors)";
    return last_error_;
  }

  if (!valid_color_counts_from(s)) {
    if (last_error_.length() == 0)
      last_error_ = "color counts invalid";
    return last_error_;
  }

  if (!edges_corners_color_consistent_from(s)) {
    // last_error_ already set inside edges_corners_color_consistent_from
    if (last_error_.length() > 0) return last_error_;
    return "edge/corner color combinations impossible (not a legal Rubik's Cube state)";
  }

  if (!check_edge_flip_parity_simplified(s)) {
    if (last_error_.length() > 0) return last_error_;
    return "edge flip parity invalid (odd number of flipped edges)";
  }

  last_error_ = "";
  return "OK";
}

// ============================================================
// Validation - core implementation
// ============================================================
bool ColorAnalyzer::is_color_string_valid_impl(const String &s) const {
  if (s.length() != 54) {
    last_error_ = "invalid length";
    return false;
  }

  // 1. Centers and counts
  if (!centers_correct_from(s)) {
    if (last_error_.length() == 0)
      last_error_ = "center colors invalid";
    return false;
  }

  if (!valid_color_counts_from(s)) {
    // valid_color_counts_from sets last_error_
    if (last_error_.length() == 0)
      last_error_ = "color counts invalid";
    return false;
  }

  // 2. Piece composition
  if (!edges_corners_color_consistent_from(s)) {
    // last_error_ set there
    if (last_error_.length() == 0)
      last_error_ = "edge/corner color combinations impossible (not a legal Rubik's Cube state)";
    return false;
  }

  // 3. Edge flip parity
  if (!check_edge_flip_parity_simplified(s)) {
    // last_error_ set there
    if (last_error_.length() == 0)
      last_error_ = "edge flip parity invalid";
    return false;
  }

  last_error_ = "";
  return true;
}

// ============================================================
// Color / center helpers
// ============================================================
void ColorAnalyzer::compute_color_counts_from(const String &s, int out[256]) const {
  for (int i = 0; i < 256; i++) out[i] = 0;
  int len = s.length();
  for (int i = 0; i < len; i++) {
    out[(uint8_t)s[i]]++;
  }
}

bool ColorAnalyzer::centers_correct_from(const String &s) const {
  char c[6] = {
    face_center_color_from(s, 'u'),
    face_center_color_from(s, 'r'),
    face_center_color_from(s, 'f'),
    face_center_color_from(s, 'd'),
    face_center_color_from(s, 'l'),
    face_center_color_from(s, 'b')
  };

  const char face_names[6] = { 'U', 'R', 'F', 'D', 'L', 'B' };

  for (int i = 0; i < 6; i++) {
    if (c[i] == '.') {
      last_error_ = String("center color not detected for face ") + face_names[i];
      return false;  // center cannot be unknown
    }
    for (int j = i + 1; j < 6; j++) {
      if (c[i] == c[j]) {
        last_error_ = String("center color '") + c[i] + "' appears on multiple faces";
        return false;  // must be 6 distinct colors
      }
    }
  }
  return true;
}

bool ColorAnalyzer::valid_color_counts_from(const String &s) const {
  int cnt[256];
  compute_color_counts_from(s, cnt);

  char centers[6] = {
    face_center_color_from(s, 'u'),
    face_center_color_from(s, 'r'),
    face_center_color_from(s, 'f'),
    face_center_color_from(s, 'd'),
    face_center_color_from(s, 'l'),
    face_center_color_from(s, 'b')
  };

  // every sticker must be one of the 6 center colors
  int len = s.length();
  for (int i = 0; i < len; i++) {
    char ch = s[i];
    bool ok = false;
    for (int j = 0; j < 6; j++) {
      if (ch == centers[j]) {
        ok = true;
        break;
      }
    }
    if (!ok) {
      last_error_ = String("invalid color '") + ch + "' at index " + String(i) + " (not equal to any center color)";
      return false;
    }
  }

  // each color must appear exactly 9 times
  for (int j = 0; j < 6; j++) {
    char c = centers[j];
    int count = cnt[(uint8_t)c];
    if (count != 9) {
      last_error_ = String("color count invalid for '") + c + "': count=" + String(count) + " (expected 9)";
      return false;
    }
  }

  return true;
}

// ============================================================
// Edge / corner composition check
// ============================================================
bool ColorAnalyzer::edges_corners_color_consistent_from(const String &s) const {
  // Build list of valid edge color pairs from face centers
  char edge_valid[12][2];
  for (int i = 0; i < 12; i++) {
    char f1 = k_edge_adj[i][0];
    char f2 = k_edge_adj[i][1];
    char c1 = face_center_color_from(s, f1);
    char c2 = face_center_color_from(s, f2);
    edge_valid[i][0] = c1;
    edge_valid[i][1] = c2;
    sort_pair(edge_valid[i][0], edge_valid[i][1]);
  }

  // Build list of valid corner color triples from face centers
  char corner_valid[8][3];
  for (int i = 0; i < 8; i++) {
    char f1 = k_corner_adj[i][0];
    char f2 = k_corner_adj[i][1];
    char f3 = k_corner_adj[i][2];
    corner_valid[i][0] = face_center_color_from(s, f1);
    corner_valid[i][1] = face_center_color_from(s, f2);
    corner_valid[i][2] = face_center_color_from(s, f3);
    sort_triple(corner_valid[i][0], corner_valid[i][1], corner_valid[i][2]);
  }

  int edge_counts[12] = { 0 };
  int corner_counts[8] = { 0 };

  // Check actual edges
  for (int e = 0; e < 12; e++) {
    EdgeDef def = k_edge_defs[e];
    char sa = s[def.a];
    char sb = s[def.b];

    char a = sa, b = sb;
    sort_pair(a, b);

    bool found = false;
    for (int k = 0; k < 12; k++) {
      if (a == edge_valid[k][0] && b == edge_valid[k][1]) {
        edge_counts[k]++;
        found = true;
        break;
      }
    }

    if (!found) {
      char ef1 = face_center_color_from(s, k_edge_adj[e][0]);
      char ef2 = face_center_color_from(s, k_edge_adj[e][1]);

      char expected_a = ef1, expected_b = ef2;
      sort_pair(expected_a, expected_b);

      char fa = sa, fb = sb;
      sort_pair(fa, fb);

      // compute missing + extra
      String missing = "";
      String extra = "";

      // expected colors
      if (fa != expected_a && fb != expected_a) missing += expected_a;
      if (fa != expected_b && fb != expected_b) missing += expected_b;

      if (fa != expected_a && fa != expected_b) extra += fa;
      if (fb != expected_a && fb != expected_b && fb != fa) extra += fb;

      last_error_ =
        String("Edge ") + k_edge_names[e] + " invalid:\n" + "  found:    " + String(sa) + String(sb) + "\n" + "  expected: " + String(expected_a) + String(expected_b) + "\n" + "  missing:  " + (missing.length() ? missing : String("none")) + "\n" + "  extra:    " + (extra.length() ? extra : String("none"));

      return false;
    }
  }

  // each valid edge combo must appear exactly once
  for (int k = 0; k < 12; k++) {
    if (edge_counts[k] != 1) {
      last_error_ =
        String("Edge color-pair ") + edge_valid[k][0] + edge_valid[k][1] + " appears " + edge_counts[k] + " times (expected 1).";
      return false;
    }
  }

  // Check actual corners
  for (int c = 0; c < 8; c++) {
    CornerDef def = k_corner_defs[c];
    char sa = s[def.a];
    char sb = s[def.b];
    char sd = s[def.c];

    char a = sa, b = sb, d = sd;
    sort_triple(a, b, d);

    bool found = false;
    for (int k = 0; k < 8; k++) {
      if (a == corner_valid[k][0] && b == corner_valid[k][1] && d == corner_valid[k][2]) {
        corner_counts[k]++;
        found = true;
        break;
      }
    }

    if (!found) {
      char ca = face_center_color_from(s, k_corner_adj[c][0]);
      char cb = face_center_color_from(s, k_corner_adj[c][1]);
      char cc = face_center_color_from(s, k_corner_adj[c][2]);

      char expA = ca, expB = cb, expC = cc;
      sort_triple(expA, expB, expC);

      char sortedA = sa, sortedB = sb, sortedC = sd;
      sort_triple(sortedA, sortedB, sortedC);

      auto in_expected = [&](char x) {
        return (x == expA || x == expB || x == expC);
      };

      String missing = "";
      String extra = "";

      // missing expected colors
      if (!((sortedA == expA) || (sortedB == expA) || (sortedC == expA)))
        missing += expA;
      if (!((sortedA == expB) || (sortedB == expB) || (sortedC == expB)))
        missing += expB;
      if (!((sortedA == expC) || (sortedB == expC) || (sortedC == expC)))
        missing += expC;

      // extra colors: any found not in expected set
      if (!in_expected(sortedA)) extra += sortedA;
      if (!in_expected(sortedB) && sortedB != sortedA) extra += sortedB;
      if (!in_expected(sortedC) && sortedC != sortedA && sortedC != sortedB)
        extra += sortedC;

      last_error_ =
        String("Corner ") + k_corner_names[c] + " invalid:\n" + "  found:    " + String(sa) + String(sb) + String(sd) + "\n" + "  expected: {" + String(expA) + "," + expB + "," + expC + "}\n" + "  missing:  " + (missing.length() ? missing : String("none")) + "\n" + "  extra:    " + (extra.length() ? extra : String("none"));

      return false;
    }
  }

  // each valid corner combo must appear exactly once
  for (int k = 0; k < 8; k++) {
    if (corner_counts[k] != 1) {
      last_error_ =
        String("Corner color-triple ") + corner_valid[k][0] + corner_valid[k][1] + corner_valid[k][2] + " appears " + corner_counts[k] + " times (expected 1).";
      return false;
    }
  }

  return true;
}

// ============================================================
// Edge flip parity (solvability) check
// ============================================================
char ColorAnalyzer::get_edge_reference_color(const String &s, int piece_id) const {
  // 0-3 (UR, UF, UL, UB) use U-face color as reference.
  if (piece_id < 4) return face_center_color_from(s, 'u');
  // 4-7 (DR, DF, DL, DB) use D-face color as reference.
  if (piece_id < 8) return face_center_color_from(s, 'd');
  // 8-11 (FR, FL, BR, BL) use the first face in the adjacency list as reference.
  return face_center_color_from(s, k_edge_adj[piece_id][0]);
}

bool ColorAnalyzer::check_edge_flip_parity_simplified(const String &s) const {
  int flip_sum = 0;

  for (int loc = 0; loc < 12; ++loc) {
    const EdgeDef &def = k_edge_defs[loc];
    char found_color_a = s[def.a];
    char found_color_b = s[def.b];

    // 1. Identify the piece type (Piece ID 0-11)
    int piece_id = -1;
    for (int k = 0; k < 12; ++k) {
      char target_c1 = face_center_color_from(s, k_edge_adj[k][0]);
      char target_c2 = face_center_color_from(s, k_edge_adj[k][1]);

      if ((found_color_a == target_c1 && found_color_b == target_c2) || (found_color_a == target_c2 && found_color_b == target_c1)) {
        piece_id = k;
        break;
      }
    }

    if (piece_id == -1) {
      // Should not happen if previous checks passed, but acts as a safety log.
      last_error_ = String("internal error: edge composition check failed at ") + k_edge_names[loc];
      return false;
    }

    // 2. Determine Orientation (Flip)
    char ref_color = get_edge_reference_color(s, piece_id);

    // Edge definitions are set up so 'a' is the reference facelet (U/D for most).
    // Flip is 1 if the reference color is NOT on the reference facelet (def.a).
    int flip = (s[def.a] != ref_color) ? 1 : 0;
    flip_sum += flip;
  }

  if (flip_sum % 2 != 0) {
    last_error_ =
      String("solvability error (edge flip): total number of flipped edges is odd (") + String(flip_sum) + "). cube is UNSOLVABLE (requires an even number of flipped edges).";
    return false;
  }
  return true;
}

// ============================================================
// Global instance
// ============================================================
ColorAnalyzer color_analyzer;
