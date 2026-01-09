#include "color_analyzer.h"
#include "utils.h"
#include "log.h"
#include "ori.h"

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
  colors_standard_orientation_54_.reserve(54);
  colors_standard_orientation_54_ = "......................................................";  // 54 dots
  last_error_ = "";
}

void ColorAnalyzer::clear_color_analyzer() {
  colors_standard_orientation_54_ = "......................................................";
}

// ============================================================
// Set colors
// ============================================================
bool ColorAnalyzer::set_colors(const String &colors) {
  if (colors.length() != 54) return false;

  String normalized = normalize_cube_colors_to_standard_simple(colors);
  if (normalized.length() != 54) return false;

  colors_standard_orientation_54_ = normalized;
  colors_standard_orientation_54_.toUpperCase();
  return true;
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
  int idx = face_base_index(face);
  char c = colors_standard_orientation_54_[idx + 4];
  for (int i = 0; i < 9; i++) {
    if (colors_standard_orientation_54_[idx + i] != c) return false;
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
  char cf = face_center_color_from(colors_standard_orientation_54_, 'f');
  char cr = face_center_color_from(colors_standard_orientation_54_, 'r');
  char cb = face_center_color_from(colors_standard_orientation_54_, 'b');
  char cl = face_center_color_from(colors_standard_orientation_54_, 'l');

  int bf = face_base_index('f');
  int br = face_base_index('r');
  int bb = face_base_index('b');
  int bl = face_base_index('l');

  for (int i = 0; i < 3; i++) {
    if (colors_standard_orientation_54_[bf + i] != cf) return false;
    if (colors_standard_orientation_54_[br + i] != cr) return false;
    if (colors_standard_orientation_54_[bb + i] != cb) return false;
    if (colors_standard_orientation_54_[bl + i] != cl) return false;
  }
  return true;
}

bool ColorAnalyzer::middle_layer_solved_bool() const {
  if (!top_layer_solved_bool()) return false;

  char cf = face_center_color_from(colors_standard_orientation_54_, 'f');
  char cr = face_center_color_from(colors_standard_orientation_54_, 'r');
  char cb = face_center_color_from(colors_standard_orientation_54_, 'b');
  char cl = face_center_color_from(colors_standard_orientation_54_, 'l');

  int bf = face_base_index('f');
  int br = face_base_index('r');
  int bb = face_base_index('b');
  int bl = face_base_index('l');

  // middle row left+right on each side
  if (colors_standard_orientation_54_[bf + 3] != cf || colors_standard_orientation_54_[bf + 5] != cf) return false;
  if (colors_standard_orientation_54_[br + 3] != cr || colors_standard_orientation_54_[br + 5] != cr) return false;
  if (colors_standard_orientation_54_[bb + 3] != cb || colors_standard_orientation_54_[bb + 5] != cb) return false;
  if (colors_standard_orientation_54_[bl + 3] != cl || colors_standard_orientation_54_[bl + 5] != cl) return false;

  return true;
}

bool ColorAnalyzer::bottom_cross_solved_bool() const {
  if (!middle_layer_solved_bool()) return false;

  int bd = face_base_index('d');
  char cd = face_center_color_from(colors_standard_orientation_54_, 'd');

  // cross on D face: positions 1,3,5,7
  if (colors_standard_orientation_54_[bd + 1] != cd) return false;
  if (colors_standard_orientation_54_[bd + 3] != cd) return false;
  if (colors_standard_orientation_54_[bd + 5] != cd) return false;
  if (colors_standard_orientation_54_[bd + 7] != cd) return false;

  // side alignment at bottom centers
  char cf = face_center_color_from(colors_standard_orientation_54_, 'f');
  char cr = face_center_color_from(colors_standard_orientation_54_, 'r');
  char cb = face_center_color_from(colors_standard_orientation_54_, 'b');
  char cl = face_center_color_from(colors_standard_orientation_54_, 'l');

  int bf = face_base_index('f');
  int br = face_base_index('r');
  int bb = face_base_index('b');
  int bl = face_base_index('l');

  if (colors_standard_orientation_54_[bf + 7] != cf) return false;
  if (colors_standard_orientation_54_[br + 7] != cr) return false;
  if (colors_standard_orientation_54_[bb + 7] != cb) return false;
  if (colors_standard_orientation_54_[bl + 7] != cl) return false;

  return true;
}

bool ColorAnalyzer::bottom_layer_solved_bool() const {
  if (!bottom_cross_solved_bool()) return false;

  int bd = face_base_index('d');
  char cd = face_center_color_from(colors_standard_orientation_54_, 'd');

  // D face solid
  for (int i = 0; i < 9; i++) {
    if (colors_standard_orientation_54_[bd + i] != cd) return false;
  }

  // bottom rows of side faces solid
  char cf = face_center_color_from(colors_standard_orientation_54_, 'f');
  char cr = face_center_color_from(colors_standard_orientation_54_, 'r');
  char cb = face_center_color_from(colors_standard_orientation_54_, 'b');
  char cl = face_center_color_from(colors_standard_orientation_54_, 'l');

  int bf = face_base_index('f');
  int br = face_base_index('r');
  int bb = face_base_index('b');
  int bl = face_base_index('l');

  for (int i = 6; i < 9; i++) {
    if (colors_standard_orientation_54_[bf + i] != cf) return false;
    if (colors_standard_orientation_54_[br + i] != cr) return false;
    if (colors_standard_orientation_54_[bb + i] != cb) return false;
    if (colors_standard_orientation_54_[bl + i] != cl) return false;
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
    int bu = face_base_index('u');
    char cu = face_center_color_from(colors_standard_orientation_54_, 'u');
    for (int i = 0; i < 9; i++) {
      if (colors_standard_orientation_54_[bu + i] == cu) return true;
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
  return is_valid_color_string_54_impl(colors_standard_orientation_54_);
}

// Already valid OR 1-sticker-change fixable?
bool ColorAnalyzer::is_string_fixable_bool() const {
  if (is_color_string_valid_bool()) return true;
  if (colors_standard_orientation_54_.length() != 54) return false;

  String tmp = colors_standard_orientation_54_;

  char centers[6] = {
    face_center_color_from(colors_standard_orientation_54_, 'u'),
    face_center_color_from(colors_standard_orientation_54_, 'r'),
    face_center_color_from(colors_standard_orientation_54_, 'f'),
    face_center_color_from(colors_standard_orientation_54_, 'd'),
    face_center_color_from(colors_standard_orientation_54_, 'l'),
    face_center_color_from(colors_standard_orientation_54_, 'b')
  };

  for (int i = 0; i < 54; i++) {
    char orig = tmp[i];
    for (int k = 0; k < 6; k++) {
      char c = centers[k];
      if (c == orig) continue;
      tmp[i] = c;
      last_error_ = "";
      if (is_valid_color_string_54_impl(tmp)) {
        return true;
      }
    }
    tmp[i] = orig;
  }
  return false;
}

bool ColorAnalyzer::try_fix_color_string(String &fixed_out) const {
  if (is_color_string_valid_bool()) {
    fixed_out = colors_standard_orientation_54_;
    return true;
  }
  if (colors_standard_orientation_54_.length() != 54) return false;

  String tmp = colors_standard_orientation_54_;

  char centers[6] = {
    face_center_color_from(colors_standard_orientation_54_, 'u'),
    face_center_color_from(colors_standard_orientation_54_, 'r'),
    face_center_color_from(colors_standard_orientation_54_, 'f'),
    face_center_color_from(colors_standard_orientation_54_, 'd'),
    face_center_color_from(colors_standard_orientation_54_, 'l'),
    face_center_color_from(colors_standard_orientation_54_, 'b')
  };

  for (int i = 0; i < 54; i++) {
    char orig = tmp[i];
    for (int k = 0; k < 6; k++) {
      char c = centers[k];
      if (c == orig) continue;
      tmp[i] = c;
      last_error_ = "";
      if (is_valid_color_string_54_impl(tmp)) {
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
  const String &s = colors_standard_orientation_54_;
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
bool ColorAnalyzer::is_valid_color_string_54_impl(const String &s) const {
  if (s.length() != 54) {
    last_error_ = "invalid length";
    LOG_ERR(MOD_COLORCHECK, "invalid color string", last_error_);
    return false;
  }

  // 1. Centers and counts
  if (!centers_correct_from(s)) {
    if (last_error_.length() == 0) {
      last_error_ = "center colors invalid";
      LOG_ERR(MOD_COLORCHECK, "invalid color string", last_error_);
    }
    return false;
  }

  if (!valid_color_counts_from(s)) {
    // valid_color_counts_from sets last_error_
    if (last_error_.length() == 0) {
      last_error_ = "color counts invalid";
      LOG_ERR(MOD_COLORCHECK, "invalid color string", last_error_);
    }
    return false;
  }

  // 2. Piece composition
  if (!edges_corners_color_consistent_from(s)) {
    // last_error_ set there
    if (last_error_.length() == 0) {
      last_error_ = "edge/corner color combinations impossible (not a legal cube state)";
      LOG_ERR(MOD_COLORCHECK, "invalid color string", last_error_);
    }
    return false;
  }

  // 3. Edge flip parity
  if (!check_edge_flip_parity_simplified(s)) {
    // last_error_ set there
    if (last_error_.length() == 0) {
      last_error_ = "edge flip parity invalid";
      LOG_ERR(MOD_COLORCHECK, "invalid color string", last_error_);
    }
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

String ColorAnalyzer::fix_string_unknown_3() const {
  if (colors_standard_orientation_54_.length() != 54) return "";

  int cnt[256];
  int unk = 0;
  count_colors_and_unknowns(colors_standard_orientation_54_, cnt, unk);

  if (unk != 3) return "";

  char missing[3];
  int m = 0;

  for (int i = 0; i < 256 && m < 3; i++) {
    if (cnt[i] == 8) {
      missing[m++] = (char)i;
    }
  }

  if (m == 0) return "";

  int idx[3], k = 0;
  for (int i = 0; i < 54; i++) {
    if (colors_standard_orientation_54_[i] == '.') idx[k++] = i;
  }

  String tmp = colors_standard_orientation_54_;

  // Try all reasonable assignments (bounded)
  for (int a = 0; a < m; a++) {
    for (int b = 0; b < m; b++) {
      for (int c = 0; c < m; c++) {
        tmp[idx[0]] = missing[a];
        tmp[idx[1]] = missing[b];
        tmp[idx[2]] = missing[c];

        if (is_valid_color_string_54_impl(tmp)) {
          return tmp;
        }
      }
    }
  }

  return "";
}

String ColorAnalyzer::fix_string_unknown_2() const {
  if (colors_standard_orientation_54_.length() != 54) return "";

  int cnt[256];
  int unk = 0;
  count_colors_and_unknowns(colors_standard_orientation_54_, cnt, unk);

  if (unk != 2) return "";

  char missing[2];
  int m = 0;

  for (int i = 0; i < 256; i++) {
    if (cnt[i] < 9 && m < 2) {
      missing[m++] = (char)i;
    }
  }

  if (m == 0) return "";

  int idx[2], k = 0;
  for (int i = 0; i < 54; i++) {
    if (colors_standard_orientation_54_[i] == '.') idx[k++] = i;
  }

  String tmp = colors_standard_orientation_54_;

  // Case 1: same color twice
  if (m == 1 || cnt[(uint8_t)missing[0]] == 7) {
    tmp[idx[0]] = missing[0];
    tmp[idx[1]] = missing[0];

    if (is_valid_color_string_54_impl(tmp)) {
      return tmp;
    }
  }

  // Case 2: two different colors
  if (m == 2) {
    tmp[idx[0]] = missing[0];
    tmp[idx[1]] = missing[1];
    if (is_valid_color_string_54_impl(tmp)) return tmp;

    tmp[idx[0]] = missing[1];
    tmp[idx[1]] = missing[0];
    if (is_valid_color_string_54_impl(tmp)) return tmp;
  }

  return "";
}

String ColorAnalyzer::fix_string_unknown_1() const {
  if (colors_standard_orientation_54_.length() != 54) return "";

  int cnt[256];
  int unk = 0;
  count_colors_and_unknowns(colors_standard_orientation_54_, cnt, unk);

  if (unk != 1) return "";

  char missing = 0;
  for (int i = 0; i < 256; i++) {
    if (cnt[i] == 8) {
      missing = (char)i;
      break;
    }
  }

  if (!missing) return "";

  String tmp = colors_standard_orientation_54_;
  for (int i = 0; i < 54; i++) {
    if (tmp[i] == '.') {
      tmp[i] = missing;
      break;
    }
  }

  if (is_valid_color_string_54_impl(tmp)) {
    return tmp;
  }

  return "";
}

void ColorAnalyzer::count_colors_and_unknowns(const String &s, int counts[256], int &unknown_count) {
  memset(counts, 0, 256 * sizeof(int));
  unknown_count = 0;

  for (int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '.') {
      unknown_count++;
    } else {
      counts[(uint8_t)c]++;
    }
  }
}

String ColorAnalyzer::fix_string_count_2() const {
  if (is_color_string_valid_bool()) {
    return colors_standard_orientation_54_;
  }
  if (colors_standard_orientation_54_.length() != 54) {
    return "";
  }

  int cnt[256];
  count_colors(colors_standard_orientation_54_, cnt);

  char too_many = 0;
  char too_few = 0;
  int delta = 0;

  for (int i = 0; i < 256; i++) {
    if (cnt[i] > 9) {
      too_many = (char)i;
      delta = cnt[i] - 9;
    }
    if (cnt[i] < 9) {
      too_few = (char)i;
    }
  }

  if (!too_many || !too_few || delta < 2) {
    return "";
  }

  String tmp = colors_standard_orientation_54_;

  for (int i = 0; i < 54; i++) {
    if (tmp[i] != too_many) continue;

    for (int j = i + 1; j < 54; j++) {
      if (tmp[j] != too_many) continue;

      tmp[i] = too_few;
      tmp[j] = too_few;

      if (is_valid_color_string_54_impl(tmp)) {
        return tmp;
      }

      tmp[i] = too_many;
      tmp[j] = too_many;
    }
  }

  return "";
}

String ColorAnalyzer::fix_string_count_1() const {
  if (is_color_string_valid_bool()) {
    return colors_standard_orientation_54_;
  }
  if (colors_standard_orientation_54_.length() != 54) {
    return "";
  }

  int cnt[256];
  count_colors(colors_standard_orientation_54_, cnt);

  char too_many = 0;
  char too_few = 0;

  for (int i = 0; i < 256; i++) {
    if (cnt[i] == 10) too_many = (char)i;
    if (cnt[i] == 8) too_few = (char)i;
  }

  if (!too_many || !too_few) {
    return "";
  }

  String tmp = colors_standard_orientation_54_;

  for (int i = 0; i < 54; i++) {
    if (tmp[i] != too_many) continue;

    tmp[i] = too_few;

    if (is_valid_color_string_54_impl(tmp)) {
      return tmp;
    }

    tmp[i] = too_many;
  }

  return "";
}

String ColorAnalyzer::fix_string_general_1() const {
  if (is_color_string_valid_bool()) {
    return colors_standard_orientation_54_;
  }
  if (colors_standard_orientation_54_.length() != 54) {
    return "";
  }

  String tmp = colors_standard_orientation_54_;

  char centers[6] = {
    face_center_color_from(colors_standard_orientation_54_, 'u'),
    face_center_color_from(colors_standard_orientation_54_, 'r'),
    face_center_color_from(colors_standard_orientation_54_, 'f'),
    face_center_color_from(colors_standard_orientation_54_, 'd'),
    face_center_color_from(colors_standard_orientation_54_, 'l'),
    face_center_color_from(colors_standard_orientation_54_, 'b')
  };

  for (int i = 0; i < 54; i++) {
    char orig = tmp[i];

    for (int k = 0; k < 6; k++) {
      char c = centers[k];
      if (c == orig) continue;

      tmp[i] = c;

      if (is_valid_color_string_54_impl(tmp)) {
        return tmp;
      }
    }

    tmp[i] = orig;
  }

  return "";
}

String ColorAnalyzer::fix_string_ro_2() const {
  if (is_color_string_valid_bool()) {
    return colors_standard_orientation_54_;
  }
  if (colors_standard_orientation_54_.length() != 54) {
    return "";
  }

  String tmp = colors_standard_orientation_54_;

  for (int i = 0; i < 54; i++) {
    char orig_i = tmp[i];
    if (orig_i != 'r' && orig_i != 'o') continue;

    for (int j = i + 1; j < 54; j++) {
      char orig_j = tmp[j];
      if (orig_j != 'r' && orig_j != 'o') continue;

      tmp[i] = (orig_i == 'r') ? 'o' : 'r';
      tmp[j] = (orig_j == 'r') ? 'o' : 'r';

      if (is_valid_color_string_54_impl(tmp)) {
        return tmp;
      }

      tmp[i] = orig_i;
      tmp[j] = orig_j;
    }
  }

  return "";
}

String ColorAnalyzer::fix_string_ro_1() const {
  if (is_color_string_valid_bool()) {
    return colors_standard_orientation_54_;
  }
  if (colors_standard_orientation_54_.length() != 54) {
    return "";
  }

  String tmp = colors_standard_orientation_54_;

  for (int i = 0; i < 54; i++) {
    char orig = tmp[i];

    if (orig == 'r') {
      tmp[i] = 'o';
    } else if (orig == 'o') {
      tmp[i] = 'r';
    } else {
      continue;
    }

    if (is_valid_color_string_54_impl(tmp)) {
      return tmp;
    }

    tmp[i] = orig;
  }

  return "";
}

void ColorAnalyzer::count_colors(const String &s, int counts[256]) {
  memset(counts, 0, 256 * sizeof(int));

  int len = s.length();
  for (int i = 0; i < len; i++) {
    char c = s[i];
    if (!is_valid_color(c)) continue;

    counts[(uint8_t)tolower(c)]++;
  }
}


String ColorAnalyzer::fix_string_smart() const {
  String s;

  s = fix_string_unknown_1();
  if (s.length() == 54) return s;

  s = fix_string_unknown_2();
  if (s.length() == 54) return s;

  s = fix_string_unknown_3();
  if (s.length() == 54) return s;

  s = fix_string_ro_1();
  if (s.length() == 54) return s;

  s = fix_string_count_1();
  if (s.length() == 54) return s;

  s = fix_string_ro_2();
  if (s.length() == 54) return s;

  s = fix_string_count_2();
  if (s.length() == 54) return s;

  s = fix_string_general_1();
  if (s.length() == 54) return s;

  return "";
}

static void rotate_face_cw(char *s, int base) {
  char t[9];
  for (int i = 0; i < 9; i++) t[i] = s[base + i];

  s[base + 0] = t[6];
  s[base + 1] = t[3];
  s[base + 2] = t[0];
  s[base + 3] = t[7];
  s[base + 4] = t[4];
  s[base + 5] = t[1];
  s[base + 6] = t[8];
  s[base + 7] = t[5];
  s[base + 8] = t[2];
}

static void apply_move_cw(char *s, char face) {
  char t[3];

  switch (face) {

    // =====================================================
    // U
    // =====================================================
    case 'U':
      rotate_face_cw(s, 0);

      t[0] = s[18];
      t[1] = s[19];
      t[2] = s[20];  // F top
      s[18] = s[36];
      s[19] = s[37];
      s[20] = s[38];  // L -> F
      s[36] = s[45];
      s[37] = s[46];
      s[38] = s[47];  // B -> L
      s[45] = s[9];
      s[46] = s[10];
      s[47] = s[11];  // R -> B
      s[9] = t[0];
      s[10] = t[1];
      s[11] = t[2];  // F -> R
      break;

    // =====================================================
    // D
    // =====================================================
    case 'D':
      rotate_face_cw(s, 27);

      t[0] = s[24];
      t[1] = s[25];
      t[2] = s[26];  // F bottom
      s[24] = s[15];
      s[25] = s[16];
      s[26] = s[17];  // R -> F
      s[15] = s[51];
      s[16] = s[52];
      s[17] = s[53];  // B -> R
      s[51] = s[42];
      s[52] = s[43];
      s[53] = s[44];  // L -> B
      s[42] = t[0];
      s[43] = t[1];
      s[44] = t[2];  // F -> L
      break;

    // =====================================================
    // F
    // =====================================================
    case 'F':
      rotate_face_cw(s, 18);

      t[0] = s[6];
      t[1] = s[7];
      t[2] = s[8];  // U bottom
      s[6] = s[38];
      s[7] = s[41];
      s[8] = s[44];  // L -> U
      s[38] = s[27];
      s[41] = s[28];
      s[44] = s[29];  // D -> L
      s[27] = s[11];
      s[28] = s[14];
      s[29] = s[17];  // R -> D
      s[11] = t[0];
      s[14] = t[1];
      s[17] = t[2];  // U -> R
      break;

    // =====================================================
    // B
    // =====================================================
    case 'B':
      rotate_face_cw(s, 45);

      t[0] = s[0];
      t[1] = s[1];
      t[2] = s[2];  // U top
      s[0] = s[9];
      s[1] = s[12];
      s[2] = s[15];  // R -> U
      s[9] = s[33];
      s[12] = s[34];
      s[15] = s[35];  // D -> R
      s[33] = s[36];
      s[34] = s[39];
      s[35] = s[42];  // L -> D
      s[36] = t[2];
      s[39] = t[1];
      s[42] = t[0];  // U -> L
      break;

    // =====================================================
    // R
    // =====================================================
    case 'R':
      rotate_face_cw(s, 9);

      t[0] = s[2];
      t[1] = s[5];
      t[2] = s[8];  // U right
      s[2] = s[20];
      s[5] = s[23];
      s[8] = s[26];  // F -> U
      s[20] = s[29];
      s[23] = s[32];
      s[26] = s[35];  // D -> F
      s[29] = s[47];
      s[32] = s[50];
      s[35] = s[53];  // B -> D
      s[47] = t[0];
      s[50] = t[1];
      s[53] = t[2];  // U -> B
      break;

    // =====================================================
    // L
    // =====================================================
    case 'L':
      rotate_face_cw(s, 36);

      t[0] = s[0];
      t[1] = s[3];
      t[2] = s[6];  // U left
      s[0] = s[45];
      s[3] = s[48];
      s[6] = s[51];  // B -> U
      s[45] = s[27];
      s[48] = s[30];
      s[51] = s[33];  // D -> B
      s[27] = s[18];
      s[30] = s[21];
      s[33] = s[24];  // F -> D
      s[18] = t[0];
      s[21] = t[1];
      s[24] = t[2];  // U -> F
      break;
  }
}

String apply_cube_move_54(const String &in, const String &move) {
  if (in.length() != 54 || move.length() == 0) return "";

  char s[55];
  in.toCharArray(s, 55);

  char face = toupper(move[0]);
  int turns = 0;

  if (move.length() > 1) {
    if (move[1] == '+') turns = 1;
    if (move[1] == '-') turns = 3;
    else if (move[1] == '2') turns = 2;
  }

  for (int i = 0; i < turns; i++)
    apply_move_cw(s, face);

  return String(s);
}

// TODO add log here or where called
bool ColorAnalyzer::apply_cube_move(String mv) {
  String after_move = apply_cube_move_54(colors_standard_orientation_54_, mv);
  if (after_move.length() == 0) return false;
  colors_standard_orientation_54_ = after_move;
  return true;
}

// slot is 1 to 9
char get_stickercolor_from_color_string_54(String color_string_54, char face, int slot) {
  if (slot < 1 || slot > 9) return '.';
  char lf = tolower(face);
  if (!is_valid_face(lf)) return '.';
  if (color_string_54.length() != 54) return '.';

  int base = -1;
  switch (lf) {
    case 'u': base = 0; break;
    case 'r': base = 9; break;
    case 'f': base = 18; break;
    case 'd': base = 27; break;
    case 'l': base = 36; break;
    case 'b': base = 45; break;
    default: return '\0';
  }

  char c = color_string_54.charAt(base + slot - 1);
  return toupper(c);
}
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

String ColorAnalyzer::get_standard_color_string_54() {
  return colors_standard_orientation_54_;
}

bool is_valid_color(char color) {
  char lc = tolower(color);
  if (lc == 'w') return true;
  if (lc == 'r') return true;
  if (lc == 'g') return true;
  if (lc == 'y') return true;
  if (lc == 'o') return true;
  if (lc == 'b') return true;
  return false;
}

// ============================================================
// Index helpers
// ============================================================
int face_base_index(char face) {
  switch (tolower(face)) {
    case 'u': return 0;
    case 'r': return 9;
    case 'f': return 18;
    case 'd': return 27;
    case 'l': return 36;
    case 'b': return 45;
  }
  return -1;
}

void sort_pair(char &a, char &b) {
  if (a > b) {
    char t = a;
    a = b;
    b = t;
  }
}

void sort_triple(char &a, char &b, char &c) {
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
char face_center_color_from(const String &s, char face) {
  int idx = face_base_index(face);
  if (idx < 0 || s.length() < idx + 5) return '.';
  return s[idx + 4];  // center is always index base+4
}

String normalize_cube_colors_to_standard_simple(const String &in54) {
  if (in54.length() != 54) return "";

  // Face bases in URFDLB
  const int base[6] = { 0, 9, 18, 27, 36, 45 };
  const char faces[6] = { 'u', 'r', 'f', 'd', 'l', 'b' };

  // ------------------------------------------------------------
  // Step 1: read center colors
  // ------------------------------------------------------------
  char center_col[6];
  bool used[256] = { false };

  for (int i = 0; i < 6; i++) {
    char c = tolower(in54[base[i] + 4]);
    if (c == '.' || !isalpha(c)) return "";
    if (used[(uint8_t)c]) return "";  // duplicate center
    used[(uint8_t)c] = true;
    center_col[i] = c;
  }

  // ------------------------------------------------------------
  // Step 2: validate opposite pairs
  // ------------------------------------------------------------
  auto opp_color = [](char c) -> char {
    switch (c) {
      case 'w': return 'y';
      case 'y': return 'w';
      case 'r': return 'o';
      case 'o': return 'r';
      case 'g': return 'b';
      case 'b': return 'g';
      default: return '\0';
    }
  };

  bool paired[6] = { false };

  for (int i = 0; i < 6; i++) {
    if (paired[i]) continue;

    char c = center_col[i];
    char o = opp_color(c);
    if (o == '\0') return "";

    bool found = false;
    for (int j = 0; j < 6; j++) {
      if (i != j && center_col[j] == o) {
        paired[i] = paired[j] = true;
        found = true;
        break;
      }
    }
    if (!found) return "";
  }

  // ------------------------------------------------------------
  // Step 3: map center colors → logical faces
  // ------------------------------------------------------------
  auto logical_face_from_color = [](char c) -> char {
    switch (c) {
      case 'w': return 'u';
      case 'y': return 'd';
      case 'r': return 'r';
      case 'o': return 'l';
      case 'g': return 'f';
      case 'b': return 'b';
      default: return '\0';
    }
  };

  int face_map[6];
  for (int i = 0; i < 6; i++) face_map[i] = -1;

  for (int i = 0; i < 6; i++) {
    char lf = logical_face_from_color(center_col[i]);
    if (lf == '\0') return "";

    for (int k = 0; k < 6; k++) {
      if (faces[k] == lf) {
        if (face_map[k] != -1) return "";
        face_map[k] = i;
        break;
      }
    }
  }

  for (int i = 0; i < 6; i++) {
    if (face_map[i] < 0) return "";
  }

  // ------------------------------------------------------------
  // Step 4: build color remap → standard colors
  // ------------------------------------------------------------
  char color_remap[256] = { 0 };

  const char std_color[6] = {
    'w',  // u
    'r',  // r
    'g',  // f
    'y',  // d
    'o',  // l
    'b'   // b
  };

  for (int i = 0; i < 6; i++) {
    char src = center_col[i];
    char lf = logical_face_from_color(src);
    for (int k = 0; k < 6; k++) {
      if (faces[k] == lf) {
        color_remap[(uint8_t)src] = std_color[k];
        break;
      }
    }
  }

  // ------------------------------------------------------------
  // Step 5: copy faces one-by-one into output
  // ------------------------------------------------------------
  String out;
  out.reserve(54);

  for (int lf = 0; lf < 6; lf++) {
    int src_face = face_map[lf];
    int src_base = base[src_face];

    for (int i = 0; i < 9; i++) {
      char c = tolower(in54[src_base + i]);
      char m = color_remap[(uint8_t)c];
      if (m == 0) return "";
      out += m;
    }
  }

  return out;
}

char get_color_from_color_string_54(String s, char face, int slot) {
  if (slot < 0 || slot > 8) return '\0';
  if (s.length() != 54) return '\0';

  char lf = tolower(face);
  if (!is_valid_face(lf)) return '\0';

  int base = -1;
  switch (lf) {
    case 'u': base = 0; break;
    case 'r': base = 9; break;
    case 'f': base = 18; break;
    case 'd': base = 27; break;
    case 'l': base = 36; break;
    case 'b': base = 45; break;
    default: return '\0';
  }
  char c = s.charAt(base + slot);
  return (c == '.' ? '\0' : tolower(c));
}

bool is_valid_color_string_54(String color) {
  if (color.length() != 54) return false;
  return color_analyzer.is_valid_color_string_54_impl(color);
}

bool is_valid_face(char face) {
  char c = tolower(face);
  return (c == 'u' || c == 'r' || c == 'f' || c == 'd' || c == 'l' || c == 'b');
}

char oposite_face(char face) {
  char c = tolower(face);
  switch (c) {
    case 'u': return 'd';
    case 'd': return 'u';
    case 'r': return 'l';
    case 'l': return 'r';
    case 'f': return 'b';
    case 'b': return 'f';
    default: return '\0';
  }
}

char oposite_color(char color) {
  char c = tolower(color);
  switch (c) {
    case 'w': return 'y';
    case 'y': return 'w';
    case 'r': return 'o';
    case 'o': return 'r';
    case 'g': return 'b';
    case 'b': return 'g';
    default: return '\0';
  }
}

String rubik_54_to_labeled_diagram(const String &s) {
  if (s.length() != 54) return "";

  auto norm = [](char c) -> char {
    c = toupper((unsigned char)c);
    switch (c) {
      case 'W':
      case 'Y':
      case 'R':
      case 'O':
      case 'G':
      case 'B':
        return c;
      default:
        return '.';
    }
  };

  auto row = [&](int base, int r) -> String {
    String o;
    for (int c = 0; c < 3; c++)
      o += norm(s[base + r * 3 + c]);
    return o;
  };

  String out;

  // -------- U --------
  out += "\n        [U]\n";
  for (int r = 0; r < 3; r++) {
    out += "        ";
    out += row(0, r);
    out += "\n";
  }

  out += "\n";

  // -------- L F R B --------
  out += "[L]     [F]     [R]     [B]\n";
  for (int r = 0; r < 3; r++) {
    out += row(36, r);
    out += "     ";
    out += row(18, r);
    out += "     ";
    out += row(9, r);
    out += "     ";
    out += row(45, r);
    out += "\n";
  }

  out += "\n";

  // -------- D --------
  out += "        [D]\n";
  for (int r = 0; r < 3; r++) {
    out += "        ";
    out += row(27, r);
    out += "\n";
  }

  return out;
}

// ============================================================
// Global instance
// ============================================================
ColorAnalyzer color_analyzer;
