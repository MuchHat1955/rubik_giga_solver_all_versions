#include "color_analyzer.h"

// ============================================================
// Static cubie index definitions (URFDLB, row-major)
// ============================================================

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
  {  2, 10 },  // UR
  {  7, 19 },  // UF
  {  0, 37 },  // UL
  {  1, 46 },  // UB
  { 29, 16 },  // DR
  { 34, 25 },  // DF
  { 27, 43 },  // DL
  { 28, 52 },  // DB
  { 23, 12 },  // FR
  { 21, 41 },  // FL
  { 50, 14 },  // BR
  { 48, 39 }   // BL
};

// Corners: URF, UFL, ULB, UBR, DRF, DFL, DLB, DBR
static const CornerDef k_corner_defs[8] = {
  {  2,  9, 20 },  // URF
  {  6, 18, 38 },  // UFL
  {  0, 36, 47 },  // ULB
  {  8, 45, 11 },  // UBR
  { 29, 15, 26 },  // DRF
  { 33, 24, 42 },  // DFL
  { 27, 44, 51 },  // DLB
  { 35, 17, 53 }   // DBR
};

// Adjacency lists in terms of faces
// These define which faces share each edge/corner.
static const char k_edge_adj[12][2] = {
  { 'u', 'r' }, { 'u', 'f' }, { 'u', 'l' }, { 'u', 'b' },
  { 'd', 'r' }, { 'd', 'f' }, { 'd', 'l' }, { 'd', 'b' },
  { 'f', 'r' }, { 'f', 'l' }, { 'b', 'r' }, { 'b', 'l' }
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

// ============================================================
// Constructor
// ============================================================
ColorAnalyzer::ColorAnalyzer() {
  colors_.reserve(54);
  colors_ = "......................................................"; // 54 dots
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

char ColorAnalyzer::face_of_index(int idx) const {
  if (idx < 9)   return 'u';
  if (idx < 18)  return 'r';
  if (idx < 27)  return 'f';
  if (idx < 36)  return 'd';
  if (idx < 45)  return 'l';
  return 'b';
}

// ============================================================
// Small helpers
// ============================================================
void ColorAnalyzer::sort_pair(char &a, char &b) const {
  if (a > b) {
    char t = a; a = b; b = t;
  }
}

void ColorAnalyzer::sort_triple(char &a, char &b, char &c) const {
  // simple bubble sort for 3 items
  if (a > b) { char t = a; a = b; b = t; }
  if (b > c) { char t = b; b = c; c = t; }
  if (a > b) { char t = a; a = b; b = t; }
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
const char* ColorAnalyzer::get_stage_name(int id) const {
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
// Stage detection (visual band-based, not parity-heavy)
// ============================================================

bool ColorAnalyzer::top_layer_solved_bool() const {
  // U face full solid
  if (!face_solved_bool('u')) return false;

  // Top rows of F, R, B, L match their centers
  char cf = face_center_color('f');
  char cr = face_center_color('r');
  char cb = face_center_color('b');
  char cl = face_center_color('l');

  int bf = base_index('f');
  int br = base_index('r');
  int bb = base_index('b');
  int bl = base_index('l');

  for (int i = 0; i < 3; i++) {
    if (colors_[bf + i] != cf) return false;  // F top row
    if (colors_[br + i] != cr) return false;  // R top row
    if (colors_[bb + i] != cb) return false;  // B top row
    if (colors_[bl + i] != cl) return false;  // L top row
  }
  return true;
}

bool ColorAnalyzer::middle_layer_solved_bool() const {
  if (!top_layer_solved_bool()) return false;

  char cf = face_center_color('f');
  char cr = face_center_color('r');
  char cb = face_center_color('b');
  char cl = face_center_color('l');

  int bf = base_index('f');
  int br = base_index('r');
  int bb = base_index('b');
  int bl = base_index('l');

  // Middle row, left & right stickers must match that face center
  if (colors_[bf + 3] != cf || colors_[bf + 5] != cf) return false;
  if (colors_[br + 3] != cr || colors_[br + 5] != cr) return false;
  if (colors_[bb + 3] != cb || colors_[bb + 5] != cb) return false;
  if (colors_[bl + 3] != cl || colors_[bl + 5] != cl) return false;

  return true;
}

bool ColorAnalyzer::bottom_cross_solved_bool() const {
  if (!middle_layer_solved_bool()) return false;

  int bd = base_index('d');
  char cd = face_center_color('d');

  // Cross on D: positions 1,3,5,7
  if (colors_[bd + 1] != cd) return false;
  if (colors_[bd + 3] != cd) return false;
  if (colors_[bd + 5] != cd) return false;
  if (colors_[bd + 7] != cd) return false;

  // Sides of the cross aligned (bottom middle on side faces)
  char cf = face_center_color('f');
  char cr = face_center_color('r');
  char cb = face_center_color('b');
  char cl = face_center_color('l');

  int bf = base_index('f');
  int br = base_index('r');
  int bb = base_index('b');
  int bl = base_index('l');

  // bottom-center stickers of each side
  if (colors_[bf + 7] != cf) return false;
  if (colors_[br + 7] != cr) return false;
  if (colors_[bb + 7] != cb) return false;
  if (colors_[bl + 7] != cl) return false;

  return true;
}

bool ColorAnalyzer::bottom_layer_solved_bool() const {
  if (!bottom_cross_solved_bool()) return false;

  int bd = base_index('d');
  char cd = face_center_color('d');

  // D face fully solid
  for (int i = 0; i < 9; i++) {
    if (colors_[bd + i] != cd) return false;
  }

  // bottom rows of side faces fully solid
  char cf = face_center_color('f');
  char cr = face_center_color('r');
  char cb = face_center_color('b');
  char cl = face_center_color('l');

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
      return face_solved_bool('u') &&
             face_solved_bool('r') &&
             face_solved_bool('f') &&
             face_solved_bool('d') &&
             face_solved_bool('l') &&
             face_solved_bool('b');
  }
  return false;
}

// "partial" = previous stage done, this one not yet done
bool ColorAnalyzer::is_stage_partial_bool(int id) const {
  if (id < 0 || id >= get_stage_count()) return false;
  if (is_stage_done_bool(id)) return false;
  if (id == 0) {
    // top face partial: any U sticker matches center but face not done
    int bu = base_index('u');
    char cu = face_center_color('u');
    for (int i = 0; i < 9; i++) {
      if (colors_[bu + i] == cu) return true;
    }
    return false;
  }
  // for id>0: previous stage done, this not
  return is_stage_done_bool(id - 1);
}

// ============================================================
// Validation - public wrappers
// ============================================================
bool ColorAnalyzer::is_color_string_valid_bool() const {
  return is_color_string_valid_impl(colors_);
}

// Check if already valid OR 1-sticker-change fixable.
bool ColorAnalyzer::is_string_fixable_bool() const {
  if (is_color_string_valid_bool()) return true;
  if (colors_.length() != 54) return false;

  String tmp = colors_;

  // get center colors
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

  if (s.length() != 54)
    return "Invalid length: must be 54 characters";

  if (!centers_correct_from(s))
    return "Center colors invalid (must be 6 distinct colors)";

  // inspect counts and invalid characters
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

  // check that every char is one of the 6 center colors
  for (int i = 0; i < 54; i++) {
    char ch = s[i];
    bool ok = false;
    for (int j = 0; j < 6; j++) {
      if (ch == centers[j]) { ok = true; break; }
    }
    if (!ok) {
      return String("Invalid color '") + ch + "' at index " + String(i);
    }
  }

  // check counts per color
  for (int j = 0; j < 6; j++) {
    char c = centers[j];
    if (cnt[(uint8_t)c] != 9) {
      return String("Color count invalid for '") + c +
             "': count=" + String(cnt[(uint8_t)c]) + " (expected 9)";
    }
  }

  if (!edges_corners_color_consistent_from(s))
    return "Edge/corner color combinations impossible (not a legal Rubik's Cube state)";

  return "OK";
}

// ============================================================
// Validation - core implementation
// ============================================================

bool ColorAnalyzer::is_color_string_valid_impl(const String &s) const {
  if (s.length() != 54) return false;
  if (!centers_correct_from(s)) return false;
  if (!valid_color_counts_from(s)) return false;
  if (!edges_corners_color_consistent_from(s)) return false;
  return true;
}

void ColorAnalyzer::compute_color_counts_from(const String &s, int out[256]) const {
  for (int i = 0; i < 256; i++) out[i] = 0;
  int len = s.length();
  for (int i = 0; i < len; i++) {
    out[(uint8_t)s[i]]++;
  }
}

bool ColorAnalyzer::centers_correct_from(const String &s) const {
  // 6 distinct center colors (positions 4 of each face)
  char c[6] = {
    face_center_color_from(s, 'u'),
    face_center_color_from(s, 'r'),
    face_center_color_from(s, 'f'),
    face_center_color_from(s, 'd'),
    face_center_color_from(s, 'l'),
    face_center_color_from(s, 'b')
  };

  for (int i = 0; i < 6; i++) {
    if (c[i] == '.') return false; // center cannot be unknown
    for (int j = i + 1; j < 6; j++) {
      if (c[i] == c[j]) return false;
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
  for (int i = 0; i < 54; i++) {
    char ch = s[i];
    bool ok = false;
    for (int j = 0; j < 6; j++) {
      if (ch == centers[j]) { ok = true; break; }
    }
    if (!ok) return false;
  }

  // each color must appear exactly 9 times
  for (int j = 0; j < 6; j++) {
    char c = centers[j];
    if (cnt[(uint8_t)c] != 9) return false;
  }

  return true;
}

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

  // Build list of valid corner color triples
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

  int edge_counts[12];
  int corner_counts[8];
  for (int i = 0; i < 12; i++) edge_counts[i] = 0;
  for (int i = 0; i < 8; i++) corner_counts[i] = 0;

  // Check actual edges
  for (int e = 0; e < 12; e++) {
    EdgeDef def = k_edge_defs[e];
    char a = s[def.a];
    char b = s[def.b];
    sort_pair(a, b);

    bool found = false;
    for (int k = 0; k < 12; k++) {
      if (a == edge_valid[k][0] && b == edge_valid[k][1]) {
        edge_counts[k]++;
        found = true;
        break;
      }
    }
    if (!found) return false;   // impossible color pair on an edge
  }

  // each valid edge combo must appear exactly once
  for (int k = 0; k < 12; k++) {
    if (edge_counts[k] != 1) return false;
  }

  // Check actual corners
  for (int c = 0; c < 8; c++) {
    CornerDef def = k_corner_defs[c];
    char a = s[def.a];
    char b = s[def.b];
    char d = s[def.c];
    sort_triple(a, b, d);

    bool found = false;
    for (int k = 0; k < 8; k++) {
      if (a == corner_valid[k][0] &&
          b == corner_valid[k][1] &&
          d == corner_valid[k][2]) {
        corner_counts[k]++;
        found = true;
        break;
      }
    }
    if (!found) return false;   // impossible color triple on a corner
  }

  // each valid corner combo must appear exactly once
  for (int k = 0; k < 8; k++) {
    if (corner_counts[k] != 1) return false;
  }

  return true;
}

ColorAnalyzer color_analyzer;
