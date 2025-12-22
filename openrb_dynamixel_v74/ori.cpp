#include "ori.h"
#include "utils.h"
#include "color_reader.h"
#include "color_analyzer.h"
#include "cmd_parser.h"

extern CubeColorReader color_reader;
extern ColorAnalyzer color_analyzer;

int cube_move_index = 0;
int cube_move_total = 0;

// ============================================================
// Tables
// ============================================================

// Orientation change table for robot moves.
// Interpretation of "u->r" hereu
//
//   The content of physical face U moves to physical face R.
//
// Orientation stores: for each physical face (U,R,F,D,L,B)
// what logical face is there. So applying a mapping:
//
//   new[R] = old[U];
//
// and any face not mentioned stays the same.
struct OriMoveMap {
  const char *robot_move;  // "z_plus", etc
  const char *changes[4];  // up to 4 directional changes "u->r"
};

// Cube logical -> robot move sequences.
// Here the key is a *physical* face direction (U,R,F,D,L,B) and suffix,
// but we always first convert logical-face to its current physical direction
// using the orientation state.
struct CubeToRobotEntry {
  const char *cube_move;    // e.g. "F+", "R'", "U2" (physical direction + suffix)
  const char *robot_moves;  // e.g. "z+ y+ d+"
};

static const CubeToRobotEntry k_cube_to_robot_table[] = {

  // ===== FRONT (F) =====
  { "f+", "y_plus z_minus d_plus" },
  { "f-", "y_plus z_minus d_minus" },
  { "f2", "y_plus z_minus d_180" },

  // ===== BACK (B) =====
  { "b+", "y_plus z_plus d_plus" },
  { "b-", "y_plus z_plus d_minus" },
  { "b2", "y_plus z_plus d_180" },

  // ===== RIGHT (R) =====
  { "r+", "z_plus d_plus" },
  { "r-", "z_plus d_minus" },
  { "r2", "z_plus d_180" },

  // ===== LEFT (L) =====
  { "l+", "z_minus d_plus" },
  { "l-", "z_minus d_minus" },
  { "l2", "z_minus d_180" },

  // ===== UP (U) =====
  { "u+", "z_180 d_plus" },
  { "u-", "z_180 d_minus" },
  { "u2", "z_180 d_180" },

  // ===== DOWN (D) =====
  { "d+", "d_plus" },
  { "d-", "d_minus" },
  { "d2", "d_180" },
};

static const int k_cube_to_robot_count =
  sizeof(k_cube_to_robot_table) / sizeof(k_cube_to_robot_table[0]);

// ============================================================
// Constructor / Reset
// ============================================================

CubeOri::CubeOri(robot_move_cb_t cb)
  : robot_cb_(cb) {
  clear_orientation_data();
}

void CubeOri::clear_orientation_data() {
  ori_.U = 'u';
  ori_.R = 'r';
  ori_.F = 'f';
  ori_.D = 'd';
  ori_.L = 'l';
  ori_.B = 'b';

  orientation_log_ = "";
}

bool CubeOri::set_orientation_string(String ori_string) {
  // reset first
  clear_orientation_data();

  bool src_seen[6] = { false };
  bool dst_seen[6] = { false };

  auto face_index = [](char c) -> int {
    switch (c) {
      case 'u': return 0;
      case 'r': return 1;
      case 'f': return 2;
      case 'd': return 3;
      case 'l': return 4;
      case 'b': return 5;
      default: return -1;
    }
  };

  int pos = 0;
  int pairs = 0;

  while (pos < ori_string.length()) {
    // skip spaces
    while (pos < ori_string.length() && ori_string[pos] == ' ')
      pos++;

    if (pos >= ori_string.length())
      break;

    // must have at least x->y
    if (pos + 3 >= ori_string.length())
      return false;

    char src = tolower(ori_string[pos]);
    if (ori_string[pos + 1] != '-' || ori_string[pos + 2] != '>')
      return false;

    char dst = tolower(ori_string[pos + 3]);

    int si = face_index(src);
    int di = face_index(dst);

    if (si < 0 || di < 0)
      return false;

    // no duplicates
    if (src_seen[si] || dst_seen[di])
      return false;

    src_seen[si] = true;
    dst_seen[di] = true;
    pairs++;

    // apply mapping
    switch (src) {
      case 'u': ori_.U = dst; break;
      case 'r': ori_.R = dst; break;
      case 'f': ori_.F = dst; break;
      case 'd': ori_.D = dst; break;
      case 'l': ori_.L = dst; break;
      case 'b': ori_.B = dst; break;
    }

    pos += 4;

    // optional space
    if (pos < ori_string.length() && ori_string[pos] == ' ')
      pos++;
  }

  // must have exactly 6 mappings
  if (pairs != 6)
    return false;

  // final validation: all faces used exactly once
  for (int i = 0; i < 6; i++) {
    if (!src_seen[i] || !dst_seen[i])
      return false;
  }

  orientation_log_ = ori_string;
  return true;
}

bool CubeOri::restore_cube_orientation() {
  // Goal (identity orientation)
  Orientation target;
  target.U = 'u';
  target.D = 'd';
  target.F = 'f';
  target.B = 'b';
  target.L = 'l';
  target.R = 'r';

  // Already aligned?
  if (orientations_equal_(ori_, target)) {
    LOG_INFO(MOD_CUBEORI, "orientation", "already at identity");
    return true;
  }

  //
  // BFS over the 24 possible cube orientations.
  // We only use: y+, y-, z+, z'.
  //

  LOG_INFO(MOD_CUBEORI, "info", "orientation_restore_start");
  const String moves[6] = { "z_plus", "z_minus", "z_180",  //
                            "y_plus", "y_minus", "y_180" };

  const int MAX_STATES = 24;

  Orientation states[MAX_STATES];
  int parent_idx[MAX_STATES];
  String move_from_parent[MAX_STATES];

  int queue[MAX_STATES];
  int q_head = 0;
  int q_tail = 0;

  // Initial state
  states[0] = ori_;
  parent_idx[0] = -1;
  queue[q_tail++] = 0;

  int state_count = 1;
  int found_idx = -1;

  // BFS loop
  while (q_head < q_tail && found_idx < 0) {
    int cur = queue[q_head++];
    Orientation cur_o = states[cur];

    // Try applying each legal rotation
    for (int m = 0; m < 6; ++m) {
      Orientation next_o = apply_rotation_to_orientation_(cur_o, moves[m]);

      // Check if visited
      bool seen = false;
      for (int i = 0; i < state_count; i++) {
        if (orientations_equal_(states[i], next_o)) {
          seen = true;
          break;
        }
      }
      if (seen) continue;

      if (state_count >= MAX_STATES) break;  // Should never overflow

      int idx = state_count++;
      states[idx] = next_o;
      parent_idx[idx] = cur;
      move_from_parent[idx] = moves[m];
      queue[q_tail++] = idx;

      if (orientations_equal_(next_o, target)) {
        found_idx = idx;
        break;
      }
    }
  }

  if (found_idx < 0) {
    // Should never happen for a valid orientation group
    LOG_ERR(MOD_CUBEORI, "orentation restore no solution found", found_idx);
    return false;
  }

  //
  // Reconstruct path from found_idx back to state 0 (reverse order)
  //
  String path_moves[16];
  int path_len = 0;

  int cur = found_idx;
  while (cur >= 0 && parent_idx[cur] >= 0 && path_len < 16) {
    path_moves[path_len++] = move_from_parent[cur];
    cur = parent_idx[cur];
  }

  //
  // Replay moves forward (reverse order)
  LOG_INFO(MOD_CUBEORI, "oerientation restore solution found with moves count", path_len);
  for (int i = path_len - 1; i >= 0; --i) {
    if (!robot_move(path_moves[i])) {

      LOG_ERR(MOD_CUBEORI, "error", "restore cube orientation robot move failed");
      LOG_VAR("move", path_moves[i].c_str());

      return false;
    }
  }

  return true;
}

// ============================================================
// Compare two Orientation objects
// ============================================================
bool CubeOri::orientations_equal_(const Orientation &a, const Orientation &b) const {
  return (a.U == b.U && a.R == b.R && a.F == b.F && a.D == b.D && a.L == b.L && a.B == b.B);
}

// ============================================================
// Apply a single rotation ("z_plus", etc") to an Orientation
// (used only inside BFS restore logic)
// ============================================================
CubeOri::Orientation CubeOri::apply_rotation_to_orientation_(const Orientation &o, const String &move) const {
  Orientation n = o;

  if (move.equalsIgnoreCase("z_plus")) {
    n.U = o.L;
    n.R = o.U;
    n.D = o.R;
    n.L = o.D;
    // unchanged
    // n.F = o.F;
    // n.B = o.B;
  } else if (move.equalsIgnoreCase("z_minus")) {
    n.U = o.R;
    n.L = o.U;
    n.D = o.L;
    n.R = o.D;
    // unchanged
    // n.F = o.F;
    // n.B = o.B;
  } else if (move.equalsIgnoreCase("y_minus")) {
    n.F = o.L;
    n.L = o.B;
    n.B = o.R;
    n.R = o.F;
    // unchanged
    // n.U = o.U;
    // n.D = o.D;
  } else if (move.equalsIgnoreCase("y_plus")) {
    n.F = o.R;
    n.R = o.B;
    n.B = o.L;
    n.L = o.F;
    // unchanged
    // n.U = o.U;
    // n.D = o.D;
  } else if (move.equalsIgnoreCase("z_180")) {
    n.U = o.D;
    n.D = o.U;
    n.R = o.L;
    n.L = o.R;
    // unchanged
    // n.F = o.F;
    // n.B = o.B;
  } else if (move.equalsIgnoreCase("y_180")) {
    n.F = o.B;
    n.R = o.L;
    n.B = o.F;
    n.L = o.R;
    // unchanged
    // n.U = o.U;
    // n.D = o.D;
  }
  return n;
}

// ============================================================
// split_moves_ : split by whitespace and commas
// ============================================================
void CubeOri::split_moves_(const String &in,
                           String *out, int &count, int max_count) const {
  count = 0;
  String cur;
  int n = in.length();

  for (int i = 0; i < n; ++i) {
    char c = in.charAt(i);
    bool sep = (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == ',');

    if (sep) {
      if (cur.length() > 0 && count < max_count) {
        out[count++] = cur;
      }
      cur = "";
      if (count >= max_count) break;
    } else {
      cur += c;
    }
  }

  if (cur.length() > 0 && count < max_count) {
    out[count++] = cur;
  }
}

// ============================================================
// Apply ori table for given robot move
// ============================================================
void CubeOri::apply_ori_table_(const String &robot_move) {

  Orientation prev_ori = ori_;
  Orientation new_ori = apply_rotation_to_orientation_(prev_ori, robot_move);
  ori_ = new_ori;
}

// ============================================================
// robot_move: call callback, update orientation, log
// ============================================================
bool CubeOri::robot_move(const String &move_str) {
  if (move_str.length() == 0) return false;

  // Callback first (actual hardware movement)
  if (robot_cb_) {
    if (!robot_cb_(move_str)) {

      LOG_ERR(MOD_CMD, "error", "robot callback failed");
      LOG_VAR("move", move_str.c_str());

      return false;
    }
  }

  // Then update orientation using table
  apply_ori_table_(move_str);

  // Append to log
  if (orientation_log_.length() > 0) orientation_log_ += ' ';
  orientation_log_ += move_str;

  return true;
}

// ============================================================
// Parse cube token "f", "r+", "u2" into (face, quarter_turns)
// qt = +1, -1, or 2
// ============================================================
bool CubeOri::parse_cube_token_(const String &tok,
                                char &face, int &qt) const {
  if (tok.length() == 0) return false;

  char f = tolower(tok.charAt(0));
  if (f != 'f' && f != 'b' && f != 'r' && f != 'l' && f != 'u' && f != 'd') {
    return false;
  }

  char suf = '+';
  if (tok.length() >= 2) {
    char c = tok.charAt(1);
    if (c == '+' || c == ' ') {
      suf = '+';
    } else if (c == '\'' || c == '-') {
      suf = '-';
    } else if (c == '2') {
      suf = '2';
    } else {
      return false;
    }
  }

  face = f;
  if (suf == '+') qt = 1;
  else if (suf == '-') qt = -1;
  else if (suf == '\'') qt = -1;
  else qt = 2;

  return true;
}

// ============================================================
// Find physical direction where a given logical face currently is
// Using the orientation map (physical -> logical).
// ============================================================
char CubeOri::find_physical_dir_for_logical_(char logical_face) const {
  if (ori_.U == logical_face) return 'u';
  if (ori_.R == logical_face) return 'r';
  if (ori_.F == logical_face) return 'f';
  if (ori_.D == logical_face) return 'd';
  if (ori_.L == logical_face) return 'l';
  if (ori_.B == logical_face) return 'b';
  return '\0';
}

// ============================================================
// Execute a single cube logical move (one face, one suffix)
// Steps:
//  1) Find where that logical face currently is physically.
//  2) Use k_cube_to_robot_table keyed by that *physical* face.
//  3) Run each robot move via robot_move().
// ============================================================
bool CubeOri::execute_single_cube_move_(char logical_face, int qt) {
  // 1) Which physical direction currently holds this logical face?
  char phys = find_physical_dir_for_logical_(logical_face);
  if (phys == '\0') {

    LOG_ERR(MOD_CUBEORI, "error", "logical face not found in orientation");
    LOG_VAR("face", logical_face);

    return false;
  }

  // 2) Build canonical cube-move key, e.g. "F+", "B'", "R2"
  String key;
  key.reserve(3);
  key += phys;
  if (qt == 1) key += '+';
  else if (qt == -1) key += '-';
  else key += '2';

  // 3) Find in k_cube_to_robot_table
  for (int i = 0; i < k_cube_to_robot_count; ++i) {
    if (key.equalsIgnoreCase(k_cube_to_robot_table[i].cube_move)) {

      String moves_str = k_cube_to_robot_table[i].robot_moves;
      String toks[8];
      int count = 0;
      split_moves_(moves_str, toks, count, 8);

      for (int j = 0; j < count; ++j) {
        if (!robot_move(toks[j])) {

          LOG_ERR(MOD_CMD, "error", "robot_move_failed_in_sequence");
          LOG_VAR("sequence", key.c_str());
          LOG_VAR("move", toks[j].c_str());

          return false;
        }
      }
      return true;
    }
  }

  LOG_ERR(MOD_CUBEMOVE, "error", "cube_move_not_found_in_table");
  LOG_VAR("move", key.c_str());

  return false;
}

// ============================================================
// cube_move: "F R' U2" etc
// ============================================================
bool CubeOri::cube_move(const String &moves_str) {

  // serial_printf_verbose("[cube_move] called with: \"%s\"\n", moves_str.c_str());

  // --- Normalize input to lowercase ---
  String moves_lc = moves_str;
  moves_lc.toLowerCase();

  const int MAX_TOKENS = 64;
  String tokens[MAX_TOKENS];
  int token_count = 0;
  cube_move_index = 0;
  cube_move_total = 0;

  split_moves_(moves_lc, tokens, token_count, MAX_TOKENS);

  // serial_printf_verbose("[cube_move] token_count=%d\n", token_count);
  cube_move_total = token_count;

  if (token_count == 0) return true;  // nothing to do

  for (int i = 0; i < token_count; ++i) {
    String t = tokens[i];
    t.trim();
    if (t.length() == 0) continue;

    // serial_printf_verbose("[cube_move] parsing token: \"%s\"\n", t.c_str());

    char face;
    int qt;
    if (!parse_cube_token_(t, face, qt)) {
      LOG_ERR(MOD_CUBEMOVE, "error", "not_a_cube_move");
      LOG_VAR("move", t.c_str());

      return false;
    }

    // serial_printf_verbose("[cube_move] parsed: face=%c, qt=%d\n", face, qt);

    // format logging: lowercase face + suffix (+, ', 2)
    char face_l = tolower(face);
    char suf;
    if (qt == 1) suf = '+';
    else if (qt == -1) suf = '-';
    else suf = '2';

    cube_move_index++;
    LOG_INFO(MOD_CUBEMOVE, "info", "cube_move_progress");
    LOG_VAR("index", cube_move_index);
    LOG_VAR("total", cube_move_total);
    LOG_VAR("face", face_l);
    LOG_VAR("suffix", suf);

    if (!execute_single_cube_move_(face, qt)) {
      LOG_ERR(MOD_CUBEMOVE, "error", "cube_move_failed_executing");
      LOG_VAR("move", t.c_str());

      return false;
    }
    color_analyzer.apply_move(t);
  }
  return true;
}

// ============================================================
// get_face_mapping: logical -> physical mapping in standard order
// U, R, F, D, L, B
// Each entry is "X->Y", e.g. "f->b" means original logical F is now on phys B.
// ============================================================
void CubeOri::get_face_mapping(String out[6]) const {
  auto find_dir = [this](char logical_face) -> char {
    if (ori_.U == logical_face) return 'u';
    if (ori_.R == logical_face) return 'r';
    if (ori_.F == logical_face) return 'f';
    if (ori_.D == logical_face) return 'd';
    if (ori_.L == logical_face) return 'l';
    if (ori_.B == logical_face) return 'b';
    return '?';
  };

  char dirs[6] = {
    find_dir('u'),
    find_dir('r'),
    find_dir('f'),
    find_dir('d'),
    find_dir('l'),
    find_dir('b')
  };

  out[0] = String("u->") + dirs[0];
  out[1] = String("r->") + dirs[1];
  out[2] = String("f->") + dirs[2];
  out[3] = String("d->") + dirs[3];
  out[4] = String("l->") + dirs[4];
  out[5] = String("b->") + dirs[5];
}

// ============================================================
// get_orientation_string: U->U R->R F->F D->D L->L B->B
// ============================================================
String CubeOri::get_orientation_string() const {
  String maps[6];
  get_face_mapping(maps);

  String out;
  out.reserve(6 * 4 + 5);  // "u->x " * 5 + last

  for (int i = 0; i < 6; i++) {
    out += maps[i];
    if (i < 5) out += ' ';
  }
  return out;
}

// ============================================================
// Helpers
// ============================================================

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

bool is_orientation_string_valid(String ori_string) {
  bool src_seen[6] = { false };
  bool dst_seen[6] = { false };

  auto face_index = [](char c) -> int {
    switch (tolower(c)) {
      case 'u': return 0;
      case 'r': return 1;
      case 'f': return 2;
      case 'd': return 3;
      case 'l': return 4;
      case 'b': return 5;
      default: return -1;
    }
  };

  int pos = 0;
  int pairs = 0;

  while (pos < ori_string.length()) {
    while (pos < ori_string.length() && ori_string[pos] == ' ')
      pos++;

    if (pos >= ori_string.length())
      break;

    // must have at least x->y
    if (pos + 3 >= ori_string.length())
      return false;

    char src = tolower(ori_string[pos]);
    if (ori_string[pos + 1] != '-' || ori_string[pos + 2] != '>')
      return false;

    char dst = tolower(ori_string[pos + 3]);

    int si = face_index(src);
    int di = face_index(dst);
    if (si < 0 || di < 0)
      return false;

    // no duplicates
    if (src_seen[si] || dst_seen[di])
      return false;

    src_seen[si] = true;
    dst_seen[di] = true;
    pairs++;

    pos += 4;
    if (pos < ori_string.length() && ori_string[pos] == ' ')
      pos++;
  }

  if (pairs != 6) return false;

  for (int i = 0; i < 6; i++) {
    if (!src_seen[i] || !dst_seen[i]) return false;
  }
  return true;
}

// ============================================================
// Face conversion helpers using current orientation
// ============================================================

char CubeOri::robot_face_to_cube_face(char f_face) {
  char c = tolower(f_face);
  if (!is_valid_face(c)) return '\0';

  switch (c) {
    case 'u': return ori_.U;
    case 'r': return ori_.R;
    case 'f': return ori_.F;
    case 'd': return ori_.D;
    case 'l': return ori_.L;
    case 'b': return ori_.B;
    default: return '\0';
  }
}

char CubeOri::cube_face_to_robot_face(char f_face) {
  char c = tolower(f_face);
  if (!is_valid_face(c)) return '\0';

  // inverse: physical direction where this logical face currently is
  return find_physical_dir_for_logical_(c);  // returns '\0' if not found
}

// ============================================================
// set orientation knowing: physical F has logical f_face, physical R has logical r_face
// ============================================================

bool CubeOri::set_orientation_from_front_and_right_faces(char f_face, char r_face) {
  char f = tolower(f_face);
  char r = tolower(r_face);

  if (!is_valid_face(f) || !is_valid_face(r)) return false;

  // Must be perpendicular (not same, not opposites)
  if (f == r) return false;
  if (oposite_face(f) == r) return false;

  // Vector encoding in cube logical space:
  // r=(+1,0,0), l=(-1,0,0), u=(0,+1,0), d=(0,-1,0), f=(0,0,+1), b=(0,0,-1)
  struct V3 {
    int x, y, z;
  };

  auto face_to_vec = [](char c) -> V3 {
    switch (tolower(c)) {
      case 'r': return { 1, 0, 0 };
      case 'l': return { -1, 0, 0 };
      case 'u': return { 0, 1, 0 };
      case 'd': return { 0, -1, 0 };
      case 'f': return { 0, 0, 1 };
      case 'b': return { 0, 0, -1 };
      default: return { 0, 0, 0 };
    }
  };

  auto cross = [](V3 a, V3 b) -> V3 {
    return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x
    };
  };

  auto vec_to_face = [](V3 v) -> char {
    if (v.x == 1 && v.y == 0 && v.z == 0) return 'r';
    if (v.x == -1 && v.y == 0 && v.z == 0) return 'l';
    if (v.x == 0 && v.y == 1 && v.z == 0) return 'u';
    if (v.x == 0 && v.y == -1 && v.z == 0) return 'd';
    if (v.x == 0 && v.y == 0 && v.z == 1) return 'f';
    if (v.x == 0 && v.y == 0 && v.z == -1) return 'b';
    return '\0';
  };

  V3 vf = face_to_vec(f);
  V3 vr = face_to_vec(r);

  // Physical axes: X=R, Y=U, Z=F. For a right-handed frame: U = F x R
  V3 vu = cross(vf, vr);
  char u = vec_to_face(vu);
  if (!is_valid_face(u)) return false;

  // Build full consistent orientation (physical -> logical)
  Orientation o;
  o.F = f;
  o.R = r;
  o.U = u;
  o.B = oposite_face(o.F);
  o.L = oposite_face(o.R);
  o.D = oposite_face(o.U);

  if (!is_valid_face(o.B) || !is_valid_face(o.L) || !is_valid_face(o.D)) return false;

  // Sanity: all 6 must be unique
  bool used[256] = { false };
  char faces[6] = { o.U, o.R, o.F, o.D, o.L, o.B };
  for (int i = 0; i < 6; i++) {
    uint8_t k = (uint8_t)faces[i];
    if (used[k]) return false;
    used[k] = true;
  }

  // Commit
  ori_ = o;
  orientation_log_ = get_orientation_string();
  return true;
}

bool update_ori_from_color_54(String color_54) {
  LOG_INFO(MOD_RUN, "infer all centers from", color_54);

  char front_color = get_stickercolor_from_color_string_54(color_54, 'f', 5);
  char right_color = get_stickercolor_from_color_string_54(color_54, 'r', 5);

  if (!is_valid_color(front_color) || !is_valid_color(right_color)) {
    LOG_ERR(MOD_RUN, "front and right colors just read invalid front", front_color);
    LOG_VAR("right", right_color);
    return false;
  }
  char robot_front_face = color_to_face(front_color);
  char robot_right_face = color_to_face(right_color);
  //
  bool ok = ori.set_orientation_from_front_and_right_faces(robot_front_face, robot_right_face);
  if (ok) {
    LOG_ERR(MOD_RUN, "ori set orintation from front and right faces failed attempted front", robot_front_face);
    LOG_VAR("right", robot_right_face);
    return false;
  }
  return true;
}
