#include "ori.h"
#include "utils.h"
#include "color_reader.h"

extern CubeColorReader color_reader;

// ============================================================
// Tables
// ============================================================

// Orientation change table for robot moves.
// Interpretation of "U->R" here:
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
  const char *robot_move;  // "y+", "y'", "z+", "z'", "z2", "d+", "d'", "d2"
  const char *changes[4];  // up to 4 directional changes "U->R"
};

static const OriMoveMap k_ori_move_table[] = {
  // y+ = roll clockwise (front view): U→R, R→D, D→L, L→U
  { "y+", { "U->R", "R->D", "D->L", "L->U" } },

  // y' = roll counterclockwise: U→L, L→D, D→R, R→U
  { "y'", { "U->L", "L->D", "D->R", "R->U" } },

  { "z+", { "F->R", "R->B", "B->L", "L->F" } },
  { "z'", { "F->L", "L->B", "B->R", "R->F" } },
  { "z2", { "F->B", "B->F", "L->R", "R->L" } },

  // No orientation change for d+, d', d2
  { "d+", { "", "", "", "" } },
  { "d'", { "", "", "", "" } },
  { "d2", { "", "", "", "" } },
};

static const int k_ori_move_table_count =
  sizeof(k_ori_move_table) / sizeof(k_ori_move_table[0]);

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
  // F → rotate cube so F becomes Down → F is already oriented by z' (not z+)
  // Best minimal sequence:
  //   F+  → z' d+
  //   F'  → z' d'
  //   F2  → z' d2
  { "F+", "z' d+" },
  { "F'", "z' d'" },
  { "F2", "z' d2" },

  // ===== BACK (B) =====
  // B → rotate cube so B becomes Down
  // Best minimal sequence:
  //   B+  → z+ d+
  { "B+", "z+ d+" },
  { "B'", "z+ d'" },
  { "B2", "z+ d2" },

  // ===== RIGHT (R) =====
  // R is one y+ away from being Down
  { "R+", "y+ d+" },
  { "R'", "y+ d'" },
  { "R2", "y+ d2" },

  // ===== LEFT (L) =====
  // L is one y′ away
  { "L+", "y' d+" },
  { "L'", "y' d'" },
  { "L2", "y' d2" },

  // ===== UP (U) =====
  // U → D requires 180° around Y → minimal is y2
  { "U+", "y2 d+" },
  { "U'", "y2 d'" },
  { "U2", "y2 d2" },

  // ===== DOWN (D) =====
  // already Down
  { "D+", "d+" },
  { "D'", "d'" },
  { "D2", "d2" },
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
  ori_.U = 'U';
  ori_.R = 'R';
  ori_.F = 'F';
  ori_.D = 'D';
  ori_.L = 'L';
  ori_.B = 'B';

  orientation_log_ = "";
}

bool CubeOri::restore_cube_orientation() {
  // Goal (identity orientation)
  Orientation target;
  target.U = 'U';
  target.D = 'D';
  target.F = 'F';
  target.B = 'B';
  target.L = 'L';
  target.R = 'R';

  // Already aligned?
  if (orientations_equal_(ori_, target)) {
    return true;
  }

  //
  // BFS over the 24 possible cube orientations.
  // We only use: y+, y', z+, z'.
  //

  const String moves[4] = { "y+", "y'", "z+", "z'" };

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
    for (int m = 0; m < 4; ++m) {
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
  //
  for (int i = path_len - 1; i >= 0; --i) {
    if (!robot_move(path_moves[i])) {
      serial_printf("ERR restore_cube_orientation robot move failed: %s\n",
                    path_moves[i].c_str());
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
// Apply a single rotation ("y+", "y'", "z+", "z'") to an Orientation
// (used only inside BFS restore logic)
// ============================================================
CubeOri::Orientation

CubeOri::apply_rotation_to_orientation_(const Orientation &o, const String &move) const {
  Orientation n = o;

  if (move.equalsIgnoreCase("y+")) {
    // U→R, R→D, D→L, L→U  (roll clockwise)
    n.U = o.L;  // WRONG earlier → fix it to correct orientation mapping
    n.R = o.U;
    n.D = o.R;
    n.L = o.D;
    n.F = o.F;
    n.B = o.B;
  } else if (move.equalsIgnoreCase("y'")) {
    // U→L, L→D, D→R, R→U  (roll counterclockwise)
    n.U = o.R;
    n.L = o.U;
    n.D = o.L;
    n.R = o.D;
    n.F = o.F;
    n.B = o.B;
  } else if (move.equalsIgnoreCase("z+")) {
    n.F = o.L;
    n.L = o.B;
    n.B = o.R;
    n.R = o.F;
    n.U = o.U;
    n.D = o.D;
  } else if (move.equalsIgnoreCase("z'")) {
    n.F = o.R;
    n.R = o.B;
    n.B = o.L;
    n.L = o.F;
    n.U = o.U;
    n.D = o.D;
  } else {
    // Should not happen for restore logic
    return o;
  }

  return n;
}

// ============================================================
// Normalize robot move token
// in: "Y+", " y' ", "Z2", "d-" etc
// out: "y+", "y'", "z2", "d+"
// ============================================================
String CubeOri::normalize_robot_move_token_(const String &in) {
  String t = in;
  t.trim();
  if (t.length() == 0) return String("");

  char axis = tolower(t.charAt(0));
  if (axis != 'y' && axis != 'z' && axis != 'd') {
    serial_printf("ERR not a robot move: %s\n", in.c_str());
    return String("");
  }

  char suf = '+';
  if (t.length() >= 2) {
    char c = t.charAt(1);
    if (c == '+') suf = '+';
    else if (c == '\'' || c == '-') suf = '\'';
    else if (c == '2') suf = '2';
    else {
      serial_printf("ERR invalid robot move suffix in: %s\n", in.c_str());
      return String("");
    }
  }

  String out;
  out.reserve(3);
  out += axis;
  out += suf;
  return out;
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
  // Find row
  for (int i = 0; i < k_ori_move_table_count; ++i) {
    if (robot_move.equalsIgnoreCase(k_ori_move_table[i].robot_move)) {
      // Copy old orientation
      Orientation old = ori_;
      Orientation neu = old;  // start with unchanged

      for (int j = 0; j < 4; ++j) {
        const char *chg = k_ori_move_table[i].changes[j];
        if (!chg || chg[0] == '\0') continue;  // empty slot

        // chg format "X->Y"
        char from_dir = chg[0];
        char to_dir = chg[3];

        char src_val = 0;
        switch (from_dir) {
          case 'U': src_val = old.U; break;
          case 'R': src_val = old.R; break;
          case 'F': src_val = old.F; break;
          case 'D': src_val = old.D; break;
          case 'L': src_val = old.L; break;
          case 'B': src_val = old.B; break;
          default: break;
        }

        if (!src_val) continue;

        switch (to_dir) {
          case 'U': neu.U = src_val; break;
          case 'R': neu.R = src_val; break;
          case 'F': neu.F = src_val; break;
          case 'D': neu.D = src_val; break;
          case 'L': neu.L = src_val; break;
          case 'B': neu.B = src_val; break;
          default: break;
        }
      }

      ori_ = neu;
      return;
    }
  }

  // Not found => no orientation change, silently ignore.
}

// ============================================================
// robot_move: call callback, update orientation, log
// ============================================================
bool CubeOri::robot_move(const String &move_str) {
  String canon = normalize_robot_move_token_(move_str);
  if (canon.length() == 0) return false;

  // Callback first (actual hardware movement)
  if (robot_cb_) {
    if (!robot_cb_(canon)) {
      serial_printf("ERR robot callback failed for %s\n", canon.c_str());
      return false;
    }
  }

  // Then update orientation using table
  apply_ori_table_(canon);

  // Append to log
  if (orientation_log_.length() > 0) orientation_log_ += ' ';
  orientation_log_ += canon;

  return true;
}

// ============================================================
// Parse cube token "F", "R'", "U2" into (face, quarter_turns)
// qt = +1, -1, or 2
// ============================================================
bool CubeOri::parse_cube_token_(const String &tok,
                                char &face, int &qt) const {
  if (tok.length() == 0) return false;

  char f = toupper(tok.charAt(0));
  if (f != 'F' && f != 'B' && f != 'R' && f != 'L' && f != 'U' && f != 'D') {
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
  else qt = 2;

  return true;
}

// ============================================================
// Find physical direction where a given logical face currently is
// Using the orientation map (physical -> logical).
// ============================================================
char CubeOri::find_physical_dir_for_logical_(char logical_face) const {
  if (ori_.U == logical_face) return 'U';
  if (ori_.R == logical_face) return 'R';
  if (ori_.F == logical_face) return 'F';
  if (ori_.D == logical_face) return 'D';
  if (ori_.L == logical_face) return 'L';
  if (ori_.B == logical_face) return 'B';
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
    serial_printf("ERR logical face %c not found in orientation\n", logical_face);
    return false;
  }

  // 2) Build canonical cube-move key, e.g. "F+", "B'", "R2"
  String key;
  key.reserve(3);
  key += phys;
  if (qt == 1) key += '+';
  else if (qt == -1) key += '\'';
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
          serial_printf("ERR robot move failed in sequence for %s at %s\n",
                        key.c_str(), toks[j].c_str());
          return false;
        }
      }
      return true;
    }
  }

  serial_printf("ERR cube move not found in table: %s\n", key.c_str());
  return false;
}

// ============================================================
// cube_move: "F R' U2" etc
// ============================================================
// ============================================================
// cube_move: "F R' U2" etc
// ============================================================
bool CubeOri::cube_move(const String &moves_str) {

  serial_printf("[cube_move] called with: \"%s\"\n", moves_str.c_str());

  const int MAX_TOKENS = 64;
  String tokens[MAX_TOKENS];
  int token_count = 0;

  split_moves_(moves_str, tokens, token_count, MAX_TOKENS);

  serial_printf_verbose("[cube_move] token_count=%d\n", token_count);

  if (token_count == 0) return true;  // nothing to do

  for (int i = 0; i < token_count; ++i) {
    String t = tokens[i];
    t.trim();
    if (t.length() == 0) continue;

    serial_printf_verbose("[cube_move] parsing token: \"%s\"\n", t.c_str());

    char face;
    int qt;
    if (!parse_cube_token_(t, face, qt)) {
      serial_printf("ERR [cube_move] not a cube move: %s\n", t.c_str());
      return false;
    }

    serial_printf_verbose("[cube_move] parsed: face=%c, qt=%d\n", face, qt);

    // format logging: lowercase face + suffix (+, ', 2)
    char face_l = tolower(face);
    char suf;
    if (qt == 1) suf = '+';
    else if (qt == -1) suf = '\'';
    else suf = '2';

    serial_printf("[cube_move] %c%c\n", face_l, suf);

    if (!execute_single_cube_move_(face, qt)) {
      serial_printf("ERR [cube_move] failed executing: %s\n", t.c_str());
      return false;
    }
  }

  serial_printf_verbose("[cube_move] done\n");
  return true;
}


// ============================================================
// get_face_mapping: logical -> physical mapping in standard order
// U, R, F, D, L, B
// Each entry is "X->Y", e.g. "F->B" means original logical F is now on phys B.
// ============================================================
void CubeOri::get_face_mapping(String out[6]) const {
  auto find_dir = [this](char logical_face) -> char {
    if (ori_.U == logical_face) return 'U';
    if (ori_.R == logical_face) return 'R';
    if (ori_.F == logical_face) return 'F';
    if (ori_.D == logical_face) return 'D';
    if (ori_.L == logical_face) return 'L';
    if (ori_.B == logical_face) return 'B';
    return '?';
  };

  char dirs[6] = {
    find_dir('U'),
    find_dir('R'),
    find_dir('F'),
    find_dir('D'),
    find_dir('L'),
    find_dir('B')
  };

  out[0] = String("U->") + dirs[0];
  out[1] = String("R->") + dirs[1];
  out[2] = String("F->") + dirs[2];
  out[3] = String("D->") + dirs[3];
  out[4] = String("L->") + dirs[4];
  out[5] = String("B->") + dirs[5];
}

// ============================================================
// get_orientation_string: "U->F R->R F->D D->B L->U B->L"
// ============================================================
String CubeOri::get_orientation_string() const {
  String maps[6];
  get_face_mapping(maps);

  String s;
  s.reserve(48);
  s += maps[0];
  s += ' ';
  s += maps[1];
  s += ' ';
  s += maps[2];
  s += ' ';
  s += maps[3];
  s += ' ';
  s += maps[4];
  s += ' ';
  s += maps[5];
  return s;
}
