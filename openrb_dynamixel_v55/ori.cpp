#include "ori.h"
#include "utils.h"
#include "color_reader.h"
#include "color_analyzer.h"

extern CubeColorReader color_reader;
extern ColorAnalyzer color_analyzer;

int cube_move_index = 0;
int cube_move_total = 0;

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
  const char *robot_move;  // "z_plus", etc
  const char *changes[4];  // up to 4 directional changes "U->R"
};

static const OriMoveMap k_ori_move_table[] = {
  { "z_plus", { "U->R", "R->D", "D->L", "L->U" } },
  { "z_minus", { "U->L", "L->D", "D->R", "R->U" } },
  { "z_180", { "U->D", "R->L", "D->U", "L->R" } },

  { "y_plus", { "F->L", "L->B", "B->R", "R->F" } },
  { "y_minus", { "F->R", "R->B", "B->L", "L->F" } },
  { "y_180", { "F->B", "B->F", "L->R", "R->L" } },

  // No orientation change for d+, d', d2
  { "d_plus", { "", "", "", "" } },
  { "d_minus", { "", "", "", "" } },
  { "d_180", { "", "", "", "" } },
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
    return true;
  }

  //
  // BFS over the 24 possible cube orientations.
  // We only use: y+, y-, z+, z'.
  //

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
// Apply a single rotation ("z_plus", etc") to an Orientation
// (used only inside BFS restore logic)
// ============================================================
CubeOri::Orientation

CubeOri::apply_rotation_to_orientation_(const Orientation &o, const String &move) const {
  Orientation n = o;

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
  } else if (move.equalsIgnoreCase("y_minus")) {
    n.F = o.L;
    n.L = o.B;
    n.B = o.R;
    n.R = o.F;
    // unchanged
    n.U = o.U;
    n.D = o.D;
  } else if (move.equalsIgnoreCase("y_plus")) {
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
    n.D = o.D;
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
          case 'u': src_val = old.U; break;
          case 'r': src_val = old.R; break;
          case 'f': src_val = old.F; break;
          case 'd': src_val = old.D; break;
          case 'l': src_val = old.L; break;
          case 'b': src_val = old.B; break;
          default: break;
        }

        if (!src_val) continue;

        switch (to_dir) {
          case 'u': neu.U = src_val; break;
          case 'r': neu.R = src_val; break;
          case 'f': neu.F = src_val; break;
          case 'd': neu.D = src_val; break;
          case 'l': neu.L = src_val; break;
          case 'b': neu.B = src_val; break;
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
  if (move_str.length() == 0) return false;

  // Callback first (actual hardware movement)
  if (robot_cb_) {
    if (!robot_cb_(move_str)) {
      serial_printf("ERR robot callback failed for %s\n", move_str.c_str());
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
    serial_printf("ERR logical face %c not found in orientation\n", logical_face);
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
bool CubeOri::cube_move(const String &moves_str) {

  serial_printf_verbose("[cube_move] called with: \"%s\"\n", moves_str.c_str());

  // --- Normalize input to lowercase ---
  String moves_lc = moves_str;
  moves_lc.toLowerCase();

  const int MAX_TOKENS = 64;
  String tokens[MAX_TOKENS];
  int token_count = 0;
  cube_move_index = 0;
  cube_move_total = 0;

  split_moves_(moves_lc, tokens, token_count, MAX_TOKENS);

  serial_printf_verbose("[cube_move] token_count=%d\n", token_count);
  cube_move_total = token_count;

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
    else if (qt == -1) suf = '-';
    else suf = '2';

    cube_move_index++;
    serial_printf("[%d/%d cube_move] %c%c\n", cube_move_index, cube_move_total, face_l, suf);

    if (!execute_single_cube_move_(face, qt)) {
      serial_printf("ERR [cube_move] failed executing: %s\n", t.c_str());
      return false;
    }
    color_reader.apply_moves(t);

    // show the stage
    String all54 = color_reader.get_cube_colors_string();
    color_analyzer.set_colors(all54);

    if (!color_analyzer.is_string_fixable_bool())
      serial_printf("ERR cube colors invalid and not fixable\n");

    for (int s = 0; s < color_analyzer.get_stage_count(); s++) {
      serial_printf("[stage %d (%s)] done=%d partial=%d\n",
                    s,
                    color_analyzer.get_stage_name(s),
                    color_analyzer.is_stage_done_bool(s),
                    color_analyzer.is_stage_partial_bool(s));
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

  out[0] = String("U->") + dirs[0];
  out[1] = String("R->") + dirs[1];
  out[2] = String("F->") + dirs[2];
  out[3] = String("D->") + dirs[3];
  out[4] = String("L->") + dirs[4];
  out[5] = String("B->") + dirs[5];
}

// ============================================================
// get_orientation_string: U->U R->R F->F D->D L->L B->B
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

// ============================================================
// print_orientation_string: 0->U 1->R 2->F 3->D 4->L 5->B
// ============================================================
//      U
//   R  F  L  B
//      D
void CubeOri::print_orientation_string() const {
  String maps[6];
  get_face_mapping(maps);

  serial_printf("      %c\n", maps[0].charAt(3));
  serial_printf("   %c  %c  %c  %c\n", maps[1].charAt(3), maps[2].charAt(3), maps[4].charAt(3), maps[5].charAt(3));
  serial_printf("      %c\n", maps[3].charAt(3));
}
