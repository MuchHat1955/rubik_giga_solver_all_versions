#include "ori.h"
#include "utils.h"
#include "color_reader.h"
#include "color_analyzer.h"
#include "cmd_parser.h"

extern CubeColorReader color_reader;
extern ColorAnalyzer color_analyzer;

int cube_move_index = 0;
int cube_move_total = 0;

extern uint32_t start_ms;

/*
ORI logging (CubeOri)

RB_INFO ORI
  info=orientation_cleared
  info=robot_move_start        move
  info=robot_move_end          move
  info=orientation_updated    orientation
  info=cube_move_token        cube_move step total_steps
  info=cube_move_applied      cube_move
  info=restore_start
  info=restore_move           move
  info=restore_complete

RB_ERR ORI
  err=robot_callback_failed   move
  err=logical_face_not_found  face
  err=cube_move_not_found     key
  err=robot_move_failed       move
  err=restore_failed
  err=invalid_cube_token      token
  */

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

  RB_INFO_CUBEORI("orientation_cleared", "(na)", "", "");
}

bool CubeOri::restore_cube_orientation() {

  RB_INFO_CUBEORI("restore_start", "(na)", "", "");

  // ------------------------------------------------------------
  // Target orientation = identity
  // ------------------------------------------------------------
  Orientation target;
  target.U = 'u';
  target.R = 'r';
  target.F = 'f';
  target.D = 'd';
  target.L = 'l';
  target.B = 'b';

  // Already aligned?
  if (orientations_equal_(ori_, target)) {
    RB_INFO_CUBEORI("restore_complete", "(na)",
                    "already_identity=true", "");
    return true;
  }

  // ------------------------------------------------------------
  // BFS over the 24 possible cube orientations
  // ------------------------------------------------------------
  const String moves[6] = {
    "z_plus", "z_minus", "z_180",
    "y_plus", "y_minus", "y_180"
  };

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

  // ------------------------------------------------------------
  // BFS search
  // ------------------------------------------------------------
  while (q_head < q_tail && found_idx < 0) {
    int cur = queue[q_head++];
    Orientation cur_o = states[cur];

    for (int m = 0; m < 6; ++m) {
      Orientation next_o = apply_rotation_to_orientation_(cur_o, moves[m]);

      // Check if already visited
      bool seen = false;
      for (int i = 0; i < state_count; i++) {
        if (orientations_equal_(states[i], next_o)) {
          seen = true;
          break;
        }
      }
      if (seen) continue;

      if (state_count >= MAX_STATES) break;

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
    RB_ERR_CUBEORI("restore_failed", "(na)",
                   "reason=no_path");
    return false;
  }

  // ------------------------------------------------------------
  // Reconstruct path (reverse)
  // ------------------------------------------------------------
  String path_moves[16];
  int path_len = 0;

  int cur = found_idx;
  while (cur >= 0 && parent_idx[cur] >= 0 && path_len < 16) {
    path_moves[path_len++] = move_from_parent[cur];
    cur = parent_idx[cur];
  }

  // ------------------------------------------------------------
  // Replay moves forward
  // ------------------------------------------------------------
  for (int i = path_len - 1; i >= 0; --i) {

    RB_INFO_CUBEORI("restore_move", "(na)",
                    "move=%s",
                    path_moves[i].c_str());

    if (!robot_move(path_moves[i])) {
      RB_ERR_CUBEORI("restore_failed", "(na)",
                     "move=%s",
                     path_moves[i].c_str());
      return false;
    }
  }

  RB_INFO_CUBEORI("restore_complete", "(na)",
                  "orientation=%s",
                  get_orientation_string().c_str());

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

  RB_INFO_ROBOTMOVE("robot_move_start", "(na)",
                    "move=%s",
                    move_str.c_str());

  // Hardware callback
  if (robot_cb_) {
    if (!robot_cb_(move_str)) {
      RB_ERR_ROBOTMOVE("robot_callback_failed", "(na)",
                       "move=%s",
                       move_str.c_str());
      return false;
    }
  }

  // Update orientation
  apply_ori_table_(move_str);

  // Append to log
  if (orientation_log_.length() > 0) orientation_log_ += ' ';
  orientation_log_ += move_str;

  RB_INFO_ROBOTMOVE("robot_move_end", "(na)",
                    "move=%s orientation=%s",
                    move_str.c_str(),
                    get_orientation_string().c_str());

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

  char phys = find_physical_dir_for_logical_(logical_face);
  if (phys == '\0') {
    RB_ERR_CUBEORI("logical_face_not_found", "(na)",
                   "face=%c",
                   logical_face);
    return false;
  }

  String key;
  key += phys;
  key += (qt == 1 ? '+' : qt == -1 ? '-'
                                   : '2');

  for (int i = 0; i < k_cube_to_robot_count; ++i) {
    if (key.equalsIgnoreCase(k_cube_to_robot_table[i].cube_move)) {

      String toks[8];
      int count = 0;
      split_moves_(k_cube_to_robot_table[i].robot_moves, toks, count, 8);

      for (int j = 0; j < count; ++j) {
        if (!robot_move(toks[j])) {
          RB_ERR_CUBEORI("robot_move_failed", "(na)",
                         "cube_move=%s robot_move=%s",
                         key.c_str(),
                         toks[j].c_str());
          return false;
        }
      }
      return true;
    }
  }

  RB_ERR_CUBEORI("cube_move_not_found", "(na)",
                 "key=%s",
                 key.c_str());
  return false;
}

// ============================================================
// cube_move: "F R' U2" etc
// ============================================================
bool CubeOri::cube_move(const String &moves_str) {

  String moves_lc = moves_str;
  moves_lc.toLowerCase();

  String tokens[64];
  int token_count = 0;

  cube_move_index = 0;
  split_moves_(moves_lc, tokens, token_count, 64);
  cube_move_total = token_count;

  if (token_count == 0) return true;

  for (int i = 0; i < token_count; ++i) {
    String t = tokens[i];
    t.trim();
    if (!t.length()) continue;

    char face;
    int qt;
    if (!parse_cube_token_(t, face, qt)) {
      RB_ERR_CUBEMOVE("invalid_cube_token", "(na)",
                      "token=%s",
                      t.c_str());
      return false;
    }

    cube_move_index++;

    RB_INFO_CUBEMOVE("cube_move_token", "(na)",
                     "cube_move=%s step=%d total_steps=%d",
                     t.c_str(),
                     cube_move_index,
                     cube_move_total);

    if (!execute_single_cube_move_(face, qt)) {
      RB_ERR_CUBEMOVE("cube_move_failed", "(na)",
                      "cube_move=%s",
                      t.c_str());
      return false;
    }

    // keep color model in sync
    color_reader.apply_moves(t);

    RB_ERR_CUBEORI("cube_move_applied", "(na)",
                    "cube_move=%s orientation=%s",
                    t.c_str(),
                    get_orientation_string().c_str());
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
// get_orientation_string from: U->U R->R F->F D->D L->L B->B
// returned as URLDLB
// ============================================================
String CubeOri::get_orientation_string() const {
  String maps[6];
  get_face_mapping(maps);

  String s;
  s.reserve(48);
  s = "URLDLB->";
  s += String(maps[0].charAt(3));  //U
  s += String(maps[1].charAt(3));
  s += String(maps[2].charAt(3));
  s += String(maps[3].charAt(3));
  s += String(maps[4].charAt(3));
  s += String(maps[5].charAt(3));
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

  serial_printf_verbose("      %c\n", maps[0].charAt(3));
  serial_printf_verbose("   %c  %c  %c  %c\n", maps[1].charAt(3), maps[2].charAt(3), maps[4].charAt(3), maps[5].charAt(3));
  serial_printf_verbose("      %c\n", maps[3].charAt(3));
}
