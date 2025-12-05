#include "ori.h"
#include "utils.h"

// ------------------------------------------------------------
// CONSTRUCTOR / RESET / SET CALLBACK
// ------------------------------------------------------------

CubeOri::CubeOri(robot_move_cb_t cb)
  : robot_cb_(cb) {
  reset();
}

void CubeOri::reset() {
  // Identity orientation: directions map to same-named logical faces.
  ori_.up = 'U';
  ori_.down = 'D';
  ori_.front = 'F';
  ori_.back = 'B';
  ori_.left = 'L';
  ori_.right = 'R';
  orientation_log_str_ = "";
}

void CubeOri::set_robot_callback(robot_move_cb_t cb) {
  robot_cb_ = cb;
}

// ------------------------------------------------------------
// UTILS: NORMALIZATION / PARSING
// ------------------------------------------------------------

String CubeOri::normalize_robot_move_token_(const String &in) {
  String t = in;
  t.trim();
  if (t.length() == 0) return String("");

  char axis = tolower(t.charAt(0));
  if (axis != 'y' && axis != 'z' && axis != 'd') {
    serial_printf("ERR not a robot move %s\n", in.c_str());
    return String("");
  }

  char suffix = '+';  // default
  if (t.length() >= 2) {
    char c1 = t.charAt(1);
    if (c1 == '+' || c1 == ' ') {
      suffix = '+';
    } else if (c1 == '\'' || c1 == '-') {
      suffix = '-';
    } else if (c1 == '2') {
      suffix = '2';
    } else {
      return String("");
    }
  }

  // Validate combinations.
  if (axis == 'y') {
    if (suffix == '2') return String("");  // y2 not allowed by hardware
  }
  if (axis == 'z') {
    // z+, z', z2 all ok
  }
  if (axis == 'd') {
    // d+, d', d2 ok
  }

  String out;
  out.reserve(3);
  out += axis;
  if (suffix == '+') {
    out += '+';
  } else if (suffix == '-') {
    out += '\'';  // canonical prime
  } else if (suffix == '2') {
    out += '2';
  }
  return out;
}

bool CubeOri::parse_cube_move_token_(const String &token,
                                     char &face_char,
                                     int &quarter_turns) {
  if (token.length() == 0) return false;

  char f = toupper(token.charAt(0));
  if (f != 'F' && f != 'B' && f != 'R' && f != 'L' && f != 'U' && f != 'D') {
    return false;
  }

  char suffix = '+';
  if (token.length() >= 2) {
    char c1 = token.charAt(1);
    if (c1 == '+' || c1 == ' ') {
      suffix = '+';
    } else if (c1 == '\'' || c1 == '-') {
      suffix = '-';
    } else if (c1 == '2') {
      suffix = '2';
    } else {
      return false;
    }
  }

  face_char = f;
  if (suffix == '+') {
    quarter_turns = 1;
  } else if (suffix == '-') {
    quarter_turns = -1;
  } else {  // '2'
    quarter_turns = 2;
  }
  return true;
}

// ------------------------------------------------------------
// ROBOT MOVE
// ------------------------------------------------------------

bool CubeOri::robot_move(const String &move_str) {
  String canon = normalize_robot_move_token_(move_str);
  if (canon.length() == 0) {
    return false;  // invalid physical move
  }

  // Hardware callback (now returning bool)
  if (robot_cb_) {
    if (!robot_cb_(canon)) {
      serial_printf("ERR robot_cb failed for %s\n", canon.c_str());
      return false;
    }
  }

  // Update orientation if it's a whole-cube rotation.
  char axis = canon.charAt(0);
  char mod = canon.charAt(1);

  if (axis == 'y') {
    if (mod == '+') rotate_y_plus_(ori_);
    else rotate_y_minus_(ori_);
  } else if (axis == 'z') {
    if (mod == '+') rotate_z_plus_(ori_);
    else if (mod == '\'') rotate_z_minus_(ori_);
    else {  // z2
      rotate_z_plus_(ori_);
      rotate_z_plus_(ori_);
    }
  }
  // d+, d', d2 do NOT update orientation.

  if (orientation_log_str_.length() > 0) orientation_log_str_ += ' ';
  orientation_log_str_ += canon;

  return true;
}

// ------------------------------------------------------------
// CUBE MOVE (LOGICAL)
// ------------------------------------------------------------

void CubeOri::split_moves_(const String &moves_str,
                           String tokens[], int &count, int max_tokens) {
  count = 0;
  String current;

  int n = moves_str.length();
  for (int i = 0; i < n; ++i) {
    char c = moves_str.charAt(i);
    bool is_sep =
      (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == ',');

    if (is_sep) {
      if (current.length() > 0) {
        if (count < max_tokens) {
          tokens[count++] = current;
        }
        current = "";
      }
    } else {
      current += c;
    }
    if (count >= max_tokens) break;
  }
  if (current.length() > 0 && count < max_tokens) {
    tokens[count++] = current;
  }
}

bool CubeOri::cube_move(const String &moves_str) {
  // Split into tokens.
  const int MAX_TOKENS = 64;  // adjust if you need longer sequences
  String tokens[MAX_TOKENS];
  int token_count = 0;
  split_moves_(moves_str, tokens, token_count, MAX_TOKENS);

  if (token_count == 0) return true;  // nothing to do

  for (int i = 0; i < token_count; ++i) {
    String t = tokens[i];
    t.trim();
    if (t.length() == 0) continue;

    char face_char;
    int quarter_turns;

    // -----------------------
    //  INVALID TOKEN LOGGING
    // -----------------------
    if (!parse_cube_move_token_(t, face_char, quarter_turns)) {
      serial_printf("ERR not a cube move: %s\n", t.c_str());
      return false;
    }

    if (!execute_single_cube_move_(face_char, quarter_turns)) {
      serial_printf("ERR cube move failed: %s\n", t.c_str());
      return false;
    }
  }

  return true;
}

bool CubeOri::execute_single_cube_move_(char face_char, int quarter_turns) {
  // 1) Find where this logical face currently is.
  int dir_idx = find_face_direction_index_(face_char);
  if (dir_idx < 0) return false;

  // 2) Bring that direction to DOWN using only y+, y', z+, z', z2
  if (!bring_direction_to_down_(dir_idx)) return false;

  // At this point, ori_.down should be == face_char.

  // 3) Turn the D face physically (one or two quarter-turns, but we use d2).
  if (quarter_turns == 2 || quarter_turns == -2) {
    // Half-turn: direct d2
    if (!robot_move("d2")) return false;
  } else if (quarter_turns == 1) {
    if (!robot_move("d+")) return false;
  } else if (quarter_turns == -1) {
    if (!robot_move("d'")) return false;
  }

  // 4) Turning D does NOT change orientation for directions,
  //    so ori_ stays as-is here.

  return true;
}

// ------------------------------------------------------------
// FIND FACE DIRECTION / BRING DIRECTION TO DOWN
// ------------------------------------------------------------

int CubeOri::find_face_direction_index_(char face_char) const {
  if (ori_.up == face_char) return DIR_U;
  if (ori_.down == face_char) return DIR_D;
  if (ori_.front == face_char) return DIR_F;
  if (ori_.back == face_char) return DIR_B;
  if (ori_.left == face_char) return DIR_L;
  if (ori_.right == face_char) return DIR_R;
  return -1;
}

bool CubeOri::bring_direction_to_down_(int dir_index) {
  // The sequences below depend ONLY on the *direction* where the target face
  // currently sits; they work for any orientation.
  //
  // Precomputed minimal sequences (using y+, y', z+, z'):
  //
  // Start direction -> sequence to bring that direction to DOWN:
  //   U : y+ y+
  //   D : (none)
  //   F : z+ y+
  //   B : z+ y'
  //   L : y'
  //   R : y+
  //
  // These sequences were derived with the rotation definitions in
  // rotate_y_plus_/rotate_y_minus_ and rotate_z_plus_/rotate_z_minus_.

  switch (dir_index) {
    case DIR_D:
      // Already down, nothing to do.
      return true;

    case DIR_U:
      if (!robot_move("y+")) return false;
      if (!robot_move("y+")) return false;
      return true;

    case DIR_F:
      if (!robot_move("z+")) return false;
      if (!robot_move("y+")) return false;
      return true;

    case DIR_B:
      if (!robot_move("z+")) return false;
      if (!robot_move("y'")) return false;
      return true;

    case DIR_L:
      if (!robot_move("y'")) return false;
      return true;

    case DIR_R:
      if (!robot_move("y+")) return false;
      return true;

    default:
      return false;
  }
}

// ------------------------------------------------------------
// ORIENTATION ROTATIONS
// ------------------------------------------------------------

void CubeOri::rotate_y_plus_(orientation_t &o) {
  // y+ : rotate around front-back axis so that:
  //   U -> R
  //   R -> D
  //   D -> L
  //   L -> U
  //   F, B unchanged
  char old_u = o.up;
  char old_d = o.down;
  char old_l = o.left;
  char old_r = o.right;

  o.up = old_l;
  o.down = old_r;
  o.left = old_d;
  o.right = old_u;
  // front/back stay
}

void CubeOri::rotate_y_minus_(orientation_t &o) {
  // Inverse of y+:
  //   U -> L
  //   L -> D
  //   D -> R
  //   R -> U
  char old_u = o.up;
  char old_d = o.down;
  char old_l = o.left;
  char old_r = o.right;

  o.up = old_r;
  o.down = old_l;
  o.left = old_u;
  o.right = old_d;
  // front/back stay
}

void CubeOri::rotate_z_plus_(orientation_t &o) {
  // z+ : rotate around up-down axis so that:
  //   F -> R
  //   R -> B
  //   B -> L
  //   L -> F
  //   U, D unchanged
  char old_f = o.front;
  char old_b = o.back;
  char old_l = o.left;
  char old_r = o.right;

  o.front = old_l;
  o.back = old_r;
  o.left = old_b;
  o.right = old_f;
}

void CubeOri::rotate_z_minus_(orientation_t &o) {
  // Inverse of z+
  char old_f = o.front;
  char old_b = o.back;
  char old_l = o.left;
  char old_r = o.right;

  o.front = old_r;
  o.back = old_l;
  o.left = old_f;
  o.right = old_b;
}

void CubeOri::apply_rotation_in_place_(const String &move_str) {
  char axis = move_str.charAt(0);
  char mod = move_str.charAt(1);

  if (axis == 'y') {
    if (mod == '+') {
      rotate_y_plus_(ori_);
    } else {  // y'
      rotate_y_minus_(ori_);
    }
  } else if (axis == 'z') {
    if (mod == '+') {
      rotate_z_plus_(ori_);
    } else if (mod == '\'') {
      rotate_z_minus_(ori_);
    } else {  // z2
      rotate_z_plus_(ori_);
      rotate_z_plus_(ori_);
    }
  }
}

CubeOri::orientation_t CubeOri::apply_rotation_to_orientation_(
  const orientation_t &in,
  const String &move_str) {

  orientation_t out = in;
  char axis = move_str.charAt(0);
  char mod = move_str.charAt(1);

  if (axis == 'y') {
    if (mod == '+') {
      rotate_y_plus_(out);
    } else {
      rotate_y_minus_(out);
    }
  } else if (axis == 'z') {
    if (mod == '+') {
      rotate_z_plus_(out);
    } else if (mod == '\'') {
      rotate_z_minus_(out);
    } else {  // z2
      rotate_z_plus_(out);
      rotate_z_plus_(out);
    }
  }
  return out;
}

bool CubeOri::orientations_equal_(const orientation_t &a,
                                  const orientation_t &b) {
  return a.up == b.up && a.down == b.down && a.front == b.front && a.back == b.back && a.left == b.left && a.right == b.right;
}

// ------------------------------------------------------------
// RESTORE ORIENTATION  (BFS on orientation group)
// ------------------------------------------------------------

bool CubeOri::restore_orientation() {
  // Target is identity orientation:
  orientation_t target;
  target.up = 'U';
  target.down = 'D';
  target.front = 'F';
  target.back = 'B';
  target.left = 'L';
  target.right = 'R';

  if (orientations_equal_(ori_, target)) {
    return true;  // already aligned
  }

  // BFS over at most 24 states using moves: y+, y', z+, z'
  const String moves[4] = { "y+", "y'", "z+", "z'" };

  const int MAX_STATES = 24;
  orientation_t states[MAX_STATES];
  int parent_idx[MAX_STATES];
  String move_from_parent[MAX_STATES];

  int queue[MAX_STATES];
  int q_head = 0;
  int q_tail = 0;

  // Init
  states[0] = ori_;
  parent_idx[0] = -1;
  queue[q_tail++] = 0;
  int state_count = 1;

  int found_idx = -1;

  while (q_head < q_tail && found_idx < 0) {
    int cur = queue[q_head++];
    orientation_t cur_o = states[cur];

    // Expand neighbors
    for (int m = 0; m < 4; ++m) {
      orientation_t next_o = apply_rotation_to_orientation_(cur_o, moves[m]);

      // Check if we've already seen this orientation
      bool seen_bool = false;
      int seen_idx = -1;
      for (int i = 0; i < state_count; ++i) {
        if (orientations_equal_(states[i], next_o)) {
          seen_bool = true;
          seen_idx = i;
          break;
        }
      }
      if (seen_bool) continue;

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
    // Should never happen for a proper rotation group.
    return false;
  }

  // Reconstruct path from found_idx back to 0.
  String path_moves[16];  // path length will be small (<= 6)
  int path_len = 0;

  int cur = found_idx;
  while (cur >= 0 && parent_idx[cur] >= 0 && path_len < 16) {
    path_moves[path_len++] = move_from_parent[cur];
    cur = parent_idx[cur];
  }

  // Apply moves in reverse order (from start to target).
  for (int i = path_len - 1; i >= 0; --i) {
    if (!robot_move(path_moves[i])) return false;
  }

  // Now ori_ should be equal to target (identity).
  return true;
}

// ------------------------------------------------------------
// HUMAN-READABLE ORIENTATION STRING
// ------------------------------------------------------------

String CubeOri::get_orientation_string() const {
  String s;
  s.reserve(32);
  s += "U=";
  s += ori_.up;
  s += " D=";
  s += ori_.down;
  s += " F=";
  s += ori_.front;
  s += " B=";
  s += ori_.back;
  s += " L=";
  s += ori_.left;
  s += " R=";
  s += ori_.right;
  return s;
}
