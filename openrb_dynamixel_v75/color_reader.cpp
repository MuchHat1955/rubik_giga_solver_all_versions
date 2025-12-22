#include "color_reader.h"
#include "color_analyzer.h"
#include "utils.h"
#include "cmd_parser.h"
#include "ori.h"

extern CubeOri ori;

// ============================================================
// Constructor
// ============================================================
CubeColorReader::CubeColorReader(CubeOri &ori, read_color_cb_t cb)
  : ori_(ori), color_sensor_cb_(cb) {
  fill_unknown_();
}

// ============================================================
// Helpers
// ============================================================
void CubeColorReader::fill_unknown_() {
  for (int i = 0; i < 54; i++) colors_justread_54[i] = '.';
}

void CubeColorReader::clear_color_reader() {
  fill_unknown_();
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

bool not_inverted = false;
bool inverted = true;

static const color_map_step_t k_color_map_steps_all[] = {

  // -----------------------------------------------------------
  // 0) none
  //      U
  //   L  F  R  B  [not inverted] F-U edge is up
  //      D
  // -----------------------------------------------------------
  { "none", "f", not_inverted, "145632" },

  // -----------------------------------------------------------
  // 1) y_minus
  //
  // --- orintentation after the step ---
  //      U
  //   F  R  B  L  [not inverted] R-U edge is up
  //      D
  // -----------------------------------------------------------
  { "y_plus", "r", not_inverted, "145632" },

  // -----------------------------------------------------------
  // 2) y_minus
  //
  // --- orintentation after the step ---
  //      U
  //   R  B  L  F  [not inverted] B-U edge is up
  //      D
  // -----------------------------------------------------------
  { "y_plus", "b", not_inverted, "145632" },

  // -----------------------------------------------------------
  // 3) y_minus
  //
  // --- orintentation after the step ---
  //      U
  //   B  L  F  R  [not inverted] L-U edge is up
  //      D
  // -----------------------------------------------------------
  { "y_plus", "l", not_inverted, "145632" },

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
  { "z_plus", "d", not_inverted, "145632" },

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
  { "z_180", "u", not_inverted, "145632" },

  // -----------------------------------------------------------
  // 12) y_180
  //
  // --- orintentation after the step --------------------------
  //      B
  //   R  D  L  U [inverted] D-F edge is down
  //      F
  // -----------------------------------------------------------
  { "y_180", "d", inverted, "231" }  // end
};

static const int k_num_color_map_steps_all =
  sizeof(k_color_map_steps_all) / sizeof(k_color_map_steps_all[0]);

static const color_map_step_t k_color_map_steps_bottom[] = {

  // -----------------------------------------------------------
  // 1) z_180
  //
  // --- orintentation after the step --------------------------
  //      D
  //   R  F  L  B  [inverted] F-U edge is down
  //      U
  //
  // -----------------------------------------------------------
  { "z_180", "f", inverted, "132" },

  // -----------------------------------------------------------
  // 2) y_plus
  //
  // --- orintentation after the step --------------------------
  //      D
  //   F  L  B  R  [inverted] U-L edge is down
  //      U
  //
  // -----------------------------------------------------------
  { "y_plus", "l", inverted, "132" },

  // -----------------------------------------------------------
  // 3) y_plus
  //
  // --- orintentation after the step --------------------------
  //      D
  //   L  B  R  F  [inverted] U-B edge is down
  //      U
  //
  // -----------------------------------------------------------
  { "y_plus", "b", inverted, "132" },

  // -----------------------------------------------------------
  // 4) y_plus
  //
  // --- orintentation after the step --------------------------
  //      D
  //   B  R  F  L  [inverted] U-R edge is down
  //      U
  //
  // -----------------------------------------------------------
  { "y_plus", "r", inverted, "132" },

  // -----------------------------------------------------------
  // 4) z_plus
  //
  // --- orintentation after the step --------------------------
  //      B
  //   U  R  D  L  [reposition]
  //      F
  //
  // -----------------------------------------------------------
  { "z_plus", "", not_inverted, "" },

  // -----------------------------------------------------------
  // 5) y_plus
  //
  // --- orintentation after the step --------------------------
  //      B
  //   R  D  L  U   [inverted] D-F edge is down
  //      F
  //
  // -----------------------------------------------------------
  { "y_plus", "d", inverted, "132" },

  // -----------------------------------------------------------
  // 5) z_180
  //
  // --- orintentation after the step --------------------------
  //      F
  //   L  D  R  U   [not inverted] D-F edge is up
  //      B
  //
  // -----------------------------------------------------------
  { "z_180", "d", not_inverted, "145632" }  // end
};

static const int k_num_color_map_steps_bottom =
  sizeof(k_color_map_steps_bottom) / sizeof(k_color_map_steps_bottom[0]);

static const color_map_step_t k_color_map_steps_centers[] = {

  // -----------------------------------------------------------
  // 1) ""
  //
  // --- orintentation after the step --------------------------
  //      U
  //   R  F  L  B  []
  //      D
  //
  // -----------------------------------------------------------
  { "none", "f", not_inverted, "5" },

  // -----------------------------------------------------------
  // 2) y_plus
  //
  // --- orintentation after the step --------------------------
  //      U
  //   F  L  B  R  []
  //      D
  //
  // -----------------------------------------------------------
  { "y_plus", "l", not_inverted, "5" },

  // -----------------------------------------------------------
  // 3) z_plus
  //
  // --- orintentation after the step --------------------------
  //      F
  //   D  L  U  R  [reposition]
  //      B
  //
  // -----------------------------------------------------------
  { "y_minus", "", not_inverted, "" },
};

static const int k_num_color_map_steps_centers =
  sizeof(k_color_map_steps_centers) / sizeof(k_color_map_steps_centers[0]);

// ============================================================
// One step processor
// ============================================================
bool CubeColorReader::process_color_scan_step_(int step_index,
                                               const char *robot_move,
                                               const char *face,
                                               bool mirrored,
                                               const char *order) {

  // Robot move (if any)
  if (robot_move && robot_move[0] != '\0' && strcmp(robot_move, "none") != 0) {
    if (!ori_.robot_move(robot_move)) {
      LOG_ERR(MOD_COLORSCAN, "robot move failed", step_index);
      LOG_VAR("robot_move", robot_move);
      return false;
    }
  }

  // No read on this step?
  if (!face || face[0] == '\0' || !order || order[0] == '\0')
    return true;

  char F = tolower(face[0]);  // ensure lowercase for face_base_index()

  // Read all slots in this step
  for (int i = 0; order[i] != '\0'; i++) {

    char d = order[i];
    if (d < '1' || d > '6') continue;

    int slot = d - '0';
    char color = color_sensor_cb_ ? color_sensor_cb_(slot) : '.';

    apply_slot_to_face_(F, slot, color, mirrored);
    LOG_INFO(MOD_COLORSCAN, "color_string_curr_face", String(F));
    LOG_VAR("color", get_justread_color_string_face(F));
  }

  // log completion of this step for this face
  LOG_INFO(MOD_COLORSCAN, "color scan step", step_index);
  LOG_VAR("robot_move", robot_move ? robot_move : "");

  get_justread_color_string_face(F);
  LOG_INFO(MOD_COLORSCAN, "cube_color_string_54", get_justread_color_string_54().c_str());
  LOG_INFO(MOD_COLORSCAN, "cube_color_string_faces", get_justread_color_string_faces().c_str());

  return true;
}

#define SCAN_MODE_FULL 0
#define SCAN_MODE_BOTTOM 1
#define SCAN_MODE_CENTERS 2

// ============================================================
// Perform  scan
// ============================================================
bool CubeColorReader::read_cube_full() {
  return read_cube(SCAN_MODE_FULL);
}
bool CubeColorReader::read_cube_bottom() {
  return read_cube(SCAN_MODE_BOTTOM);
}
bool CubeColorReader::read_cube_centers() {
  return read_cube(SCAN_MODE_CENTERS);
}

bool CubeColorReader::read_cube(int scan_mode) {
  if (scan_mode != SCAN_MODE_FULL &&    //
      scan_mode != SCAN_MODE_BOTTOM &&  //
      scan_mode != SCAN_MODE_CENTERS) {
    LOG_ERR(MOD_COLORSCAN, "invalid scan mode", scan_mode);
    return false;
  }

  if (!color_sensor_cb_) {
    LOG_ERR(MOD_COLORSCAN, "error", "no callback");
    return false;
  }
  int total_steps = k_num_color_map_steps_all;
  if (scan_mode == SCAN_MODE_BOTTOM) total_steps = k_num_color_map_steps_bottom;
  if (scan_mode == SCAN_MODE_CENTERS) total_steps = k_num_color_map_steps_centers;

  if (total_steps < 1) {
    LOG_ERR(MOD_COLORSCAN, "step count invalid", total_steps);
    return false;
  }
  String mode_string =
    (scan_mode == SCAN_MODE_BOTTOM) ? "bottom" :    //
      (scan_mode == SCAN_MODE_CENTERS) ? "centers"  //
                                       : "full";

  // Ensure orientation is clear
  LOG_INFO(MOD_COLORSCAN, "info", "orientation cleared");
  if (scan_mode != SCAN_MODE_CENTERS) ori_.clear_orientation_data();

  LOG_INFO(MOD_COLORSCAN, "color scan start", mode_string);
  LOG_VAR("total_steps", total_steps);

  if (scan_mode != SCAN_MODE_BOTTOM) clear_color_reader();
  if (scan_mode == SCAN_MODE_FULL) fill_unknown_();
  if (scan_mode == SCAN_MODE_BOTTOM) fill_solved_top_2_layers();

  const color_map_step_t *orientation_map_ptr = k_color_map_steps_all;
  if (scan_mode == SCAN_MODE_BOTTOM) orientation_map_ptr = k_color_map_steps_bottom;
  if (scan_mode == SCAN_MODE_CENTERS) orientation_map_ptr = k_color_map_steps_centers;

  // ~~~~~~~~~~~~~~~~ start scan ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (int i = 0; i < total_steps; i++) {
    const auto &s = orientation_map_ptr[i];
    if (!process_color_scan_step_(i, s.robot_move, s.face, s.mirrored, s.order)) {
      LOG_ERR(MOD_COLORSCAN, "step failed", i);
      LOG_INFO(MOD_COLORSCAN, "color scan failed", "restoring_ori");
      if (scan_mode != SCAN_MODE_CENTERS) ori_.restore_cube_orientation();
      return false;
    }
    // print_cube_colors_justread_54diagram();
  }
  // ~~~~~~~~~~~~~~~~ end scan ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  LOG_INFO(MOD_COLORSCAN, "info", "color_scan_completed");
  LOG_INFO(MOD_COLORSCAN, "color_string_54", get_justread_color_string_54().c_str());
  LOG_INFO(MOD_COLORSCAN, "color_string_faces", get_justread_color_string_faces().c_str());
  LOG_INFO(MOD_COLORSCAN, "orientation", ori.get_orientation_string().c_str());

  return true;
}

// Apply a read color for a single slot of a face:
// slot = 1..6, band semantics:
//   non-mirrored: 1,2,3 = top row (L,C,R), 4,5,6 = middle row (L,C,R)
//   mirrored:     1,2,3 = bottom row (L,C,R)
void CubeColorReader::apply_slot_to_face_(char face, int slot, char color, bool mirrored) {
  int base = face_base_index(face);
  if (base < 0) return;

  if (slot < 1 || slot > 6) {
    LOG_ERR(MOD_COLORSCAN, "error", "invalid color reader slot");
    LOG_VAR("slot", slot);
    LOG_VAR("face", face);
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
        LOG_ERR(MOD_COLORSCAN, "skipping mirrored slot", slot);
        LOG_VAR("face", face);
        return;
    }
  }
  update_color_string(face, offset, color);

  if (offset >= 0) {
    colors_justread_54[base + offset] = color;
    LOG_INFO(MOD_COLORSCAN, "read face", face);
    LOG_VAR("slot", offset + 1);
    LOG_VAR("color", color);
  }
}

// eg update_color_string('f', 0, 'r');
void CubeColorReader::update_color_string(char face, int offset, char color) {
  if (offset < 0 || offset >= 9) {
    LOG_ERR(MOD_COLORSCAN, "error", "update_color_string_invalid_params");
    LOG_VAR("face", face);
    LOG_VAR("offset", offset);
    LOG_VAR("color", color);
    return;
  }

  int base = -1;
  switch (tolower(face)) {
    case 'u': base = 0; break;
    case 'r': base = 9; break;
    case 'f': base = 18; break;
    case 'd': base = 27; break;
    case 'l': base = 36; break;
    case 'b': base = 45; break;
    default:
      LOG_ERR(MOD_COLORSCAN, "error", "update_color_string_invalid_face");
      LOG_VAR("face", face);
      return;
  }
  colors_justread_54[base + offset] = color;
}

// Print compact face state
String CubeColorReader::get_justread_color_string_face(char face) const {
  int base = face_base_index(face);
  if (base < 0) return "";
  String return_str = String(face) + "=";
  for (int i = 0; i < 9; i++) {
    return_str += String(colors_justread_54[base + i]);
  }
  return return_str;
}

// ============================================================
// Return full cube string
// ============================================================
String CubeColorReader::get_justread_color_string_54() const {
  String s;
  s.reserve(55);
  for (int i = 0; i < 54; i++)
    s += colors_justread_54[i];
  return s;
}

String CubeColorReader::get_justread_color_string_faces() const {
  String out;
  out.reserve(100);

  // Faces
  out += "{";
  out += get_justread_color_string_face('u');
  out += " ";
  out += get_justread_color_string_face('r');
  out += " ";
  out += get_justread_color_string_face('f');
  out += " ";
  out += get_justread_color_string_face('d');
  out += " ";
  out += get_justread_color_string_face('l');
  out += " ";
  out += get_justread_color_string_face('b');
  out += "}";

  // Centers (URFDLB)
  out += " centers{";

  static const int center_idx[6] = { 4, 13, 22, 31, 40, 49 };
  for (int i = 0; i < 6; i++) {
    char c = colors_justread_54[center_idx[i]];
    out += (c ? c : '.');
  }

  out += "}";

  return out;
}

// ============================================================
// SOLVED cube template
// ============================================================

// Faces:
//   U: indices  0.. 8
//   R: indices  9..17
//   F: indices 18..26
//   D: indices 27..35
//   L: indices 36..44
//   B: indices 45..53

static const char solved_54[55] =
  "WWWWWWWWW"   // U
  "RRRRRRRRR"   // R
  "GGGGGGGGG"   // F
  "YYYYYYYYY"   // D
  "OOOOOOOOO"   // L
  "BBBBBBBBB";  // B


void CubeColorReader::fill_solved_cube() {
  memcpy(colors_justread_54, solved_54, 54);
}
bool CubeColorReader::fill_solved_top_2_layers() {
  // BASED on existing colors
  // TODO read two centers first
  //TODO
  return true;
}
char color_to_face(char color) {
  char lc = tolower(color);
  if (color == 'w') return 'u';
  if (color == 'r') return 'r';
  if (color == 'g') return 'f';
  if (color == 'y') return 'd';
  if (color == 'o') return 'l';
  if (color == 'b') return 'b';
  return '.';
}
CubeColorReader color_reader(ori, read_one_color_cb);
