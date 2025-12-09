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
    serial_printf("ERR ColorReader: invalid slot %d for face %c\n", slot, face);
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
        serial_printf("WARN ColorReader: mirrored with slot %d on face %c\n",
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
// Mapping table (with explicit mirrored flag)
// ============================================================
struct color_map_step_t {
  const char *robot_move;  // movement: "ud_axis_counterclockwise", "ud_axis_clockwise", "fb_axis_counterclockwise", "ud_axis_180", …
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
  // 1) ud_axis_clockwise
  //      U
  //   F  R  B  L  [not inverted] R-U edge is up
  //      D
  // -----------------------------------------------------------
  { "ud_axis_clockwise", "r", not_inverted, "236541" },

  // -----------------------------------------------------------
  // 2) ud_axis_clockwise
  //      U
  //   R  B  L  F  [not inverted] B-U edge is up
  //      D
  // -----------------------------------------------------------
  { "ud_axis_clockwise", "b", not_inverted, "236541" },

  // -----------------------------------------------------------
  // 3) ud_axis_clockwise
  //      U
  //   B  L  F  R  [not inverted] L-U edge is up
  //      D
  // -----------------------------------------------------------
  { "ud_axis_clockwise", "l", not_inverted, "236541" },

  // -----------------------------------------------------------
  // 4) fb_axis_180
  //      D
  //   F  L  B  R  [inverted] L-U edge is down
  //      U
  // -----------------------------------------------------------
  { "fb_axis_180", "l", inverted, "231" },

  // -----------------------------------------------------------
  // 5) ud_axis_clockwise
  //      D
  //   L  B  R  F  [inverted] B-U edge is down
  //      U
  // -----------------------------------------------------------
  { "ud_axis_clockwise", "b", inverted, "231" },

  // -----------------------------------------------------------
  // 6) ud_axis_clockwise
  //      D
  //   B  R  F  L  [inverted] R-U edge is down
  //      U
  // -----------------------------------------------------------
  { "ud_axis_clockwise", "r", inverted, "231" },

  // -----------------------------------------------------------
  // 7) ud_axis_clockwise
  //      D
  //   R  F  L  B  [inverted] F-U edge is down
  //      U
  // -----------------------------------------------------------
  { "ud_axis_clockwise", "f", inverted, "231" },

  // -----------------------------------------------------------
  // 8) fb_axis_clockwise
  //      R
  //   U  F  D  B  [reposition]
  //      L
  // -----------------------------------------------------------
  { "fb_axis_clockwise", "", not_inverted, "" },

  // -----------------------------------------------------------
  // 9) ud_axis_clockwise
  //      R
  //   F  D  B  U  [reposition]
  //      L
  // -----------------------------------------------------------
  { "ud_axis_clockwise", "", not_inverted, "" },

  // -----------------------------------------------------------
  // 9) fb_axis_clockwise,
  //      F
  //   L  D  R  U  [not inverted] D-F edge is up
  //      B
  //   -----------------------------------------------------------
  { "fb_axis_clockwise", "d", not_inverted, "236541" },

  // -----------------------------------------------------------
  // 10) ud_axis_180
  //      F
  //   R  U  L  D  [inverted] U-B edge is down
  //      B
  // -----------------------------------------------------------
  { "ud_axis_180", "u", inverted, "231" },

  // -----------------------------------------------------------
  // 11) fb_axis_180
  //      B
  //   L  U  R  D  [not inverted] U-B edge is up
  //      F
  // -----------------------------------------------------------
  { "fb_axis_180", "u", not_inverted, "236541" },

  // -----------------------------------------------------------
  // 12) ud_axis_180
  //      B
  //   R  D  L  U [inverted] D-F edge is down
  //      F
  // -----------------------------------------------------------
  { "ud_axis_180", "d", inverted, "231" },

  // -----------------------------------------------------------
  // 13) ud_axis_clockwise
  //      B
  //   D  L  U  R  [reposition]
  //      F
  // -----------------------------------------------------------
  { "ud_axis_clockwise", "", not_inverted, "" },

  // -----------------------------------------------------------
  // 14) fb_axis_counterclockwise
  //      U
  //   B  L  F  R  [reposition]
  //      D
  // -----------------------------------------------------------
  { "fb_axis_counterclockwise", "", not_inverted, "" },

  // -----------------------------------------------------------
  // 15) ud_axis_clockwise
  //      U
  //   L  F  R  B  [reposition]
  //      D
  // -----------------------------------------------------------
  { "ud_axis_clockwise", "", not_inverted, "" },
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
  serial_printf("[step] %d move=%s ",
                step_index,
                robot_move ? robot_move : "",
                F);
  print_face_compact_(F);
  serial_printf("\n");
  ori.print_orientation_string();

  return true;
}

// ============================================================
// Perform full scan
// ============================================================
bool CubeColorReader::read_full_cube() {
  if (!cb_) {
    serial_printf("err ColorReader: no callback\n");
    return false;
  }

  fill_unknown_();

  // Ensure starting orientation
  if (!ori_.restore_cube_orientation()) {
    serial_printf("err ColorReader: initial restore failed\n");
    return false;
  }

  for (int i = 0; i < k_num_color_map_steps; i++) {
    const auto &s = k_color_map_steps[i];
    if (!process_step_(i, s.robot_move, s.face, s.mirrored, s.order)) {
      serial_printf("err ColorReader: step %d failed\n", i);
      ori_.restore_cube_orientation();
      return false;
    }
  }

  // Final restore
  if (!ori_.restore_cube_orientation()) {
    serial_printf("err ColorReader: final restore failed\n");
    return false;
  }

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

CubeColorReader color_reader(ori, read_one_color_cb);
