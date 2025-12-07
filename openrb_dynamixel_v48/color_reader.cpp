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
void CubeColorReader::apply_slot_to_face_(char face, int slot, char color, bool mirrored) {
  int base = face_base_index_(face);
  if (base < 0) return;

  int offset = -1;

  if (!mirrored) {
    // Normal reading: top row + middle row
    switch (slot) {
      case 1: offset = 0; break;  // top-left
      case 2: offset = 1; break;  // top-center
      case 3: offset = 2; break;  // top-right
      case 6: offset = 5; break;  // mid-right
      case 5: offset = 4; break;  // mid-center
      case 4: offset = 3; break;  // mid-left
    }
  } else {
    // Mirrored (bottom band)
    // Order "231" means:
    //   slot 2 → bottom-left   (6)
    //   slot 3 → bottom-center (7)
    //   slot 1 → bottom-right  (8)
    switch (slot) {
      case 2: offset = 6; break;
      case 3: offset = 7; break;
      case 1: offset = 8; break;
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
  const char *robot_move;   // movement: "z+", "z'", "y2", …
  const char *face;         // face to read: "f","r","u","", …
  bool mirrored;            // true = bottom band (mirror), false = normal
  const char *order;        // slot order: "236541" or "231"
};

static const color_map_step_t k_color_map_steps[] = {

  // 0: read F top+middle
  { "na", "f", false, "236541" },

  // 1: z' → L
  { "z'", "l", false, "236541" },

  // 2: z' → B
  { "z'", "b", false, "236541" },

  // 3: z' → R
  { "z'", "r", false, "236541" },

  // 4: y' → R bottom band (was y2 + y+)
  { "y'", "r", true,  "231" },

  // 5: z+ → F bottom band
  { "z+", "f", true,  "231" },

  // 6: z+ → L bottom band
  { "z+", "l", true,  "231" },

  // 7: z+ → B bottom band
  { "z+", "b", true,  "231" },

  // 8: y+ → U bottom band
  { "y+", "u", true,  "231" },

  // 9: z2 → U top+middle
  { "z2", "u", false, "236541" },

  // 10: z2 → D top+middle
  { "z2", "d", false, "236541" },

  // 11: y' → D bottom band (was y2 + y+)
  { "y'", "d", true,  "231" },

  // 12: z2 final restore
  { "z2", "", false, "" }
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

  if (robot_move && robot_move[0] != '\0' && strcmp(robot_move, "na") != 0) {
    if (!ori_.robot_move(robot_move)) {
      serial_printf("err step %d: robot move %s failed\n",
                    step_index, robot_move);
      return false;
    }
  }

  if (!face || face[0] == '\0' || !order || order[0] == '\0')
    return true;

  char F = face[0];

  // Read all slots in this step
  for (int i = 0; order[i] != '\0'; i++) {

    char d = order[i];
    if (d < '1' || d > '6') continue;

    int slot = d - '0';
    char color = cb_ ? cb_(slot) : '.';

    apply_slot_to_face_(F, slot, color, mirrored);

    serial_printf("   ---%d %c%d c=%c  ",
                  step_index,
                  F,
                  slot,
                  color);
    print_face_compact_(F);
    serial_printf("\n");
  }

  serial_printf("[step] %d move=%s face=%c  ",
                step_index,
                robot_move ? robot_move : "",
                F);

  print_face_compact_(F);
  serial_printf("\n");

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
