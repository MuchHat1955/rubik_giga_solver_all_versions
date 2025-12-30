#include "ui_cube_view.h"
#include "logging.h"

static lv_obj_t* cube_cells[54] = { nullptr };

// map cube colors → LVGL colors
static lv_color_t color_from_char(char c) {
  c = tolower(c);
  switch (c) {
    case 'w': return lv_color_white();
    case 'y': return lv_color_hex(0xFFD700);
    case 'r': return lv_color_hex(0xFF0000);
    case 'o': return lv_color_hex(0xFF8000);
    case 'g': return lv_color_hex(0x00A000);
    case 'b': return lv_color_hex(0x0040FF);
    default: return lv_color_hex(0x303030);  // unknown
  }
}

lv_obj_t* ui_cube_view_create(lv_obj_t* parent) {
  // ------------------------------------------------------------------
  // Layout constants (single source of truth)
  // ------------------------------------------------------------------
  const int cell = 18;
  const int gap = 2;

  const int face = 3 * cell + 2 * gap;  // size of one 3x3 face
  const int h_gap1 = 6;                 // spacing between faces
  const int h_gap2 = 12;
  const int h_gap3 = 18;
  const int v_gap = 6;

  const int padding = 8;

  // Net size (URFDLB: U, L F R B, D)
  const int net_w = 4 * face + h_gap3;
  const int net_h = 3 * face + 2 * v_gap;

  // ------------------------------------------------------------------
  // Cube container
  // ------------------------------------------------------------------
  lv_obj_t* cont = lv_obj_create(parent);
  lv_obj_remove_style_all(cont);

  lv_obj_set_size(cont, net_w + padding * 2, net_h + padding * 2);
  lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(cont, 0, 0);
  lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

  // Origin (top-left of net inside container)
  const int ox = padding;
  const int oy = padding - 8;

  // ------------------------------------------------------------------
  // Helper to create one sticker
  // ------------------------------------------------------------------
  auto make_cell = [&](int idx, int x, int y) {
    lv_obj_t* c = lv_obj_create(cont);
    lv_obj_remove_style_all(c);

    lv_obj_set_size(c, cell, cell);
    lv_obj_set_pos(c, x, y);

    lv_obj_set_style_bg_color(c, lv_color_hex(0x303030), 0);
    lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(c, 3, 0);

    // Debug / clarity border (keep or remove)
    lv_obj_set_style_border_width(c, 1, 0);
    lv_obj_set_style_border_color(c, lv_color_black(), 0);

    cube_cells[idx] = c;
  };


  // ------------------------------------------------------------------
  // L F R B faces
  // ------------------------------------------------------------------
  int baseY = oy + face + v_gap;

  int baseX[] = {
    ox,
    ox + face + h_gap1,
    ox + 2 * face + h_gap2,
    ox + 3 * face + h_gap3
  };

  int fx = baseX[1];  // Front face X

  int faces[] = { 36, 18, 9, 45 };  // L F R B

  // ------------------------------------------------------------------
  // U face (0..8)
  // ------------------------------------------------------------------
  // --- U face (0..8), aligned above F
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      make_cell(
        0 + r * 3 + c,
        fx + c * (cell + gap),
        oy + r * (cell + gap));
    }
  }

  for (int f = 0; f < 4; f++) {
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        make_cell(
          faces[f] + r * 3 + c,
          baseX[f] + c * (cell + gap),
          baseY + r * (cell + gap));
      }
    }
  }

  // ------------------------------------------------------------------
  // D face (27..35)
  // ------------------------------------------------------------------
  // --- D face (27..35), aligned below F
  int dY = baseY + face + v_gap;

  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      make_cell(
        27 + r * 3 + c,
        fx + c * (cell + gap),
        dY + r * (cell + gap));
    }
  }

  return cont;
}

void ui_cube_view_set_colors(const String& s) {
  if (s.length() != 54) return;

  for (int i = 0; i < 54; i++) {
    if (!cube_cells[i]) continue;
    lv_obj_set_style_bg_color(cube_cells[i],
                              color_from_char(s[i]),
                              0);

    LOG_PRINTF("setting cell %d color %d\n", i, color_from_char(s[i]));
  }
}
