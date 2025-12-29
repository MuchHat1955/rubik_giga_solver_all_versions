#include "ui_cube_view.h"
#include "logging.h"

static lv_obj_t* cube_cells[54] = { nullptr };

// map cube colors → LVGL colors
static lv_color_t color_from_char(char c) {
  switch (c) {
    case 'w': return lv_color_white();
    case 'y': return lv_color_hex(0xFFD700);
    case 'r': return lv_color_hex(0xFF0000);
    case 'o': return lv_color_hex(0xFF8000);
    case 'g': return lv_color_hex(0x00A000);
    case 'b': return lv_color_hex(0x0040FF);
    default:  return lv_color_hex(0x303030); // unknown
  }
}

lv_obj_t* ui_cube_view_create(lv_obj_t* parent) {
  lv_obj_t* cont = lv_obj_create(parent);
  lv_obj_remove_style_all(cont);
  lv_obj_set_size(cont, 240, 180);
  lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(cont, 0, 0);

  const int cell = 18;
  const int gap  = 2;

  // Layout net: U, L F R B, D
  // Indices follow your URFDLB mapping

  auto make_cell = [&](int idx, int x, int y) {
    lv_obj_t* c = lv_obj_create(cont);
    lv_obj_remove_style_all(c);
    lv_obj_set_size(c, cell, cell);
    lv_obj_set_pos(c, x, y);
    lv_obj_set_style_radius(c, 3, 0);
    lv_obj_set_style_bg_color(c, lv_color_hex(0x303030), 0);
    cube_cells[idx] = c;
  };

  // --- U (0..8)
  for (int r=0;r<3;r++)
    for (int c=0;c<3;c++)
      make_cell(0 + r*3 + c, 4 + c*(cell+gap), 4 + r*(cell+gap));

  // --- L F R B rows
  int baseY = 4 + 3*(cell+gap) + 6;
  int baseX[] = { 4, 4+3*(cell+gap)+6, 4+6*(cell+gap)+12, 4+9*(cell+gap)+18 };

  int faces[] = { 36, 18, 9, 45 }; // L F R B
  for (int f=0; f<4; f++) {
    for (int r=0;r<3;r++)
      for (int c=0;c<3;c++)
        make_cell(faces[f] + r*3 + c,
                  baseX[f] + c*(cell+gap),
                  baseY + r*(cell+gap));
  }

  // --- D (27..35)
  int dY = baseY + 3*(cell+gap) + 6;
  for (int r=0;r<3;r++)
    for (int c=0;c<3;c++)
      make_cell(27 + r*3 + c,
                4 + c*(cell+gap),
                dY + r*(cell+gap));

  return cont;
}

void ui_cube_view_set_colors(const String& s) {
  if (s.length() != 54) return;

  for (int i=0;i<54;i++) {
    if (!cube_cells[i]) continue;
    lv_obj_set_style_bg_color(cube_cells[i],
                              color_from_char(s[i]),
                              0);
  }
}
