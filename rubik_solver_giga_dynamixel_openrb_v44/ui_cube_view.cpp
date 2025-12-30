#include <arduino.h>
#include <vector>
#include <ctype.h>
#include "ui_cube_view.h"
#include "logging.h"
#include "ui_theme.h"  // for fonts


// ------------------------------------------------------------------
// CUBE VIEW
// ------------------------------------------------------------------
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
  lv_obj_t* cube_cont = lv_obj_create(parent);
  lv_obj_remove_style_all(cube_cont);

  lv_obj_set_size(cube_cont, net_w + padding * 2, net_h + padding * 2);
  lv_obj_set_style_bg_opa(cube_cont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(cube_cont, 0, 0);

  lv_obj_set_style_outline_width(cube_cont, 0, 0);
  lv_obj_set_style_outline_opa(cube_cont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_outline_pad(cube_cont, 0, 0);

  lv_obj_clear_flag(cube_cont, LV_OBJ_FLAG_SCROLLABLE);

  // Origin (top-left of net inside container)
  const int ox = padding;
  const int oy = padding - 8;

  // ------------------------------------------------------------------
  // Helper to create one sticker
  auto make_cell = [&](int idx, int x, int y) {
    lv_obj_t* c = lv_obj_create(cube_cont);
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
  int dY = baseY + face + v_gap;

  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      make_cell(
        27 + r * 3 + c,
        fx + c * (cell + gap),
        dY + r * (cell + gap));
    }
  }

  return cube_cont;
}

void ui_cube_view_set_colors(const String& s) {
  if (s.length() != 54) return;

  for (int i = 0; i < 54; i++) {
    if (!cube_cells[i]) continue;
    lv_obj_set_style_bg_color(cube_cells[i],
                              color_from_char(s[i]),
                              0);

    // LOG_PRINTF("setting cell %d color %c\n", i, s[i]);
  }
}

// ------------------------------------------------------------------
// PROGRESS BAR
// ------------------------------------------------------------------

static lv_obj_t* moves_root = nullptr;
static lv_obj_t* moves_cont = nullptr;

struct progress_item_t {
  String move;
  char color;
};

static std::vector<progress_item_t> progress_items;
static int progress_idx = 0;

static lv_color_t face_color(char c) {
  switch (tolower(c)) {
    case 'r': return lv_color_hex(0xFF0000);
    case 'g': return lv_color_hex(0x00A000);
    case 'b': return lv_color_hex(0x0040FF);
    case 'y': return lv_color_hex(0xFFD700);
    case 'o': return lv_color_hex(0xFF8000);
    case 'w': return lv_color_white();
    default: return lv_color_hex(0x808080);
  }
}

// ------------------------------------------------------------------
// PROGRESS BAR PUBLIC
lv_obj_t* ui_moves_progress_create(lv_obj_t* parent, int w, int h) {
  moves_root = lv_obj_create(parent);
  lv_obj_remove_style_all(moves_root);
  lv_obj_set_size(moves_root, w, h);

  lv_obj_set_style_border_width(moves_root, 1, 0);
  lv_obj_set_style_border_color(moves_root, lv_color_hex(0x404040), 0);
  lv_obj_set_style_bg_color(moves_root, lv_color_hex(0x101010), 0);
  lv_obj_set_style_bg_opa(moves_root, LV_OPA_COVER, 0);

  moves_cont = lv_obj_create(moves_root);
  lv_obj_remove_style_all(moves_cont);
  lv_obj_set_size(moves_cont, w - 8, h - 8);
  lv_obj_align(moves_cont, LV_ALIGN_CENTER, 0, 0);

  // FLEX
  lv_obj_set_flex_flow(moves_cont, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_flex_align(
    moves_cont,
    LV_FLEX_ALIGN_CENTER,
    LV_FLEX_ALIGN_CENTER,
    LV_FLEX_ALIGN_CENTER);

  // SCROLL (hidden bars)
  lv_obj_set_scroll_dir(moves_cont, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(moves_cont, LV_SCROLLBAR_MODE_OFF);

  lv_obj_set_style_pad_all(moves_cont, 6, 0);
  lv_obj_set_style_pad_gap(moves_cont, 6, 0);

  return moves_root;
}

void ui_moves_progress_set(String moves_str,
                           String colors_str) {
  // normalize case
  moves_str.toLowerCase();
  colors_str.toLowerCase();

  // split moves
  std::vector<String> tokens;
  String tmp;

  for (int i = 0; i < moves_str.length(); i++) {
    char c = moves_str[i];

    if (isspace((unsigned char)c)) {
      if (tmp.length()) {
        tokens.push_back(tmp);
        tmp = "";
      }
    } else {
      tmp += c;
    }
  }

  if (tmp.length()) {
    tokens.push_back(tmp);
  }

  // normalize colors length
  while (colors_str.length() < tokens.size()) {
    colors_str += '.';
  }
  if (colors_str.length() > tokens.size()) {
    colors_str.remove(tokens.size());
  }

  // build internal list
  progress_items.clear();
  for (int i = 0; i < (int)tokens.size(); i++) {
    progress_items.push_back({ tokens[i], colors_str[i] });
  }

  ui_moves_progress_set_index(0);
}

static std::vector<lv_obj_t*> move_labels;

void ui_moves_progress_set_index(int idx) {
  if (!moves_cont) return;

  progress_idx = idx;

  lv_obj_clean(moves_cont);
  move_labels.clear();

  const int start = max(0, progress_idx - 2);
  const int end = min((int)progress_items.size(), progress_idx + 21);

  // ---- render visible moves ----
  for (int i = start; i < end; i++) {

    lv_obj_t* lbl = lv_label_create(moves_cont);
    lv_label_set_text(lbl, progress_items[i].move.c_str());
    lv_obj_set_style_text_font(lbl, FONT_BTN_LARGE_PTR, 0);  // TODO was FONT_BTN_SMALL_PTR

    lv_color_t col;

    if (i < progress_idx) {
      col = lv_color_hex(0x808080);  // past
    } else if (i == progress_idx) {
      col = face_color(progress_items[i].color);  // current
      lv_obj_set_style_border_width(lbl, 1, 0);
      lv_obj_set_style_border_color(lbl, col, 0);
    } else {
      col = face_color(progress_items[i].color);  // future
    }

    lv_obj_set_style_text_color(lbl, col, 0);
    move_labels.push_back(lbl);
  }

  // ---- scroll so current + previous visible ----
  if (!move_labels.empty()) {

    int local_idx = progress_idx - start - 1;
    if (local_idx < 0) local_idx = 0;
    if (local_idx >= (int)move_labels.size())
      local_idx = move_labels.size() - 1;

    lv_obj_scroll_to_view(move_labels[local_idx], LV_ANIM_OFF);

    // settle at end
    if (progress_idx >= (int)progress_items.size() - 1) {
      lv_obj_scroll_to_view(move_labels.back(), LV_ANIM_OFF);
    }
  }
}
