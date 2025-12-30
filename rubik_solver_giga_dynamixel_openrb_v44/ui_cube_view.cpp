#include <vector>
#include "ui_cube_view.h"
#include "logging.h"
#include "ui_theme.h" // for fonts

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

  lv_obj_set_style_outline_width(cont, 0, 0);
  lv_obj_set_style_outline_opa(cont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_outline_pad(cont, 0, 0);

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

//-----------------------------------------------------------------------------------------------------
// PROGRESS BAR

static lv_obj_t* root = nullptr;
static lv_obj_t* cont = nullptr;

struct move_item_t {
  String move;
  char color;
};

static std::vector<move_item_t> moves;
static int progress_idx = 0;

static lv_color_t face_color(char c) {
  switch (tolower(c)) {
    case 'r': return lv_color_hex(0xFF0000);
    case 'g': return lv_color_hex(0x00A000);
    case 'b': return lv_color_hex(0x0040FF);
    case 'y': return lv_color_hex(0xFFD700);
    case 'o': return lv_color_hex(0xFF8000);
    case 'w': return lv_color_white();
    default:  return lv_color_hex(0x808080);
  }
}

lv_obj_t* ui_moves_progress_create(lv_obj_t* parent, int w, int h) {
  root = lv_obj_create(parent);
  lv_obj_remove_style_all(root);
  lv_obj_set_size(root, w, h);

  lv_obj_set_style_border_width(root, 1, 0);
  lv_obj_set_style_border_color(root, lv_color_hex(0x404040), 0);
  lv_obj_set_style_bg_color(root, lv_color_hex(0x101010), 0);
  lv_obj_set_style_bg_opa(root, LV_OPA_COVER, 0);

  cont = lv_obj_create(root);
  lv_obj_remove_style_all(cont);
  lv_obj_set_size(cont, w - 8, h - 8);
  lv_obj_align(cont, LV_ALIGN_CENTER, 0, 0);

  lv_obj_set_scroll_dir(cont, LV_DIR_VER);
  lv_obj_set_style_pad_gap(cont, 6, 0);
  lv_obj_set_style_pad_all(cont, 6, 0);
  lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW_WRAP);

  return root;
}

void ui_moves_progress_set(const String& moves_str,
                           const String& colors_str) {
  moves.clear();
  progress_idx = 0;
  lv_obj_clean(cont);

  std::vector<String> tokens;
  String tmp;

  // split by space
  for (char c : moves_str) {
    if (c == ' ') {
      if (tmp.length()) tokens.push_back(tmp);
      tmp = "";
    } else tmp += c;
  }
  if (tmp.length()) tokens.push_back(tmp);

  if ((int)tokens.size() != colors_str.length()) return;

  for (int i = 0; i < (int)tokens.size(); i++) {
    moves.push_back({ tokens[i], colors_str[i] });
  }

  ui_moves_progress_set_index(0);
}

void ui_moves_progress_set_index(int idx) {
  if (!cont) return;
  progress_idx = idx;

  lv_obj_clean(cont);

  const int start =
    std::max(0, progress_idx - 2);

  for (int i = start; i < (int)moves.size(); i++) {

    // stop if too many future moves
    if (i > progress_idx + 20) break;

    lv_obj_t* lbl = lv_label_create(cont);
    lv_label_set_text(lbl, moves[i].move.c_str());

    lv_color_t col;

    if (i < progress_idx) {
      // past
      col = lv_color_hex(0x808080);
    } else if (i == progress_idx) {
      // current
      col = face_color(moves[i].color);
      lv_obj_set_style_border_width(lbl, 1, 0);
      lv_obj_set_style_border_color(lbl, col, 0);
    } else {
      // future
      col = face_color(moves[i].color);
    }

    lv_obj_set_style_text_color(lbl, col, 0);
    lv_obj_set_style_text_font(lbl, FONT_BTN_SMALL_PTR, 0);
  }
}



