/* NOT NEEDED ANYMORE

#include "ui_lvgl.h"
#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino.h>

// ======= Encoder pins (must match .ino) =======
#define ENC_A 6
#define ENC_B 7
#define ENC_BTN 8

static GigaDisplay_GFX Display;
static lv_obj_t* title_lbl = nullptr;

static int enc_lastA = 1, enc_lastB = 1;
static int enc_diff = 0;
static bool enc_btn_last = true;
static bool enc_btn_pressed_flag = false;

bool solvingActive = false;
bool readingActive = false;

// ============ LVGL Display buffer ============
static lv_color_t buf1[800 * 60];  // strip buffer
static lv_display_t* disp = nullptr;

/*
static void flush_cb(lv_display_t *d, const lv_area_t *area, uint8_t *px_map) {
  // Delegate to Arduino_GigaDisplay_GFX
  // The GigaDisplay_GFX library handles LVGL internally in latest versions,
  // but we implement a generic push for compatibility.
  Display.drawBitmap(area->x1, area->y1, (uint16_t*)px_map,
                     area->x2 - area->x1 + 1, area->y2 - area->y1 + 1);
  lv_display_flush_ready(d);
}

void ui_init() {
  Display.begin();  // init shield
  lv_init();

  disp = lv_display_create(800, 480);
  // lv_display_set_flush_cb(disp, flush_cb);
  // lv_display_set_buffers(disp, buf1, NULL, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

  // Title
  title_lbl = lv_label_create(lv_screen_active());
  lv_label_set_text(title_lbl, "Rubik Solver");
  lv_obj_align(title_lbl, LV_ALIGN_TOP_MID, 0, 6);
}

void ui_set_title(const char* t) {
  if (title_lbl) lv_label_set_text(title_lbl, t);
}

static void draw_list(const char* const* items, int n) {
  // Clear old
  lv_obj_clean(lv_screen_active());
  // Re-add title
  title_lbl = lv_label_create(lv_screen_active());
  lv_label_set_text(title_lbl, "Rubik Solver (GIGA)");
  lv_obj_align(title_lbl, LV_ALIGN_TOP_MID, 0, 6);

  // Simple menu as list of buttons
  lv_obj_t* col = lv_obj_create(lv_screen_active());
  lv_obj_set_size(col, 760, 360);
  lv_obj_align(col, LV_ALIGN_TOP_MID, 0, 40);
  lv_obj_set_style_pad_all(col, 8, 0);
  lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);

  for (int i = 0; i < n; i++) {
    lv_obj_t* b = lv_btn_create(col);
    lv_obj_set_size(b, 720, 48);
    lv_obj_t* lbl = lv_label_create(b);
    lv_label_set_text(lbl, items[i]);
    lv_obj_center(lbl);
  }
}

void ui_show_main_menu() {
  static const char* items[] = {
    "🧩 Solve Cube",
    "🎨 Read Colors",
    "🔀 Random Cube",
    "🧪 Tests (poses/limits)"
  };
  draw_list(items, 4);
}

void ui_show_solve() {
  lv_obj_clean(lv_screen_active());
  title_lbl = lv_label_create(lv_screen_active());
  lv_label_set_text(title_lbl, "Solve Cube");
  lv_obj_align(title_lbl, LV_ALIGN_TOP_MID, 0, 6);
}

void ui_show_read() {
  lv_obj_clean(lv_screen_active());
  title_lbl = lv_label_create(lv_screen_active());
  lv_label_set_text(title_lbl, "Read Colors");
  lv_obj_align(title_lbl, LV_ALIGN_TOP_MID, 0, 6);
}

void ui_show_scramble() {
  lv_obj_clean(lv_screen_active());
  title_lbl = lv_label_create(lv_screen_active());
  lv_label_set_text(title_lbl, "Scramble");
  lv_obj_align(title_lbl, LV_ALIGN_TOP_MID, 0, 6);
}

// Minimal progress bar & stats
void drawProgressBarGeneric(int y_offset,
                            const std::vector<String>& items,
                            int currentIndex,
                            const std::vector<uint8_t>& states,
                            bool showCounter) {
  (void)showCounter;
  static lv_obj_t* cont = nullptr;
  if (!cont) {
    cont = lv_obj_create(lv_screen_active());
    lv_obj_set_size(cont, 760, 80);
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(cont, 6, 0);
  } else {
    lv_obj_clean(cont);
  }

  // up to 10 items visible
  int start = 0;
  int maxV = 10;
  if (currentIndex >= maxV - 2) start = currentIndex - (maxV - 3);
  start = std::max(0, start);
  if (start > (int)items.size() - maxV) start = std::max(0, (int)items.size() - maxV);
  int visible = std::min(maxV, (int)items.size() - start);

  for (int i = 0; i < visible; i++) {
    int idx = start + i;
    lv_obj_t* b = lv_btn_create(cont);
    lv_obj_set_size(b, 64, 28);
    lv_obj_t* l = lv_label_create(b);
    lv_label_set_text(l, items[idx].c_str());
    lv_obj_center(l);
    if (idx == currentIndex) lv_obj_set_style_outline_width(b, 2, 0);
  }
}

void drawSolveStats(int y, int current, int total, unsigned long elapsed_ms) {
  (void)y;
  static lv_obj_t* lbl = nullptr;
  if (!lbl) {
    lbl = lv_label_create(lv_screen_active());
    lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 40);
  }
  float pct = (total > 0) ? (100.f * (float)(current + 1) / total) : 0.f;
  char buf[80];
  snprintf(buf, sizeof(buf), "Move %d/%d (%.0f%%)  %.1fs",
           current + 1, total, pct, elapsed_ms / 1000.0f);
  lv_label_set_text(lbl, buf);
}

// Cube draw
lv_color_t cube_color_from_char(char c) {
  switch (toupper(c)) {
    case 'W': return lv_color_hex(0xFFFFFF);
    case 'R': return lv_color_hex(0xE74C3C);
    case 'G': return lv_color_hex(0x2ECC71);
    case 'Y': return lv_color_hex(0xF1C40F);
    case 'O': return lv_color_hex(0xE67E22);
    case 'B': return lv_color_hex(0x3498DB);
    case 'E': return lv_color_hex(0xFF00FF);
    default: return lv_color_hex(0x2A2E33);
  }
}

void cube_draw_to_lvgl(const char face[6][9], const char faceToColor[6],
                       const std::string& highlight) {
  static lv_obj_t* cont = nullptr;
  if (!cont) {
    cont = lv_obj_create(lv_screen_active());
    lv_obj_set_size(cont, 700, 260);
    lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, 80);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
  }
  lv_obj_clean(cont);

  auto resolve = [&](int fIdx, char c) -> lv_color_t {
    switch (toupper(c)) {
      case 'U': return cube_color_from_char(faceToColor[0]);
      case 'R': return cube_color_from_char(faceToColor[1]);
      case 'F': return cube_color_from_char(faceToColor[2]);
      case 'D': return cube_color_from_char(faceToColor[3]);
      case 'L': return cube_color_from_char(faceToColor[4]);
      case 'B': return cube_color_from_char(faceToColor[5]);
      default: return cube_color_from_char(c);
    }
  };

  const int cell = 22, pad = 3, gap = 8;
  const int cw = 3 * cell + 2 * pad, ch = cw;
  const int total = cw * 4 + gap * 3;
  const int sx = (700 - total) / 2, sy = 0;

  auto fidx = [](char c) {
    switch (toupper(c)) {
      case 'U': return 0;
      case 'R': return 1;
      case 'F': return 2;
      case 'D': return 3;
      case 'L': return 4;
      case 'B': return 5;
      default: return -1;
    }
  };
  auto origin = [&](char f) {
    switch (toupper(f)) {
      case 'U': return std::pair<int, int>{ sx + cw + gap, sy };
      case 'L': return std::pair<int, int>{ sx, sy + ch + gap };
      case 'F': return std::pair<int, int>{ sx + cw + gap, sy + ch + gap };
      case 'R': return std::pair<int, int>{ sx + 2 * (cw + gap), sy + ch + gap };
      case 'B': return std::pair<int, int>{ sx + 3 * (cw + gap), sy + ch + gap };
      case 'D': return std::pair<int, int>{ sx + cw + gap, sy + 2 * (ch + gap) };
      default: return std::pair<int, int>{ sx, sy };
    }
  };

  const char faces[] = { 'U', 'L', 'F', 'R', 'B', 'D' };
  for (char fc : faces) {
    int fi = fidx(fc);
    auto [fx, fy] = origin(fc);
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++) {
        int idx = r * 3 + c;
        lv_obj_t* box = lv_obj_create(cont);
        lv_obj_set_style_bg_color(box, resolve(fi, face[fi][idx]), 0);
        lv_obj_set_style_border_width(box, 1, 0);
        lv_obj_set_style_border_color(box, lv_color_hex(0x000000), 0);
        lv_obj_set_pos(box, fx + c * (cell + pad), fy + r * (cell + pad));
        lv_obj_set_size(box, cell, cell);
      }
  }
  if (!highlight.empty() && highlight.size() >= 2) {
    char fch = highlight[0];
    int idx = 4;
    auto [fx, fy] = origin(fch);
    int row = idx / 3, col = idx % 3;
    lv_obj_t* sel = lv_obj_create(cont);
    lv_obj_set_style_bg_opa(sel, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(sel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(sel, 2, 0);
    lv_obj_set_pos(sel, fx + col * (22 + 3) - 2, fy + row * (22 + 3) - 2);
    lv_obj_set_size(sel, 22 + 4, 22 + 4);
  }
}

// ======= Encoder helpers =======
/*
int enc_get_diff(){
  int A = digitalRead(ENC_A);
  int B = digitalRead(ENC_B);
  int diff=0;
  if (A != enc_lastA) {
    diff = (B != A) ? +1 : -1;
    enc_lastA = A;
  }
  enc_btn_pressed_flag = (digitalRead(ENC_BTN)==LOW);
  return diff;
}
bool enc_btn_pressed(){ return enc_btn_pressed_flag; }

void ui_loop_once() {
  // Pump LVGL
  lv_timer_handler();
  // update diff just so flows can poll it if needed
  (void)enc_get_diff();
}
*/