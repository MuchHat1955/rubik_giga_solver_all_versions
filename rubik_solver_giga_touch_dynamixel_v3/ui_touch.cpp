#include "ui_touch.h"
#include "Arduino_GigaDisplayTouch.h"
#include "Arduino_H7_Video.h"
#include <Arduino.h>
#include "servo_program.h"
#include "vertical_tune.h"
#include "action_store.h"

// ------------ private UI state ------------
static Arduino_GigaDisplayTouch *g_touch = nullptr;
static Arduino_H7_Video *g_video = nullptr;
static lv_obj_t *title_lbl = nullptr;
static lv_obj_t *content = nullptr;

// ------------ LVGL indev for touch ------------
static lv_indev_t *touch_indev = nullptr;

static bool touch_pressed = false;
static lv_point_t touch_point = { 0, 0 };

static lv_color_t text_color = lv_color_hex(0xF5F8FF);

// TODO: If your touch library has different methods,
// update the lines in touch_read_cb() accordingly.
static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  LV_UNUSED(indev);

  bool pressed = false;
  int x = 0, y = 0;

  if (g_touch) {
    GDTpoint_t points[1];
    uint8_t count = g_touch->getTouchPoints(points);
    pressed = (count > 0);
    if (pressed) {
      x = points[0].x;
      y = points[0].y;
    }
  }

  if (pressed) {
    touch_pressed = true;
    touch_point.x = x;
    touch_point.y = y;
    data->state = LV_INDEV_STATE_PRESSED;
    data->point = touch_point;
  } else {
    touch_pressed = false;
    data->state = LV_INDEV_STATE_RELEASED;
    data->point = touch_point;
  }
}

void ui_touch_attach(Arduino_H7_Video *disp, Arduino_GigaDisplayTouch *touch) {
  g_video = disp;
  g_touch = touch;
}

// ------------ UI helpers ------------
static void ensure_content() {
  if (!content) {
    content = lv_obj_create(lv_screen_active());
    lv_obj_set_pos(content, 20, 60);
    lv_obj_set_size(content, 760, 380);
    lv_obj_set_style_pad_all(content, 8, 0);
    lv_obj_set_scrollbar_mode(content, LV_SCROLLBAR_MODE_OFF);
  } else {
    lv_obj_clean(content);
  }
}

void ui_init() {
  // Register touch input driver
  // indev_drv;
  // lv_indev_drv_init(&indev_drv);
  // indev_drv.type = LV_INDEV_TYPE_POINTER;
  // indev_drv.read_cb = touch_read_cb;
  // touch_indev = lv_indev_drv_register(&indev_drv);

  // ----- Apply dark background theme -----
  lv_obj_t *scr = lv_scr_act();
  lv_color_t bg_color = lv_color_hex(0x101215);    // deep grey-black
  lv_color_t text_color = lv_color_hex(0xE0E8FF);  // light bluish-white text
  lv_obj_set_style_bg_color(scr, bg_color, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_set_style_text_color(scr, text_color, 0);

  // Title
  title_lbl = lv_label_create(lv_screen_active());
  lv_obj_align(title_lbl, LV_ALIGN_TOP_MID, 0, 6);
  lv_label_set_text(title_lbl, "Rubik Solver");
}

void ui_set_title(const char *t) {
  if (title_lbl) lv_label_set_text(title_lbl, t);
}

void ui_show_main_menu() {
  ensure_content();

  // Four large main buttons
  const char *names[] = { "Solve Cube", "Read Colors", "Random Cube", "Tests" };
  const char *keys[] = { "run_solve", "run_read", "scramble_20", "tests_menu" };

  // Dark button style
  static lv_style_t dark_btn;
  lv_style_init(&dark_btn);
  lv_style_set_bg_color(&dark_btn, lv_color_hex(0x223344));
  lv_style_set_radius(&dark_btn, 10);
  lv_style_set_border_width(&dark_btn, 0);
  lv_style_set_text_font(&dark_btn, &lv_font_montserrat_18);
  lv_style_set_text_color(&dark_btn, lv_color_hex(0xF5F8FF));

  // Create four buttons directly on "content"
  for (int i = 0; i < 4; i++) {
    lv_obj_t *btn = lv_btn_create(content);
    lv_obj_add_style(btn, &dark_btn, 0);
    lv_obj_set_size(btn, 720, 70);
    lv_obj_set_pos(btn, 20, 10 + i * 80);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, names[i]);
    lv_obj_center(label);

    // Attach click callback and pass the "key" as user data
    lv_obj_add_event_cb(
      btn,
      [](lv_event_t *e) {
        const char *key = (const char *)lv_event_get_user_data(e);
        extern void actionExecute(const std::string &key);

        if (strcmp(key, "run_solve") == 0) {
          extern void ui_show_solve();
          ui_show_solve();
          actionExecute("run_solve");
        } else if (strcmp(key, "run_read") == 0) {
          extern void ui_show_read();
          ui_show_read();
          actionExecute("run_read");
        } else if (strncmp(key, "scramble_", 9) == 0) {
          extern void ui_show_scramble();
          ui_show_scramble();
          actionExecute(key);
        } else if (strcmp(key, "tests_menu") == 0) {
          // --- Submenu example ---
          ensure_content();

          const char *items[] = { "Program Servos", "Servo Limits", "Poses",
                                  "Group Poses", "Vertical Tune-Up",
                                  "Cube Moves", "Color Read Tune", "↩ Back" };
          const char *keys2[] = { "prog_menu", "limits_menu", "poses_menu",
                                  "groups_menu", "vertical_tune", "moves_menu",
                                  "color_read_tune", "back_root" };

          for (int j = 0; j < 8; j++) {
            lv_obj_t *bb = lv_btn_create(content);
            lv_obj_add_style(bb, &dark_btn, 0);
            lv_obj_set_size(bb, 720, 56);
            lv_obj_set_pos(bb, 20, 10 + j * 60);

            lv_obj_t *ll = lv_label_create(bb);
            lv_label_set_text(ll, items[j]);
            lv_obj_center(ll);

            lv_obj_add_event_cb(
              bb,
              [](lv_event_t *ee) {
                const char *key2 = (const char *)lv_event_get_user_data(ee);
                extern void actionExecute(const std::string &key);

                if (strcmp(key2, "back_root") == 0) {
                  ui_show_main_menu();
                  return;
                }
                if (strcmp(key2, "prog_menu") == 0) {
                  runServoProgrammingWizard();
                  return;
                }
                if (strcmp(key2, "vertical_tune") == 0) {
                  runVerticalTuningUI();
                  return;
                }
                if (strcmp(key2, "moves_menu") == 0) {
                  ensure_content();
                  const char *mnames[] = { "F+", "F-", "B+", "B-", "U+", "U-", "D+",
                                           "D-", "L+", "L-", "R+", "R-" };

                  for (int k = 0; k < 12; k++) {
                    lv_obj_t *mb = lv_btn_create(content);
                    lv_obj_add_style(mb, &dark_btn, 0);
                    lv_obj_set_size(mb, 220, 50);
                    lv_obj_set_pos(mb, 20 + (k % 3) * 240, 10 + (k / 3) * 60);

                    lv_obj_t *ml = lv_label_create(mb);
                    lv_label_set_text(ml, mnames[k]);
                    lv_obj_center(ml);

                    lv_obj_add_event_cb(
                      mb,
                      [](lv_event_t *e3) {
                        const char *mk =
                          (const char *)lv_event_get_user_data(e3);
                        extern void actionExecute(const std::string &key);
                        actionExecute(std::string(mk));
                      },
                      LV_EVENT_CLICKED, (void *)mnames[k]);
                  }

                  // Back button for moves menu
                  lv_obj_t *backb = lv_btn_create(content);
                  lv_obj_add_style(backb, &dark_btn, 0);
                  lv_obj_set_size(backb, 720, 50);
                  lv_obj_set_pos(backb, 20, 10 + 5 * 60);
                  lv_obj_t *bl = lv_label_create(backb);
                  lv_label_set_text(bl, "↩ Back");
                  lv_obj_center(bl);
                  lv_obj_add_event_cb(
                    backb,
                    [](lv_event_t *) {
                      ui_show_main_menu();
                    },
                    LV_EVENT_CLICKED, NULL);
                  return;
                }

                // --- Stub for other menus ---
                lv_obj_clean(content);
                lv_obj_t *info = lv_label_create(content);
                std::string txt = std::string("Opened: ") + key2 + " (stub)";
                lv_label_set_text(info, txt.c_str());
                lv_obj_align(info, LV_ALIGN_CENTER, 0, 0);
              },
              LV_EVENT_CLICKED, (void *)keys2[j]);
          }
        }
      },
      LV_EVENT_CLICKED, (void *)keys[i]);
  }
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

// Minimal progress/stats
void drawProgressBarGeneric(int y_offset,
                            const std::vector<std::string> &items,
                            int currentIndex,
                            const std::vector<uint8_t> &states,
                            bool showCounter) {
  LV_UNUSED(y_offset);
  LV_UNUSED(states);
  LV_UNUSED(showCounter);
  static lv_obj_t *cont = nullptr;
  if (!cont) {
    cont = lv_obj_create(lv_screen_active());
    lv_obj_set_size(cont, 760, 80);
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_pad_all(cont, 6, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
  } else {
    lv_obj_clean(cont);
  }
  // window of 10
  int total = (int)items.size();
  int start = 0, maxV = 10;
  if (currentIndex >= maxV - 2) start = currentIndex - (maxV - 3);
  start = std::max(0, start);
  if (start > total - maxV) start = std::max(0, total - maxV);
  int visible = std::min(maxV, total - start);

  for (int i = 0; i < visible; i++) {
    int idx = start + i;
    lv_obj_t *b = lv_btn_create(cont);
    lv_obj_set_size(b, 64, 28);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, items[idx].c_str());
    lv_obj_center(l);
    if (idx == currentIndex) lv_obj_set_style_outline_width(b, 2, 0);
  }
}

void drawSolveStats(int y, int current, int total, unsigned long elapsed_ms) {
  LV_UNUSED(y);
  static lv_obj_t *lbl = nullptr;
  if (!lbl) {
    lbl = lv_label_create(lv_screen_active());
    lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 36);
  }
  float pct = (total > 0) ? (100.f * (float)(current + 1) / total) : 0.f;
  char buf[96];
  snprintf(buf, sizeof(buf), "Move %d/%d (%.0f%%)  %.1fs",
           current + 1, total, pct, elapsed_ms / 1000.0f);
  lv_label_set_text(lbl, buf);
}

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
                       const std::string &highlight) {
  static lv_obj_t *cont = nullptr;
  if (!cont) {
    cont = lv_obj_create(lv_screen_active());
    lv_obj_set_size(cont, 700, 260);
    lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, 100);
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
        lv_obj_t *box = lv_obj_create(cont);
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
    lv_obj_t *sel = lv_obj_create(cont);
    lv_obj_set_style_bg_opa(sel, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(sel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(sel, 2, 0);
    lv_obj_set_pos(sel, fx + col * (22 + 3) - 2, fy + row * (22 + 3) - 2);
    lv_obj_set_size(sel, 22 + 4, 22 + 4);
  }
}

void ui_loop_once() {
  lv_timer_handler();
}
