// ---------------- See a desciption of the project at the end of this file  ----------------
#include <Arduino.h>
#include "Arduino_H7_Video.h"
#include <lvgl.h>
#include <Arduino_GigaDisplayTouch.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include "logging.h"
#include "param_store.h"
#include "servos.h"

// ----------------------------------------------------------
//                  NOTES
// ----------------------------------------------------------
// --- update "C:\Users\gigel\AppData\Local\Arduino15\packages\arduino\hardware\mbed_giga\4.4.1\libraries\Arduino_H7_Video\src\lv_conf_8.h" for fonts
// --- update "C:\Users\gigel\AppData\Local\Arduino15\packages\arduino\hardware\mbed_giga\4.4.1\libraries\Arduino_H7_Video\src\lv_conf_9.h" for fonts
// --- see a desciption of the project at the end of this file  ----------------

// ----------------------------------------------------------
//                  CONFIGURATION & COLORS
// ----------------------------------------------------------
static const int SCREEN_W = 800;
static const int SCREEN_H = 480;

/*
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_22 1
#define LV_FONT_MONTSERRAT_26 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_MONTSERRAT_30 1
*/

// Font sizes
#define FONT_TITLE_PTR &lv_font_montserrat_40
#define FONT_BTN_LARGE_PTR &lv_font_montserrat_30
#define FONT_BTN_SMALL_PTR &lv_font_montserrat_26
#define FONT_SUBTITLE_PTR &lv_font_montserrat_26
#define FONT_FOOT_PTR &lv_font_montserrat_26

// base colors
static const lv_color_t COLOR_BG = lv_color_hex(0x000000);
static const lv_color_t COLOR_TEXT = lv_color_hex(0xE0E0E0);
static const lv_color_t COLOR_BTN_TEXT = lv_color_hex(0xE0E0E0);

// Type-specific colors
static const lv_color_t COLOR_BTN_ACTION = lv_color_hex(0x2070FF);  // blue
// static const lv_color_t COLOR_BTN_MENU = lv_color_hex(0x2070FF);    // blue
static const lv_color_t COLOR_BTN_MENU = lv_color_hex(0x40C040);    // green
static const lv_color_t COLOR_BTN_NUM = lv_color_hex(0xFF4000);     // orange
static const lv_color_t COLOR_BTN_STATUS = lv_color_hex(0xE0E0E0);  // gray
// static const lv_color_t COLOR_BTN_BACK = lv_color_hex(0xFF4040);    //red
static const lv_color_t COLOR_BTN_BACK = lv_color_hex(0xC04040);  // reddish tone for back

static const int HORIZONTAL_SPACING = 20;
static const int VERTICAL_SPACING = 70;
static const int CORNERS = 20;  // was 40

// Refresh interval (in seconds)
#define UI_REFRESH_INTERVAL_SEC 2  // <-- change as needed

// ----------------------------------------------------------
//                   GLOBAL STATE / FORWARD DECLS
// ----------------------------------------------------------
static String currentMenu = "main";
StaticJsonDocument<12288> menuDoc;

static lv_obj_t *footLbl = nullptr;
static std::map<String, lv_obj_t *> numLabels;
static std::map<String, lv_obj_t *> statusWidgets;

lv_obj_t *selected_num_box = nullptr;
lv_style_t style_num_selected;     // orange border + light background
lv_style_t style_num_btn_active;   // filled yellow (selected)
lv_style_t style_num_btn_pressed;  // bright flash on press

void buildMenu(const char *menuName);
void buttonAction(const char *key);
int incrementValue(const char *key, int delta);
void updateStatus(const char *key, const char *value, const char *colorName);

// ----------------------------------------------------------
//                    STUBS
// ----------------------------------------------------------

int getParamValue(const char *name) {
  return 0;
}
void setParamValue(const char *name, int val) {
}
const char *getStatusText(const char *name) {
  return "na";
}
const char *getStatusColor(const char *name) {
  return "green";
}

// ----------------------------------------------------------
//                    HELPER UTILITIES
// ----------------------------------------------------------
static lv_color_t colorFromName(const char *name) {
  if (!name) return COLOR_TEXT;
  if (!strcasecmp(name, "green")) return lv_color_hex(0x00FF00);
  if (!strcasecmp(name, "red")) return lv_color_hex(0xFF4040);
  if (!strcasecmp(name, "yellow")) return lv_color_hex(0xFFFF00);
  if (!strcasecmp(name, "orange")) return lv_color_hex(0xFFA000);
  if (!strcasecmp(name, "gray")) return lv_color_hex(0xE0E0E0);
  return COLOR_TEXT;
}

static void setFooter(const char *msg) {
  if (footLbl) lv_label_set_text(footLbl, msg);
}

// Increment storage (simple)
int incrementValue(const char *key, int delta) {
  static std::map<String, int> values;
  values[key] += delta;
  if (values[key] < 0) values[key] = 0;
  if (values[key] > 9999) values[key] = 9999;
  return values[key];
}

// Update status label or button text/color
void updateStatus(const char *key, const char *value, const char *colorName) {
  if (!statusWidgets.count(key)) return;
  lv_obj_t *obj = statusWidgets[key];
  lv_color_t col = colorFromName(colorName);
  lv_label_set_text(obj, value);
  lv_obj_set_style_text_color(obj, col, 0);
}

// ----------------------------------------------------------
//                     BUTTON ACTION LOGIC
// ----------------------------------------------------------
void buttonAction(const char *key) {
  LOG_SECTION_START_VAR("buttonAction", "key", key);

  if (menuDoc.containsKey(key)) {  // navigate to submenu
    currentMenu = key;
    setFooter((String("Switch menu -> ") + key).c_str());
    buildMenu(currentMenu.c_str());
  } else if (strcmp(key, "main") == 0) {  // explicit back to main
    currentMenu = "main";
    setFooter("back to main menu");
    buildMenu(currentMenu.c_str());
  } else {
    setFooter((String("Action: ") + key).c_str());
  }

  LOG_SECTION_END();
}

static void apply_pair_selected_styles(lv_obj_t *numBox, bool on) {
  // assumes children: [0] minus button, [1] value label, [2] plus button
  if (!numBox) return;
  lv_obj_t *btnMinus = lv_obj_get_child(numBox, 0);
  lv_obj_t *btnPlus = lv_obj_get_child(numBox, 2);
  if (btnMinus) {
    if (on) lv_obj_add_style(btnMinus, &style_num_btn_active, 0);
    else lv_obj_remove_style(btnMinus, &style_num_btn_active, 0);
  }
  if (btnPlus) {
    if (on) lv_obj_add_style(btnPlus, &style_num_btn_active, 0);
    else lv_obj_remove_style(btnPlus, &style_num_btn_active, 0);
  }
}

static void select_num_pair(lv_obj_t *numBox, bool toggle) {
  if (selected_num_box == numBox) {
    if (toggle) {
      apply_pair_selected_styles(selected_num_box, false);
      selected_num_box = nullptr;
    }
    return;
  }
  if (selected_num_box) apply_pair_selected_styles(selected_num_box, false);
  selected_num_box = numBox;
  apply_pair_selected_styles(selected_num_box, true);
}


// ----------------------------------------------------------
//                     MENU BUILDER
// ----------------------------------------------------------
void buildMenu(const char *menuName) {

  LOG_SECTION_START_VAR("buildMenu", "menu", menuName);

  lv_obj_t *scr = lv_scr_act();
  if (!scr || !menuDoc.containsKey(menuName)) {
    LOG_SECTION_END();
    return;
  }

  lv_obj_clean(scr);
  statusWidgets.clear();
  numLabels.clear();
  JsonObject root = menuDoc[menuName];
  if (root.isNull()) {
    LOG_SECTION_END();
    return;
  }

  const char *title = root["title"] | "";
  const char *footer = root["footer"] | "";
  const char *equalColumns = root["equal columns"] | "";  // "all" or "last"
  int columns = root["columns"] | 1;
  JsonArray rows = root["rows"].as<JsonArray>();

  lv_obj_set_style_bg_color(scr, COLOR_BG, 0);
  lv_obj_set_style_text_color(scr, COLOR_TEXT, 0);

  // ---------- TITLE ----------
  lv_obj_t *titleLbl = lv_label_create(scr);
  lv_label_set_text(titleLbl, title);
  lv_obj_set_style_text_font(titleLbl, FONT_TITLE_PTR, 0);
  lv_obj_align(titleLbl, LV_ALIGN_TOP_MID, 0, 6);

  const lv_font_t *btnFont = FONT_BTN_LARGE_PTR;

  // ---------- SCROLLABLE CONTAINER ----------
  lv_obj_t *cont = lv_obj_create(scr);
  lv_obj_set_scroll_dir(cont, LV_DIR_VER);
  lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(cont, 0, 0);

  lv_obj_update_layout(titleLbl);
  int title_h = lv_obj_get_height(titleLbl);
  lv_obj_set_size(cont, SCREEN_W, SCREEN_H - (title_h + 80));
  lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, title_h + 20);

  // ---------- DYNAMIC COLUMN WIDTH ----------
  std::vector<int> colWidths(columns, 0);
  const int textPad = 20;

  lv_obj_t *measLbl = lv_label_create(scr);
  lv_obj_add_flag(measLbl, LV_OBJ_FLAG_HIDDEN);

  for (JsonArray row : rows) {
    for (int ci = 0; ci < (int)row.size() && ci < columns; ci++) {
      JsonObject it = row[ci];
      const char *txt = it["text"] | "";
      const char *type = it["type"] | "text";
      int extraPad = textPad;
      if (strcmp(type, "num") == 0) {
        txt = "-0000+";
        extraPad = 20;
      }
      lv_point_t sz;
      lv_txt_get_size(&sz, txt, btnFont, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
      int width = sz.x + textPad + extraPad;
      if (width > colWidths[ci]) colWidths[ci] = width;
    }
  }
  lv_obj_del(measLbl);

  if (strcasecmp(equalColumns, "all") == 0) {
    int maxW = *std::max_element(colWidths.begin(), colWidths.end());
    for (auto &w : colWidths) w = maxW;
  } else if (strcasecmp(equalColumns, "last") == 0 && columns > 1) {
    int maxW = *std::max_element(colWidths.begin() + 1, colWidths.end());
    for (int i = 1; i < columns; i++) colWidths[i] = maxW;
  }

  int totalW = 0;
  for (int w : colWidths) totalW += w + 10;
  if (totalW > SCREEN_W - 20) totalW = SCREEN_W - 20;
  int xStartbase = (SCREEN_W - totalW) / 2;

  // ---------- BUILD ROWS ----------
  int rowH = 65;
  int contH = lv_obj_get_height(cont);
  int fullRows = contH / rowH;
  if (contH % rowH > rowH / 2) fullRows++;
  int adjustedRowH = contH / fullRows;

  static lv_obj_t *selected_num_box = nullptr;  // current selected numeric box
  extern lv_style_t style_num_selected;         // highlight style

  int y = 0;
  for (JsonArray row : rows) {
    int numCols = row.size();
    int x = xStartbase;

    for (int ci = 0; ci < numCols && ci < columns; ci++) {
      JsonObject it = row[ci];
      const char *txt = it["text"] | "";
      const char *type = it["type"] | "text";
      const char *key = it["key"] | "";
      int colW = colWidths[ci];

      // ---------- TEXT ----------
      if (strcmp(type, "text") == 0) {
        // transparent cell container (no inherited border/background)
        lv_obj_t *cell = lv_obj_create(cont);
        lv_obj_remove_style_all(cell);  // <-- remove all theme styles
        lv_obj_set_size(cell, colW, 48);
        lv_obj_set_pos(cell, x, y);
        lv_obj_set_style_bg_opa(cell, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(cell, 0, 0);

        // centered label
        lv_obj_t *lbl = lv_label_create(cell);
        lv_label_set_text(lbl, txt);
        lv_obj_set_style_text_font(lbl, btnFont, 0);
        lv_obj_set_style_text_color(lbl, COLOR_BTN_TEXT, 0);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_opa(lbl, LV_OPA_COVER, 0);  // solid text, no alpha blend
        lv_obj_center(lbl);
      }

      // ---------- ACTION / MENU ----------
      else if (strcmp(type, "action") == 0 || strcmp(type, "menu") == 0) {
        lv_obj_t *btn = lv_btn_create(cont);
        lv_obj_set_size(btn, colW, 48);
        lv_obj_set_pos(btn, x, y);

        lv_color_t colorBtn = COLOR_BTN_ACTION;
        if (strcmp(type, "menu") == 0) colorBtn = COLOR_BTN_MENU;
        if (strcasecmp(txt, "back") == 0) colorBtn = COLOR_BTN_BACK;

        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_radius(btn, CORNERS, 0);
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_border_color(btn, colorBtn, 0);
        lv_obj_set_style_bg_color(btn, colorBtn, LV_STATE_PRESSED);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, txt);
        lv_obj_center(lbl);
        lv_obj_set_style_text_font(lbl, btnFont, 0);

        lv_obj_add_event_cb(
          btn, [](lv_event_t *e) {
            const char *k = (const char *)lv_event_get_user_data(e);
            lv_async_call([](void *p) {
              const char *key = (const char *)p;
              buttonAction(key);
            },
                          (void *)k);
          },
          LV_EVENT_CLICKED, (void *)key);
      }

      // ---------- NUMERIC ----------
      else if (strcmp(type, "num") == 0) {

        // outer numeric container
        lv_obj_t *numBox = lv_obj_create(cont);
        lv_obj_set_size(numBox, colW, 48);
        lv_obj_set_pos(numBox, x, y);
        lv_obj_set_flex_flow(numBox, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(numBox,
                              LV_FLEX_ALIGN_SPACE_BETWEEN,
                              LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);

        // neutral look (no box highlight)
        lv_obj_set_style_bg_opa(numBox, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(numBox, 2, 0);
        lv_obj_set_style_border_color(numBox, COLOR_BTN_NUM, 0);
        lv_obj_set_style_radius(numBox, CORNERS, 0);
        lv_obj_set_style_pad_all(numBox, 4, 0);
        lv_obj_set_style_pad_gap(numBox, 6, 0);
        lv_obj_clear_flag(numBox, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(numBox, LV_OBJ_FLAG_SCROLL_CHAIN);
        lv_obj_add_flag(numBox, LV_OBJ_FLAG_CLICKABLE);

        // --- MINUS button ---
        lv_obj_t *btnMinus = lv_btn_create(numBox);
        lv_obj_set_size(btnMinus, 40, 40);
        lv_obj_set_style_radius(btnMinus, 20, 0);
        lv_obj_set_style_bg_opa(btnMinus, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(btnMinus, 0, 0);
        lv_obj_add_flag(btnMinus, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(btnMinus, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_clear_flag(btnMinus, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(btnMinus, LV_OBJ_FLAG_SCROLL_CHAIN);
        // press feedback (outline thicker + bright)
        lv_obj_add_style(btnMinus, &style_num_btn_pressed, LV_STATE_PRESSED);

        lv_obj_t *lblMinus = lv_label_create(btnMinus);
        lv_label_set_text(lblMinus, LV_SYMBOL_MINUS);
        lv_obj_center(lblMinus);

        // --- VALUE label ---
        lv_obj_t *lblVal = lv_label_create(numBox);
        char buf[8];
        snprintf(buf, sizeof(buf), "%04d", getParamValue(key));
        lv_label_set_text(lblVal, buf);
        lv_obj_set_style_text_font(lblVal, FONT_BTN_SMALL_PTR, 0);
        lv_obj_set_style_text_color(lblVal, COLOR_BTN_TEXT, 0);
        lv_obj_set_style_text_align(lblVal, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_flag(lblVal, LV_OBJ_FLAG_CLICKABLE);  // clickable to toggle selection
        lv_obj_add_flag(lblVal, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_clear_flag(lblVal, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(lblVal, LV_OBJ_FLAG_SCROLL_CHAIN);

        // --- PLUS button ---
        lv_obj_t *btnPlus = lv_btn_create(numBox);
        lv_obj_set_size(btnPlus, 40, 40);
        lv_obj_set_style_radius(btnPlus, 20, 0);
        lv_obj_set_style_bg_opa(btnPlus, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(btnPlus, 0, 0);
        lv_obj_add_flag(btnPlus, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(btnPlus, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_clear_flag(btnPlus, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(btnPlus, LV_OBJ_FLAG_SCROLL_CHAIN);
        // press feedback
        lv_obj_add_style(btnPlus, &style_num_btn_pressed, LV_STATE_PRESSED);

        lv_obj_t *lblPlus = lv_label_create(btnPlus);
        lv_label_set_text(lblPlus, LV_SYMBOL_PLUS);
        lv_obj_center(lblPlus);

        // --- selection by tapping the number (ONLY fills +/−, no box outline) ---
        lv_obj_add_event_cb(
          lblVal, [](lv_event_t *e) {
            if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
              lv_obj_t *val = (lv_obj_t *)lv_event_get_target(e);
              lv_obj_t *numBoxP = lv_obj_get_parent(val);
              select_num_pair(numBoxP, /*toggle=*/true);
            }
          },
          LV_EVENT_CLICKED, nullptr);

        // --- value update helper ---
        auto update_value = [&](lv_event_t *e, int delta) {
          lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);
          lv_obj_t *numBoxP = lv_obj_get_parent(btn);
          if (!numBoxP) return;

          // ensure this pair is selected (fills +/−)
          select_num_pair(numBoxP, /*toggle=*/false);

          const char *keyUD = (const char *)lv_event_get_user_data(e);
          int val = getParamValue(keyUD);
          val += delta;
          if (val < 0) val = 0;
          else if (val > 9999) val = 9999;
          setParamValue(keyUD, val);

          lv_obj_t *lbl = lv_obj_get_child(numBoxP, 1);  // middle label
          char b[8];
          snprintf(b, sizeof(b), "%04d", val);
          lv_label_set_text(lbl, b);
        };

        // --- minus click: increments + press feedback already handled by style ---
        lv_obj_add_event_cb(
          btnMinus, [](lv_event_t *e) {
            auto fn = (decltype(update_value) *)lv_event_get_user_data(e);
            fn->operator()(e, -1);
          },
          LV_EVENT_CLICKED, (void *)&update_value);

        // --- plus click ---
        lv_obj_add_event_cb(
          btnPlus, [](lv_event_t *e) {
            auto fn = (decltype(update_value) *)lv_event_get_user_data(e);
            fn->operator()(e, +1);
          },
          LV_EVENT_CLICKED, (void *)&update_value);
      }

      // ---------- STATUS BUTTON ----------
      else if (strcmp(type, "status_button") == 0) {
        const char *key = it["key"] | "";
        const char *status = getStatusText(key);
        const char *colStr = getStatusColor(key);
        if (strcmp(status, "na") == 0) {
          status = it["status"] | "";
          colStr = it["color"] | "gray";
        }
        lv_color_t sColor = colorFromName(colStr);

        // make a clean, visible button
        lv_obj_t *btn = lv_btn_create(cont);
        lv_obj_remove_style_all(btn);  // remove theme junk
        lv_obj_set_size(btn, colW, 48);
        lv_obj_set_pos(btn, x, y);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);

        // base outline only, transparent fill
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_border_color(btn, COLOR_BTN_STATUS, 0);
        lv_obj_set_style_radius(btn, CORNERS, 0);

        // pressed feedback: thicker outline (no fill)
        static lv_style_t style_status_pressed;
        static bool style_inited = false;
        if (!style_inited) {
          lv_style_init(&style_status_pressed);
          lv_style_set_border_width(&style_status_pressed, 3);
          lv_style_set_border_color(&style_status_pressed, COLOR_BTN_MENU);  // light greenish highlight
          lv_style_set_bg_opa(&style_status_pressed, LV_OPA_TRANSP);
          style_inited = true;
        }
        lv_obj_add_style(btn, &style_status_pressed, LV_STATE_PRESSED);

        // label (centered text)
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, status);
        lv_obj_set_style_text_color(lbl, sColor, 0);
        lv_obj_set_style_text_font(lbl, btnFont, 0);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_opa(lbl, LV_OPA_COVER, 0);
        lv_obj_center(lbl);

        // store label for dynamic updates
        statusWidgets[key] = lbl;

        // click handler
        lv_obj_add_event_cb(
          btn, [](lv_event_t *e) {
            const char *k = (const char *)lv_event_get_user_data(e);
            lv_async_call([](void *p) {
              const char *key = (const char *)p;
              buttonAction(key);
            },
                          (void *)k);
          },
          LV_EVENT_CLICKED, (void *)key);
      }

      // ---------- STATUS TEXT ----------
      else if (strcmp(type, "status_text") == 0) {
        const char *key = it["key"] | "";
        const char *status = getStatusText(key);
        const char *colStr = getStatusColor(key);

        if (strcmp(status, "na") == 0) {
          status = it["status"] | "";
          colStr = it["color"] | "gray";
        }

        lv_color_t sColor = colorFromName(colStr);

        // plain, fully neutral cell
        lv_obj_t *cell = lv_obj_create(cont);
        lv_obj_remove_style_all(cell);  // remove default background and border
        lv_obj_set_size(cell, colW, 48);
        lv_obj_set_pos(cell, x, y);
        lv_obj_set_style_bg_opa(cell, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(cell, 0, 0);
        lv_obj_set_style_shadow_opa(cell, LV_OPA_TRANSP, 0);  // ensure no shadow from theme
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);

        // centered label only
        lv_obj_t *lbl = lv_label_create(cell);
        lv_label_set_text(lbl, status);
        lv_obj_set_style_text_color(lbl, sColor, 0);
        lv_obj_set_style_text_font(lbl, btnFont, 0);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_opa(lbl, LV_OPA_COVER, 0);  // fully opaque text
        lv_obj_set_style_bg_opa(lbl, LV_OPA_TRANSP, 0);   // label itself transparent
        lv_obj_set_style_border_width(lbl, 0, 0);
        lv_obj_center(lbl);

        statusWidgets[key] = lbl;
      }

      x += colW + 8;
    }
    y += adjustedRowH;
  }

  footLbl = lv_label_create(scr);
  lv_label_set_text(footLbl, footer);
  lv_obj_set_style_text_font(footLbl, FONT_FOOT_PTR, 0);
  lv_obj_align(footLbl, LV_ALIGN_BOTTOM_MID, 0, -10);

  LOG_SECTION_END();
}

// ----------------------------------------------------------
//                     Example JSON with all types
// ----------------------------------------------------------
static char jsonBuffer[] = R"json(
{
  "main": {
    "title": "rubik cube solver",
    "footer": "ready",
    "columns": 1,
    "rows": [
      [{ "text": "solve cube", "type": "menu", "key": "solve" }],
      [{ "text": "read cube", "type": "menu", "key": "read" }],
      [{ "text": "scramble cube", "type": "menu", "key": "random" }],
      [{ "text": "tests", "type": "menu", "key": "tests" }]
    ]
  },
  "solve": {
    "title": "solve cube",
    "footer": "press to start solving",
    "columns": 1,
    "rows": [
      [{ "text": "start", "type": "action", "key": "run_solve" }],
      [{ "text": "back", "type": "menu", "key": "main" }]
    ]
  },
  "read": {
    "title": "read cube",
    "footer": "read cube colors",
    "columns": 1,
    "rows": [
      [{ "text": "start read", "type": "action", "key": "run_read" }],
      [{ "text": "back", "type": "menu", "key": "main" }]
    ]
  },
  "random": {
    "title": "scramble cube",
    "footer": "scramble cube randomly",
    "columns": 1,
    "rows": [
      [{ "text": "scramble (12)", "type": "action", "key": "scramble_12" }],
      [{ "text": "scramble (20)", "type": "action", "key": "scramble_20" }],
      [{ "text": "back", "type": "menu", "key": "main" }]
    ]
  },
  "tests": {
    "title": "tests",
    "footer": "perform hardware tests",
    "columns": 2,
    "equal columns": "all",
    "rows": [
      [
        { "text": "servo ids", "type": "menu", "key": "servos_programming" },
        { "text": "servo limits", "type": "menu", "key": "servo_limits" }
      ],
      [
        { "text": "vertical tune", "type": "menu", "key": "vertical_tune" },
        { "text": "", "type": "text", "key": "" }
      ],
      [
        { "text": "poses", "type": "menu", "key": "poses" },
        { "text": "pose groups", "type": "menu", "key": "pose_groups" }
      ],
      [
        { "text": "sequences", "type": "menu", "key": "sequences" },
        { "text": "cube moves", "type": "menu", "key": "cube_moves" }
      ],
      [
        { "text": "back", "type": "menu", "key": "main" },
        { "text": "", "type": "text", "key": "" }
      ]
    ]
  },
  "servo_limits": {
    "title": "servo limits",
    "footer": "touch to edit numeric limits",
    "columns": 4,
    "equal columns": "last",
    "rows": [
      [
        { "text": "servo", "type": "text" },
        { "text": "0 degree", "type": "text" },
        { "text": "min limit", "type": "text" },
        { "text": "max limit", "type": "text" }
      ],
      [
        { "text": "arm1", "type": "text" },
        { "text": " -0000+ ", "type": "num", "key": "arm1_0" },
        { "text": " -0000+ ", "type": "num", "key": "arm1_min" },
        { "text": " -0000 +", "type": "num", "key": "arm1_max" }
      ],
      [
        { "text": "arm2", "type": "text" },
        { "text": "-0000+", "type": "num", "key": "arm2_0" },
        { "text": "-0000+", "type": "num", "key": "arm2_min" },
        { "text": "-0000+", "type": "num", "key": "arm2_max" }
      ],
      [
        { "text": "wrist", "type": "text" },
        { "text": "-0000+", "type": "num", "key": "wrist_0" },
        { "text": "-0000+", "type": "num", "key": "wrist_min" },
        { "text": "-0000+", "type": "num", "key": "wrist_max" }
      ],
      [
        { "text": "grip l", "type": "text" },
        { "text": "-0000+", "type": "num", "key": "grip1_0" },
        { "text": "-0000+", "type": "num", "key": "grip1_min" },
        { "text": "-0000+", "type": "num", "key": "grip1_max" }
      ],
      [
        { "text": "grip r", "type": "text" },
        { "text": "-0000+", "type": "num", "key": "grip2_0" },
        { "text": "-0000+", "type": "num", "key": "grip2_min" },
        { "text": "-0000+", "type": "num", "key": "grip2_max" }
      ],
      [
        { "text": "base", "type": "text" },
        { "text": "-0000+", "type": "num", "key": "base_0" },
        { "text": "-0000+", "type": "num", "key": "base_min" },
        { "text": "-0000+", "type": "num", "key": "base_max" }
      ],
      [
        { "text": "back", "type": "menu", "key": "tests" },
        { "text": "", "type": "text", "key": "" },
        { "text": "", "type": "text", "key": "" },
        { "text": "", "type": "text", "key": "" }
      ]
    ]
  },
  "servos_programming": {
    "title": "tests",
    "footer": "set servo id and reset to mid range",
    "columns": 4,
    "equal columns":"last",
    "rows": [
      [
        { "text": "arm1 (id11)", "type": "text", "key": "" },
        { "text": "program", "type": "status_button", "key": "arm1_program", "status": "program" },
        { "text": "test", "type": "status_button", "key": "arm1_test", "status": "test" },
        { "text": "status", "type": "status_text", "key": "arm1_status", "status": "not set" }
      ],
      [
        { "text": "arm2 (id12)", "type": "text", "key": "" },
        { "text": "program", "type": "status_button", "key": "arm2_program", "status": "program" },
        { "text": "test", "type": "status_button", "key": "arm2_test", "status": "test" },
        { "text": "status", "type": "status_text", "key": "arm2_status", "status": "not set" }
      ],
      [
        { "text": "wrist (id13)", "type": "text", "key": "" },
        { "text": "program", "type": "status_button", "key": "wrist_program", "status": "program" },
        { "text": "test", "type": "status_button", "key": "wrist_test", "status": "test" },
        { "text": "status", "type": "status_text", "key": "wrist_status", "status": "not set" }
      ],
      [
        { "text": "grip l (id14)", "type": "text", "key": "" },
        { "text": "program", "type": "status_button", "key": "grip1_program", "status": "program" },
        { "text": "test", "type": "status_button", "key": "grip1_test", "status": "test" },
        { "text": "status", "type": "status_text", "key": "grip1_status", "status": "not set" }
      ],
      [
        { "text": "grip r (id15)", "type": "text", "key": "" },
        { "text": "program", "type": "status_button", "key": "grip2_program", "status": "program" },
        { "text": "test", "type": "status_button", "key": "grip2_test", "status": "test" },
        { "text": "status", "type": "status_text", "key": "grip2_status", "status": "not set" }
      ],
      [
        { "text": "base (id16)", "type": "text", "key": "" },
        { "text": "program", "type": "status_button", "key": "base_program", "status": "program" },
        { "text": "test", "type": "status_button", "key": "base_test", "status": "test" },
        { "text": "status", "type": "status_text", "key": "base_status", "status": "not set" }
      ],
      [
        { "text": "back", "type": "menu", "key": "tests" },
        { "text": "", "type": "text" },
        { "text": "", "type": "text" },
        { "text": "", "type": "text" }
      ]
    ]
  },
  "vertical_tune": {
    "title": "tests",
    "footer": "adjust for vertical move arm1, arm2, wrist",
    "columns": 4,
    "equal columns":"last",
    "rows": [
      [
        { "text": "mm", "type": "text", "key": "grip_open" },
        { "text": "arm1", "type": "text" },
        { "text": "arm2", "type": "text", "key": "grip_close" },
        { "text": "wrist", "type": "text" }
      ],
      [
        { "text": "0 mm", "type": "action", "key": "vertical_0mm" },
        { "text": "arm1", "type": "num", "key": "arm1_0mm" },
        { "text": "arm2", "type": "num", "key": "arm2_0mm" },
        { "text": "wrist", "type": "num", "key": "wrist_0mm" }
      ],
      [
        { "text": "5 mm", "type": "action", "key": "vertical_5mm" },
        { "text": "arm1", "type": "num", "key": "arm1_5mm" },
        { "text": "arm2", "type": "num", "key": "arm2_5mm" },
        { "text": "wrist", "type": "num", "key": "wrist_5mm" }
      ],
      [
        { "text": "10 mm", "type": "action", "key": "vertical_10mm" },
        { "text": "arm1", "type": "num", "key": "arm1_10mm" },
        { "text": "arm2", "type": "num", "key": "arm2_10mm" },
        { "text": "wrist", "type": "num", "key": "wrist_10mm" }
      ],
      [
        { "text": "15 mm", "type": "action", "key": "vertical_15mm" },
        { "text": "arm1", "type": "num", "key": "arm1_15mm" },
        { "text": "arm2", "type": "num", "key": "arm2_15mm" },
        { "text": "wrist", "type": "num", "key": "wrist_15mm" }
      ],
      [
        { "text": "", "type": "text" },
        { "text": "g open", "type": "action", "key": "grip_open" },
        { "text": "g close", "type": "action", "key": "grip_close" },
        { "text": "", "type": "text" }
      ],
      [
        { "text": "back", "type": "menu", "key": "tests" },
        { "text": "", "type": "text" },
        { "text": "", "type": "text" },
        { "text": "", "type": "text" }
      ]
    ]
  },
  "poses": {
    "title": "servo poses", "footer": "tap to edit pose values", 
    "columns": 3,
    "equal columns":"all",
    "rows": [
       [{ "text": "arm1 0", "type": "action", "key": "arm1_0" },
        { "text": "+0000-", "type": "num", "key": "arm1_pose_0" },
        { "text": "current na", "type": "status_text", "status":"current na", "key": "arm1_current" }],

       [{ "text": "arm2 0", "type": "action", "key": "arm2_0" },
        { "text": "+0000-", "type": "num", "key": "arm2_pose_0" },
        { "text": "current na", "type": "status_text", "status":"current na", "key": "arm2_current" }],

       [{ "text": "wrist 0", "type": "action", "key": "wrist_0" },
        { "text": "+0000-", "type": "num", "key": "wrist_pose_0" },
        { "text": "current na", "type": "status_text", "status":"current na", "key": "wrist_current" }],

       [{ "text": "wrist 90", "type": "action", "key": "wrist_90" },
        { "text": "+0000-", "type": "num", "key": "wrist_pose_90" },
        { "text": "current na", "type": "status_text", "status":"current na", "key": "wrist_current" }],

      [{ "text": "wrist -90", "type": "action", "key": "wrist_minus90" },
        { "text": "+0000-", "type": "num", "key": "wrist_pose_minus90" },
        { "text": "current na", "type": "status_text", "status":"current na", "key": "wrist_current" }],

       [{ "text": "grip1 Open", "type": "action", "key": "grip1_open" },
        { "text": "+0000-", "type": "num", "key": "grip1_pose_0" },
        { "text": "current na", "type": "status_text", "status":"current na", "key": "grip1_current" }],

       [{ "text": "grip1 Close", "type": "action", "key": "grip1_close" },
        { "text": "+0000-", "type": "num", "key": "grip1_pose_1" },
        { "text": "current na", "type": "status_text", "status":"current na", "key": "grip1_current" }],

       [{ "text": "grip2 Open", "type": "action", "key": "grip2_open" },
        { "text": "+0000-", "type": "num", "key": "grip2_pose_0" },
        { "text": "current na", "type": "status_text", "status":"current na", "key": "grip2_current" }],

       [{ "text": "grip2 Close", "type": "action", "key": "grip2_close" },
        { "text": "+0000-", "type": "num", "key": "grip2_pose_1" },
        { "text": "current na", "type": "status_text", "status":"current na", "key": "grip2_current" }],

       [{ "text": "base 0", "type": "action", "key": "base_0" },
        { "text": "+0000-", "type": "num", "key": "base_pose_0" },
        { "text": "current na", "type": "status_text", "status":"current na", "key": "base_current" }],

       [{ "text": "base 90", "type": "action", "key": "base_90" },
        { "text": "+0000-", "type": "num", "key": "base_pose_90" },
        { "text": "current na", "type": "status_text", "status":"current na", "key": "base_current" }],

       [{ "text": "base -90", "type": "action", "key": "base_minus90" },
        { "text": "+0000-", "type": "num", "key": "base_pose_minus90" },
        { "text": "current na", "type": "status_text", "status":"current na", "key": "base_current" }],

       [{ "text": "back", "type": "menu", "key": "tests" },
        { "text": "", "type": "text" },
        { "text": "", "type": "text" }]
    ]
  },
  "pose_groups": {
    "title": "pose groups", "footer": "tap to run pose groups", 
    "columns": 2,
    "equal columns":"all",
    "rows": [
      [{ "text": "arms 0", "type": "action", "key": "arms_0" }, 
       { "text": "arms mid", "type": "action", "key": "arms_row1" }], 

      [{ "text": "arms high", "type": "action", "key": "arms_row1" },
      { "text": "wrist 0", "type": "action", "key": "wrist_v0" }], 

      [{ "text": "wrist 90", "type": "action", "key": "wrist_v90" }, 
       { "text": "wrist -90", "type": "action", "key": "wrist_vminus90" }],

      [{ "text": "back", "type": "menu", "key": "tests" }, 
       { "text": "", "type": "text" }] 
    ]
  },
  "sequences": {
    "title": "sequences",  "footer": "tap to execute motion sequence", 
    "columns": 2,
    "equal columns":"all",
    "rows": [
      [{ "text": "bottom+", "type": "action", "key": "bottom_plus" }, 
       { "text": "bottom-", "type": "action", "key": "bottom_minus" }],

      [{ "text": "front to base", "type": "action", "key": "front_to_base" }, 
       { "text": "back to base", "type": "action", "key": "back_to_base" }],

      [{ "text": "left to base", "type": "action", "key": "left_to_base" }, 
       { "text": "right to base", "type": "action", "key": "right_to_base" }],

      [{ "text": "top to base", "type": "action", "key": "top_to_base" }, 
       { "text": "", "type": "text", "key": "" }],

      [{ "text": "rotate down face+", "type": "action", "key": "rotate_down_90" }, 
       { "text": "rotate down face-", "type": "action", "key": "rotate_down_minus90" }],

      [{ "text": "back", "type": "menu", "key": "tests" }, 
       { "text": "", "type": "text" }]
    ]
  },
  "cube_moves": {
    "title": "cube moves", "footer": "tap to run a move", 
    "columns": 4,
    "equal columns":"all",
    "rows": [
      [{ "text": "F+", "type": "action", "key": "f_plus" }, 
       { "text": "F-", "type": "action", "key": "f_minus" }, 
       { "text": "B+", "type": "action", "key": "b_plus" }, 
       { "text": "B-", "type": "action", "key": "b_minus" }],

      [{ "text": "U+", "type": "action", "key": "u_plus" }, 
       { "text": "U-", "type": "action", "key": "u_minus" }, 
       { "text": "D+", "type": "action", "key": "d_plus" }, 
       { "text": "D-", "type": "action", "key": "d_minus" }],

      [{ "text": "L+", "type": "action", "key": "l_plus" }, 
       { "text": "L-", "type": "action", "key": "l_minus" }, 
       { "text": "R+", "type": "action", "key": "r_plus" }, 
       { "text": "R-", "type": "action", "key": "r_minus" }],

      [{ "text": "F++", "type": "action", "key": "f_plusplus" }, 
       { "text": "B++", "type": "action", "key": "b_plusplus" }, 
       { "text": "U++", "type": "action", "key": "u_plusplus" }, 
       { "text": "D++", "type": "action", "key": "d_plusplus" }],

      [{ "text": "L++", "type": "action", "key": "l_plusplus" }, 
       { "text": "R++", "type": "action", "key": "r_plusplus" }, 
       { "text": "back", "type": "menu", "key": "tests" }, 
       { "text": "", "type": "text" }]
    ]
  }
}
)json";

// ----------------------------------------------------------
//                     LVGL INITIALIZATION
// ----------------------------------------------------------

void ui_init() {
  LOG_SECTION_START("ui_init");

  // 1. Initialize LVGL core
  lv_init();
  //lv_disp_t *disp = lv_disp_get_default();
  //lv_disp_set_antialiasing(disp, false);

  // 2. Initialize the display hardware
  static Arduino_H7_Video Display(800, 480, GigaDisplayShield);
  static Arduino_GigaDisplayTouch TouchDetector;

  Display.begin();  // sets up framebuffer and LVGL display driver
  TouchDetector.begin();

  // 3. Parse JSON AFTER display init
  DeserializationError err = deserializeJson(menuDoc, jsonBuffer);
  if (err) {
    LOG_VAR("error on deserialize json", "err");
    LOG_SECTION_END();
    return;  // avoid using empty doc
  }

  buildMenu(currentMenu.c_str());

  LOG_SECTION_END();
}

// ----------------------------------------------------------
//              PERIODIC REFRESH FUNCTION
// ----------------------------------------------------------

// Track last refresh time
static unsigned long lastRefresh = 0;

void ui_refresh() {
  unsigned long now = millis();
  if (now - lastRefresh < UI_REFRESH_INTERVAL_SEC * 1000UL) return;
  lastRefresh = now;

  // Example: update dynamic fields
  static int dummyStall = 250;
  static int dummyTemp = 30;

  dummyStall += (random(-2, 3));  // simulate current fluctuation
  dummyTemp += (random(-1, 2));   // simulate temp fluctuation

  // Keep values in range
  if (dummyStall < 230) dummyStall = 230;
  if (dummyStall > 270) dummyStall = 270;
  if (dummyTemp < 28) dummyTemp = 28;
  if (dummyTemp > 35) dummyTemp = 35;

  // Update displayed values
  char buf[32];
  snprintf(buf, sizeof(buf), "%d mA", dummyStall);
  updateStatus("stall", buf, "orange");

  snprintf(buf, sizeof(buf), "%d C", dummyTemp);
  updateStatus("temp", buf, "green");

  // Optional footer note to see refresh activity
  // setFooter("refreshed status data");
}

void ui_loop() {
  lv_timer_handler();
  ui_refresh();  // <-- call refresh periodically
}

void setup() {
  LOG_SECTION_START("setup");

  Serial.begin(115200);
  while (!Serial && millis() < 9999) delay(111);
  delay(333);

  ui_init();

  // selected state: fill +/− buttons
  lv_style_init(&style_num_btn_active);
  lv_style_set_bg_opa(&style_num_btn_active, LV_OPA_TRANSP);        // transparent background
  lv_style_set_border_color(&style_num_btn_active, COLOR_BTN_NUM);  // orange outline
  lv_style_set_border_width(&style_num_btn_active, 2);
  lv_style_set_radius(&style_num_btn_active, 20);

  // pressed feedback: thicker orange outline, no fill
  lv_style_init(&style_num_btn_pressed);
  lv_style_set_bg_opa(&style_num_btn_pressed, LV_OPA_TRANSP);        // no fill
  lv_style_set_border_color(&style_num_btn_pressed, COLOR_BTN_NUM);  // same orange
  lv_style_set_border_width(&style_num_btn_pressed, 3);              // thicker border on press
  lv_style_set_radius(&style_num_btn_pressed, 20);

  dynamixel_begin();

  LOG_SECTION_END();
}

void loop() {
  LOG_RESET();
  ui_loop();
  delay(5);
}

/**
 * ================================================================
 *  Rubik Solver — System Architecture Overview
 *  Target: Arduino GIGA R1 WiFi + GIGA Display Shield (ASX00039)
 * ================================================================
 *
 *  Hardware:
 *    - 800x480 LVGL touchscreen (Arduino_H7_Video + GigaDisplayTouch)
 *    - Dynamixel XL430 servos via Serial1 + DIR pin
 *    - Adafruit TCS34725 color sensor on I2C (SDA=20, SCL=21)
 *    - No rotary encoder; touch-only interface
 *
 * ---------------------------------------------------------------
 *  Purpose:
 *    A self-contained Rubik’s Cube solving robot with a full
 *    touchscreen UI.  Uses LVGL for menus, Dynamixel2Arduino
 *    for motion, and Adafruit TCS34725 for color detection.
 *
 * ---------------------------------------------------------------
 *  Core Architecture
 *
 *  1. Hardware Control Layer
 *     - servos.cpp/.h:
 *         Controls 6 Dynamixel servos (arm1, arm2, wrist,
 *         grip1, grip2, base).  Provides tick <-> angle
 *         helpers and goal position movement.
 *     - color_sensor.cpp/.h:
 *         Reads RGB data from TCS34725, converts to HSV,
 *         classifies as W/R/G/Y/O/B cube colors.
 *     - Timing:
 *         Uses Arduino millis()/delay() for sequencing.
 *
 *  2. Parameter Management Layer
 *     - param_store.cpp/.h:
 *         Holds all calibration data: servo limits, poses,
 *         derived values (e.g. base ±30°, ±60°).
 *         Provides get/set functions and automatic updates.
 *
 *  3. Action & Sequence Layer
 *     - action_store.cpp/.h:
 *         Maps text commands ("arms_row1", "wrist_90", etc.)
 *         to servo poses or sequences (POSE/GROUP/SEQUENCE).
 *     - group_motion.cpp/.h:
 *         Moves multiple servos together with cosine easing.
 *     - orientation_map.cpp:
 *         Tracks cube orientation (F/R/U etc.) and remaps
 *         logical moves to current physical axes.
 *
 *  4. Solver Logic Layer
 *     - read_cube.cpp:
 *         Uses color sensor to read cube faces, updates model.
 *     - solve.cpp:
 *         Executes move list, shows progress bar.
 *     - scramble.cpp:
 *         Generates random scramble sequences.
 *     - cube_colors.cpp/.h:
 *         Holds 6 faces (3x3 each), draws cube net in LVGL.
 *
 *  5. User Interface Layer (Touch + LVGL)
 *     - ui_touch.cpp/.h:
 *         Touch menus built with LVGL 8.2.
 *         Buttons respond instantly to taps.
 *         Numeric fields enter edit mode showing
 *         small +/- buttons for value change.
 *         Long menus show ▲/▼ arrows for scrolling.
 *     - Theme:
 *         Dark background (0x101215), bright text (0xF5F8FF),
 *         buttons with LV_SYMBOL icons.
 *
 *  6. Main Control Layer
 *     - rubik_solver_giga_touch.ino:
 *         Initializes Display, LVGL, touch, servos, sensor.
 *         Loads main menu and runs LVGL event loop.
 *
 * ---------------------------------------------------------------
 *  System Flow:
 *     1. Setup
 *        - LVGL, touch, and hardware initialize.
 *        - Params and actions load.
 *        - Servos centered, UI shown.
 *
 *     2. User Interaction
 *        - Tap a main menu button:
 *          "Solve Cube", "Read Colors",
 *          "Random Cube", or "Tests".
 *
 *     3. Execution
 *        - runGroupPoseServos() drives coordinated motion.
 *        - Progress and cube visuals update live.
 *
 *     4. Completion
 *        - Returns to idle screen; user can start again.
 *
 * ---------------------------------------------------------------
 *  UI Design Principles:
 *    - Touch-first, no rotary encoder.
 *    - Minimal navigation depth (2 levels max).
 *    - Immediate feedback for each tap.
 *    - High-contrast dark theme for readability.
 *
 * ---------------------------------------------------------------
 *  Hardware I/O Summary:
 *    - Dynamixel bus: Serial1 (TX1=PA9, RX1=PA10) + D2 (DIR)
 *    - Color sensor: SDA=20, SCL=21 (I2C)
 *    - Display Shield: internal DPI + I2C (no pin conflict)
 *    - Power: 5V logic, separate 12V for servos, shared GND
 *
 * ---------------------------------------------------------------
 *  Development Environment:
 *    - Arduino IDE 2.x
 *    - Board package: Arduino mbed GIGA (>= 4.1.x)
 *    - Libraries:
 *        Arduino_H7_Video
 *        Arduino_GigaDisplayTouch
 *        Arduino_GigaDisplay_GFX  (LVGL 8.2)
 *        Dynamixel2Arduino
 *        Adafruit_TCS34725
 *        ArduinoJson (optional)
 *
 * ---------------------------------------------------------------
 *  Future Extensions:
 *    - Save/load parameters to Flash or SD.
 *    - Implement on-board solver algorithm (Kociemba).
 *    - Real-time 3D cube visualization in LVGL.
 *    - Wi-Fi or USB telemetry.
 *
 * ---------------------------------------------------------------
 *  Summary:
 *    - Clean separation of UI, control logic, and hardware.
 *    - Runs entirely on Arduino GIGA R1 + Display Shield.
 *    - Provides a responsive, touch-based Rubik Solver system.
 *
 * ================================================================
 */
