// ---------------- See a desciption of the project at the end of this file  ----------------
#include <Arduino.h>
#include "Arduino_H7_Video.h"
#include <lvgl.h>
#include <Arduino_GigaDisplayTouch.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include "param_store.h"
#include "servos.h"
#include "logging.h"

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

// Base colors
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
static const lv_color_t COLOR_BTN_BACK = lv_color_hex(0xC04040);  // reddish tone for Back

static const int HORIZONTAL_SPACING = 20;
static const int VERTICAL_SPACING = 70;
static const int CORNERS = 20; // was 40

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
  // __LOG_VAR("footer", msg);
}

// Increment storage (simple)
int incrementValue(const char *key, int delta) {
  static std::map<String, int> values;
  values[key] += delta;
  if (values[key] < 0) values[key] = 0;
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
  if (menuDoc.containsKey(key)) {  // navigate to submenu
    currentMenu = key;
    setFooter((String("Switch menu -> ") + key).c_str());
    buildMenu(currentMenu.c_str());
  } else if (strcmp(key, "main") == 0) {  // explicit back to main
    currentMenu = "main";
    setFooter("Back to main menu");
    buildMenu(currentMenu.c_str());
  } else {
    setFooter((String("Action: ") + key).c_str());
  }
  __LOG_VAR("button action", key);
}

// ----------------------------------------------------------
//                     MENU BUILDER
// ----------------------------------------------------------
void buildMenu(const char *menuName) {

  __LOG_VAR("build menu start", menuName);

  lv_obj_t *scr = lv_scr_act();
  if (scr == nullptr) {
    __LOG_VAR("lvgl screen not ready yet", "null");
    return;
  }
  if (!menuDoc.containsKey(menuName)) {
    __LOG_VAR("menu not found", menuName);
    return;
  }

  lv_obj_clean(lv_scr_act());
  statusWidgets.clear();
  numLabels.clear();
  JsonObject root = menuDoc[menuName];

  if (root.isNull()) {
    __LOG_VAR("root", "null");
    return;
  }

  const char *title = root["title"] | "";
  const char *subtitle = root["subtitle"] | "";
  const char *footer = root["footer"] | "";

  int columns = root["columns"] | 1;
  JsonArray rows = root["rows"].as<JsonArray>();

  lv_obj_set_style_bg_color(scr, COLOR_BG, 0);
  lv_obj_set_style_text_color(scr, COLOR_TEXT, 0);

  // Title
  lv_obj_t *titleLbl = lv_label_create(scr);
  __LOG_VAR("   title", title);
  lv_label_set_text(titleLbl, title);
  lv_obj_set_style_text_font(titleLbl, FONT_TITLE_PTR, 0);
  lv_obj_align(titleLbl, LV_ALIGN_TOP_MID, 0, 6);

  // Subtitle
  lv_obj_t *subLbl = lv_label_create(scr);
  __LOG_VAR("   sub title", subtitle);
  lv_label_set_text(subLbl, subtitle);
  lv_obj_set_style_text_font(subLbl, FONT_SUBTITLE_PTR, 0);
  lv_obj_update_layout(titleLbl);  // ensure title height is computed
  int title_h = lv_obj_get_height(titleLbl);

  lv_obj_align(subLbl, LV_ALIGN_TOP_MID, 0, 6 + title_h + 10);
  // 6 = top margin of title, 10 = extra spacing between title & subtitle

  const lv_font_t *btnFont = (columns <= 4) ? FONT_BTN_LARGE_PTR
                                            : FONT_BTN_SMALL_PTR;
  // Scrollable area for tall menus
  lv_obj_t *cont = lv_obj_create(scr);

  // --- create the container first with provisional size ---
  lv_obj_set_size(cont, SCREEN_W, SCREEN_H / 2);  // temporary
  lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, 60);
  lv_obj_set_scroll_dir(cont, LV_DIR_VER);
  lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(cont, 0, 0);

  // --- defer final sizing until LVGL has computed layouts ---
  lv_async_call([](void *param) {
    lv_obj_t *scr = (lv_obj_t *)param;
    lv_obj_t *subLbl = lv_obj_get_child(scr, 1);    // your subtitle
    lv_obj_t *footLbl = lv_obj_get_child(scr, -1);  // your footer (last child)
    lv_obj_t *cont = lv_obj_get_child(scr, 2);      // the container

    lv_obj_update_layout(subLbl);
    lv_obj_update_layout(footLbl);

    int sub_bottom = lv_obj_get_y(subLbl) + lv_obj_get_height(subLbl);
    int footer_top = lv_obj_get_y(footLbl);
    int available_h = footer_top - (sub_bottom + 20);
    if (available_h < 60) available_h = 60;

    lv_obj_set_size(cont, SCREEN_W, available_h);
    lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, sub_bottom + 10);
    lv_refr_now(NULL);  // single clean refresh
  },
                scr);


  // ---------- DYNAMIC COLUMN WIDTH COMPUTATION ----------
  std::vector<int> colWidths(columns, 0);
  const int textPad = 20;  // margin per column
  const int numPad = 70;   // extra room for "+0000-" etc.

  // Temporary label object for measuring text
  lv_obj_t *measLbl = lv_label_create(lv_scr_act());
  lv_obj_add_flag(measLbl, LV_OBJ_FLAG_HIDDEN);

  // Iterate all rows to find widest text per column
  for (JsonArray row : rows) {
    for (int ci = 0; ci < (int)row.size() && ci < columns; ci++) {
      JsonObject it = row[ci];
      const char *txt = it["text"] | "";
      const char *type = it["type"] | "text";

      lv_point_t sz;
      lv_txt_get_size(&sz, txt, btnFont, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);

      int width = sz.x + textPad;
      if (strcmp(type, "num") == 0)
        width += numPad;

      if (width > colWidths[ci]) colWidths[ci] = width;
    }
  }

  lv_obj_del(measLbl);  // cleanup

  // Compute total width
  int totalW = 0;
  for (int w : colWidths) totalW += w + 10;            // include spacing
  if (totalW > SCREEN_W - 20) totalW = SCREEN_W - 20;  // cap to screen

  // Center horizontally if total narrower than screen
  int xStartBase = (SCREEN_W - totalW) / 2;

  // ---------- BUILD ROWS ----------
  int y = 0;
  for (JsonArray row : rows) {
    int numCols = row.size();
    int x = xStartBase;

    for (int ci = 0; ci < numCols && ci < columns; ci++) {
      JsonObject it = row[ci];
      const char *txt = it["text"] | "";
      const char *type = it["type"] | "text";
      const char *key = it["key"] | "";
      int colW = colWidths[ci];

      __LOG_VAR("    type", type);

      // ---------- TEXT ----------
      if (strcmp(type, "text") == 0) {
        lv_obj_t *lbl = lv_label_create(cont);
        lv_label_set_text(lbl, txt);
        lv_obj_set_style_text_font(lbl, btnFont, 0);
        lv_obj_set_style_text_color(lbl, COLOR_BTN_TEXT, 0);
        lv_obj_set_pos(lbl, x, y);
      }

      // ---------- ACTION / MENU ----------
      else if (strcmp(type, "action") == 0 || strcmp(type, "menu") == 0) {
        lv_obj_t *btn = lv_btn_create(cont);
        lv_obj_set_size(btn, colW, 48);
        lv_obj_set_pos(btn, x, y);

        // Pick color based on type and key/text
        lv_color_t colorBtn = COLOR_BTN_ACTION;
        if (strcmp(type, "menu") == 0)
          colorBtn = COLOR_BTN_MENU;
        if (strcasecmp(txt, "back") == 0)
          colorBtn = COLOR_BTN_BACK;  // special Back color

        // === NORMAL STATE ===
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_radius(btn, CORNERS, 0);  // rounder corners
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_border_color(btn, colorBtn, 0);

        // === PRESSED STATE ===
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, colorBtn, LV_STATE_PRESSED);
        lv_obj_set_style_text_color(btn, colorBtn, 0);

        // === LABEL ===
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, txt);
        lv_obj_center(lbl);
        lv_obj_set_style_text_font(lbl, btnFont, 0);

        // === EVENT ===
        lv_obj_add_event_cb(
          btn, [](lv_event_t *e) {
            const char *k = (const char *)lv_event_get_user_data(e);
            // Run the action asynchronously after LVGL finishes the current event
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
        // --- Outer numeric box ---
        lv_obj_t *numBox = lv_obj_create(cont);
        lv_obj_set_size(numBox, colW, 48);
        lv_obj_set_pos(numBox, x, y);
        lv_obj_set_flex_flow(numBox, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(numBox,
                              LV_FLEX_ALIGN_SPACE_BETWEEN,
                              LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_radius(numBox, CORNERS, 0);
        lv_obj_set_style_border_width(numBox, 2, 0);
        lv_obj_set_style_border_color(numBox, COLOR_BTN_NUM, 0);
        lv_obj_set_style_bg_opa(numBox, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(numBox, 4, 0);
        lv_obj_set_style_pad_gap(numBox, 6, 0);

        // --- minus button ---
        lv_obj_t *btnMinus = lv_btn_create(numBox);
        lv_obj_set_size(btnMinus, 40, 40);
        lv_obj_set_style_radius(btnMinus, 20, 0);  // rounder corners
        lv_obj_set_flex_grow(btnMinus, 0);         // fixed width
        lv_obj_set_style_bg_opa(btnMinus, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(btnMinus, 0, 0);
        lv_obj_t *lblMinus = lv_label_create(btnMinus);
        lv_label_set_text(lblMinus, LV_SYMBOL_MINUS);
        lv_obj_center(lblMinus);

        // --- middle numeric label ---
        lv_obj_t *lblVal = lv_label_create(numBox);
        char buf[8];
        snprintf(buf, sizeof(buf), "%04d", getParamValue(key));
        lv_label_set_text(lblVal, buf);
        lv_obj_set_style_text_font(lblVal, FONT_BTN_SMALL_PTR, 0);
        lv_obj_set_style_text_color(lblVal, COLOR_BTN_TEXT, 0);
        lv_obj_set_width(lblVal, LV_PCT(100));  // expand to fill remaining space
        lv_obj_set_style_text_align(lblVal, LV_TEXT_ALIGN_CENTER, 0);

        // --- plus button ---
        lv_obj_t *btnPlus = lv_btn_create(numBox);
        lv_obj_set_size(btnPlus, 40, 40);
        lv_obj_set_style_radius(btnPlus, 20, 0);  // rounder corners
        lv_obj_set_flex_grow(btnPlus, 0);         // fixed width
        lv_obj_set_style_bg_opa(btnPlus, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(btnPlus, 0, 0);
        lv_obj_t *lblPlus = lv_label_create(btnPlus);
        lv_label_set_text(lblPlus, LV_SYMBOL_PLUS);
        lv_obj_center(lblPlus);

        // --- inactive state colors (outline only) ---
        lv_obj_set_style_bg_opa(btnPlus, LV_OPA_TRANSP, 0);
        lv_obj_set_style_bg_opa(btnMinus, LV_OPA_TRANSP, 0);

        // --- Click on number toggles active edit mode ---
        lv_obj_add_event_cb(
          numBox, [](lv_event_t *e) {
            lv_obj_t *box = (lv_obj_t *)lv_event_get_target(e);
            bool editing = lv_obj_has_state(box, LV_STATE_CHECKED);
            lv_obj_t *btnMinus = lv_obj_get_child(box, 0);
            lv_obj_t *lblVal = lv_obj_get_child(box, 1);
            lv_obj_t *btnPlus = lv_obj_get_child(box, 2);

            if (!editing) {
              lv_obj_t *parent_cont = lv_obj_get_parent(box);  // container that holds all rows
              deselect_all_numeric_boxes(parent_cont);
              // enter edit mode
              lv_obj_add_state(box, LV_STATE_CHECKED);
              lv_obj_set_style_bg_opa(btnMinus, LV_OPA_COVER, 0);
              lv_obj_set_style_bg_color(btnMinus, COLOR_BTN_NUM, 0);
              lv_obj_set_style_bg_opa(btnPlus, LV_OPA_COVER, 0);
              lv_obj_set_style_bg_color(btnPlus, COLOR_BTN_NUM, 0);
            } else {
              // TODO - do not leave until another one is selected per above
              // lv_obj_t *parent_cont = lv_obj_get_parent(box);  // container that holds all rows
              // deselect_all_numeric_boxes(parent_cont);
              // leave edit mode
              // lv_obj_clear_state(box, LV_STATE_CHECKED);
              // lv_obj_set_style_bg_opa(btnMinus, LV_OPA_TRANSP, 0);
              // lv_obj_set_style_bg_opa(btnPlus, LV_OPA_TRANSP, 0);
            }
          },
          LV_EVENT_CLICKED, NULL);

        // --- Increment ---
        lv_obj_add_event_cb(
          btnPlus, [](lv_event_t *e) {
            lv_obj_t *box = (lv_obj_t *)lv_obj_get_parent((lv_obj_t *)lv_event_get_target(e));
            const char *k = (const char *)lv_obj_get_user_data(box);
            int val = getParamValue(k) + 1;
            setParamValue(k, val);
            char buf[8];
            snprintf(buf, sizeof(buf), "%04d", val);
            lv_label_set_text(lv_obj_get_child(box, 1), buf);
            setFooter((String("Increment ") + k).c_str());
          },
          LV_EVENT_CLICKED, NULL);

        // --- Decrement ---
        lv_obj_add_event_cb(
          btnMinus, [](lv_event_t *e) {
            lv_obj_t *box = (lv_obj_t *)lv_obj_get_parent((lv_obj_t *)lv_event_get_target(e));
            const char *k = (const char *)lv_obj_get_user_data(box);
            int val = getParamValue(k) - 1;
            setParamValue(k, val);
            char buf[8];
            snprintf(buf, sizeof(buf), "%04d", val);
            lv_label_set_text(lv_obj_get_child(box, 1), buf);
            setFooter((String("Decrement ") + k).c_str());
          },
          LV_EVENT_CLICKED, NULL);
      }

      // ---------- STATUS BUTTON ----------
      else if (strcmp(type, "status_button") == 0) {
        const char *key = it["key"] | "";
        const char *status = getStatusText(key);
        const char *colStr = getStatusColor(key);
        if (strcmp(status, "na") == 0) {  //TODO
          status = it["status"] | "";
          colStr = it["color"] | "gray";
        }
        lv_color_t sColor = colorFromName(colStr);

        lv_obj_t *btn = lv_btn_create(cont);
        lv_obj_set_size(btn, colW, 48);
        lv_obj_set_pos(btn, x, y);
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_color(btn, COLOR_BTN_STATUS, 0);
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_radius(btn, CORNERS, 0);
        lv_obj_set_style_text_font(btn, btnFont, 0);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, status);
        lv_obj_set_style_text_color(lbl, sColor, 0);
        lv_obj_center(lbl);
        statusWidgets[key] = lbl;
        lv_obj_set_style_text_font(lbl, btnFont, 0);

        lv_obj_add_event_cb(
          btn, [](lv_event_t *e) {
            const char *k = (const char *)lv_event_get_user_data(e);
            // Run the action asynchronously after LVGL finishes the current event
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
        if (strcmp(status, "na") == 0) {  //TODO
          status = it["status"] | "";
          colStr = it["color"] | "gray";
        }
        lv_color_t sColor = colorFromName(colStr);
        lv_obj_t *lbl = lv_label_create(cont);
        lv_label_set_text(lbl, status);
        lv_obj_set_style_text_color(lbl, sColor, 0);
        lv_obj_set_pos(lbl, x, y);
        lv_obj_center(lbl);
        statusWidgets[key] = lbl;
      }
      x += colW + 8;  // horizontal spacing
    }
    y += 65;  // vertical spacing (tweak as needed)
  }

  // Footer
  footLbl = lv_label_create(scr);
  lv_label_set_text(footLbl, footer);
  lv_obj_set_style_text_font(footLbl, FONT_FOOT_PTR, 0);
  lv_obj_align(footLbl, LV_ALIGN_BOTTOM_MID, 0, -10);

  __LOG_VAR("build menu", "end");
}

//TODO change the servo limits to set the zero instead of crr


// ----------------------------------------------------------
//                     Example JSON with all types
// ----------------------------------------------------------
static char jsonBuffer[] = R"json(
{
  "main": {
    "title": "Rubik Cube Solver", "subtitle": "ver 10 (Oct 2025)", "footer": "Ready", "columns": 1,
    "rows": [
      [{ "text": "Solve Cube", "type": "menu", "key": "solve" }],
      [{ "text": "Read Cube", "type": "menu", "key": "read" }],
      [{ "text": "Scramble Cube", "type": "menu", "key": "random" }],
      [{ "text": "Tests", "type": "menu", "key": "tests" }]
    ]
  },
  "solve": {
    "title": "Solve Cube", "subtitle": "ver 8 (oct 2025)", "footer": "Press to start solving.", "columns": 1,
    "rows": [
      [{ "text": "Start", "type": "action", "key": "run_solve" }],
      [{ "text": "Back", "type": "menu", "key": "main" }]
    ]
  },
  "read": {
    "title": "Read Cube", "subtitle": "ver 10 (Oct 2025)", "footer": "Read cube colors.", "columns": 1,
    "rows": [
      [{ "text": "Start Read", "type": "action", "key": "run_read" }],
      [{ "text": "Back", "type": "menu", "key": "main" }]
    ]
  },
  "random": {
    "title": "Scramble Cube", "subtitle": "ver 8 (oct 2025)", "footer": "Scramble cube randomly.", "columns": 1,
    "rows": [
      [{ "text": "Scramble (12)", "type": "action", "key": "scramble_12" }],
      [{ "text": "Scramble (20)", "type": "action", "key": "scramble_20" }],
      [{ "text": "Back", "type": "menu", "key": "main" }]
    ]
  },
  "tests": {
    "title": "Tests", "subtitle": "Test & Diagnostics", "footer": "Perform hardware tests.", "columns": 2,
    "rows": [
      [{ "text": "Servo IDs", "type": "menu", "key": "servos_programming" }, 
       { "text": "Servo Limits", "type": "menu", "key": "servo_limits" }],

      [{ "text": "Vertical Tune", "type": "menu", "key": "vertical_tune" }, 
       { "text": "", "type": "text", "key": "" }],

      [{ "text": "Poses", "type": "menu", "key": "poses" }, 
       { "text": "Pose Groups", "type": "menu", "key": "pose_groups" }],

      [{ "text": "Sequences", "type": "menu", "key": "sequences" }, 
       { "text": "Cube Moves", "type": "menu", "key": "cube_moves" }],

      [{ "text": "Back", "type": "menu", "key": "main" }, 
       { "text": "", "type": "text", "key": "" }]
    ]
  },
  "servo_limits": {
    "title": "Servo Limits", "subtitle": "Adjust range", "footer": "Touch to edit numeric limits.", "columns": 4,
    "rows": [
      [{ "text": "Servo", "type": "text" }, 
       { "text": "0 degree ", "type": "text" }, 
       { "text": "Min Limit,", "type": "text" }, 
       { "text": "Max Limit,", "type": "text" }],

      [{ "text": "Arm1", "type": "text" }, 
       { "text": "", "type": "num", "key": "arm1_0deg" }, 
       { "text": "", "type": "num", "key": "arm1_min" }, 
       { "text": "", "type": "num", "key": "arm1_max" }],

      [{ "text": "Arm2", "type": "text" }, 
       { "text": "", "type": "num", "key": "arm2_0deg" }, 
       { "text": "", "type": "num", "key": "arm2_min" }, 
       { "text": "", "type": "num", "key": "arm2_max" }],

      [{ "text": "Wrist", "type": "text" }, 
       { "text": "", "type": "num", "key": "wrist_0deg" }, 
       { "text": "", "type": "num", "key": "wrist_min" }, 
       { "text": "", "type": "num", "key": "wrist_max" }],

      [{ "text": "Grip L", "type": "text" }, 
       { "text": "", "type": "num", "key": "grip1_0deg" }, 
       { "text": "", "type": "num", "key": "grip1_min" }, 
       { "text": "", "type": "num", "key": "grip1_max" }],

      [{ "text": "Grip R", "type": "text" }, 
       { "text": "", "type": "num", "key": "grip2_0deg" }, 
       { "text": "", "type": "num", "key": "grip2_min" }, 
       { "text": "", "type": "num", "key": "grip2_max" }],

      [{ "text": "Base", "type": "text" }, 
       { "text": "", "type": "num", "key": "base_0deg" }, 
       { "text": "", "type": "num", "key": "base_min" }, 
       { "text": "", "type": "num", "key": "base_max" }],

      [{ "text": "Back", "type": "menu", "key": "tests" }, 
       { "text": "", "type": "text", "key": "" }, 
       { "text": "", "type": "text", "key": "" }, 
       { "text": "", "type": "text", "key": "" }]
    ]
  },
  "servos_programming": {
    "title": "Tests", "subtitle": "Servos Programming", "footer": "Set servo id and reset to 0", "columns": 4,
    "rows": [
      [{ "text": "Arm1 (id11)", "type": "text", "key": "" }, 
       { "text": "Program", "type": "status_button", "key": "arm1_program", "status": "Program" }, 
       { "text": "Test", "type": "status_button", "key": "arm1_test", "status": "Test" }, 
       { "text": "Status", "type": "status_text", "key": "arm1_status", "status": "na" }],
      [{ "text": "Arm2 (id12)", "type": "text", "key": "" }, 

       { "text": "Program", "type": "status_button", "key": "arm2_program", "status": "Program" }, 
       { "text": "Test", "type": "status_button", "key": "arm2_test", "status": "Test" }, 
       { "text": "Status", "type": "status_text", "key": "arm2_status", "status": "na" }],
      [{ "text": "Wrist (id13)", "type": "text", "key": "" }, 

       { "text": "Program", "type": "status_button", "key": "wrist_program", "status": "Program" }, 
       { "text": "Test", "type": "status_button", "key": "wrist_test", "status": "Test" }, 
       { "text": "Status", "type": "status_text", "key": "wrist_status", "status": "na" }],
      [{ "text": "Grip L (id14)", "type": "text", "key": "" }, 

       { "text": "Program", "type": "status_button", "key": "grip1_program", "status": "Program" }, 
       { "text": "Test", "type": "status_button", "key": "grip1_test", "status": "Test" }, 
       { "text": "Status", "type": "status_text", "key": "grip1_status", "status": "na" }],
      [{ "text": "Grip R (id15)", "type": "text", "key": "" }, 

       { "text": "Program", "type": "status_button", "key": "grip2_program", "status": "Program" }, 
       { "text": "Test", "type": "status_button", "key": "grip2_test", "status": "Test" }, 
       { "text": "Status", "type": "status_text", "key": "grip2_status", "status": "na" }],
      [{ "text": "Base (id16)", "type": "text", "key": "" }, 

       { "text": "Program", "type": "status_button", "key": "base_program", "status": "Program" }, 
       { "text": "Test", "type": "status_button", "key": "base_test", "status": "Test" }, 
       { "text": "Status", "type": "status_text", "key": "base_status", "status": "na" }],

      [{ "text": "Back", "type": "menu", "key": "tests" }, 
       { "text": "", "type": "text" }, 
       { "text": "", "type": "text" }, 
       { "text": "", "type": "text" }]
    ]
  },
  "vertical_tune": {
    "title": "Tests", "subtitle": "Vertical Tune", "footer": "Adjust for veritcal move arm1, arm2, wrist", "columns": 4,
    "rows": [
      [{ "text": "mm", "type": "text", "key": "grip_open" }, 
       { "text": " Arm1 ", "type": "text" }, 
       { "text": " Arm2 ", "type": "text", "key": "grip_close" }, 
       { "text": "Wrist", "type": "text" }],

      [{ "text": "0 mm", "type": "action", "key": "vertical_0mm" }, 
       { "text": "Arm1", "type": "num", "key": "arm1_0mm" }, 
       { "text": "Arm2", "type": "num", "key": "arm2_0mm" }, 
       { "text": "Wrist", "type": "num", "key": "wrist_0mm" }],

      [{ "text": "5 mm", "type": "action", "key": "vertical_5mm" }, 
       { "text": "Arm1", "type": "num", "key": "arm1_5mm" }, 
       { "text": "Arm2", "type": "num", "key": "arm2_5mm" }, 
       { "text": "Wrist", "type": "num", "key": "wrist_5mm" }],

      [{ "text": "10 mm", "type": "action", "key": "vertical_10mm" }, 
       { "text": "Arm1", "type": "num", "key": "arm1_10mm" }, 
       { "text": "Arm2", "type": "num", "key": "arm2_10mm" }, 
       { "text": "Wrist", "type": "num", "key": "wrist_10mm" }],

      [{ "text": "15 mm", "type": "action", "key": "vertical_15mm" }, 
       { "text": "Arm1", "type": "num", "key": "arm1_15mm" }, 
       { "text": "Arm2", "type": "num", "key": "arm2_15mm" }, 
       { "text": "Wrist", "type": "num", "key": "wrist_15mm" }],

      [{ "text": "", "type": "text" }, 
       { "text": "G Open ", "type": "action", "key": "grip_open" }, 
       { "text": "G Close", "type": "action", "key": "grip_close" }, 
       { "text": "", "type": "text" }],

      [{ "text": "Back", "type": "menu", "key": "tests" }, 
       { "text": "", "type": "text" }, 
       { "text": "", "type": "text" }, 
       { "text": "", "type": "text" }]
    ]
  },
  "poses": {
    "title": "Servo Poses", "subtitle": "Calibration", "footer": "Tap to edit pose values.", "columns": 3,
    "rows": [
       [{ "text": "Arm1 0", "type": "action", "key": "arm1_0" },
        { "text": "", "type": "num", "key": "arm1_pose_0" },
        { "text": "", "type": "status_text", "key": "arm1_current" }],

       [{ "text": "Arm2 0", "type": "action", "key": "arm2_0" },
        { "text": "", "type": "num", "key": "arm2_pose_0" },
        { "text": "", "type": "status_text", "key": "arm2_current" }],

       [{ "text": "Wrist 0", "type": "action", "key": "wrist_0" },
        { "text": "", "type": "num", "key": "wrist_pose_0" },
        { "text": "", "type": "status_text", "key": "wrist_current" }],

       [{ "text": "Wrist 90", "type": "action", "key": "wrist_90" },
        { "text": "", "type": "num", "key": "wrist_pose_90" },
        { "text": "", "type": "status_text", "key": "wrist_current" }],

      [{ "text": "Wrist -90", "type": "action", "key": "wrist_minus90" },
        { "text": "", "type": "num", "key": "wrist_pose_minus90" },
        { "text": "", "type": "status_text", "key": "wrist_current" }],

       [{ "text": "Grip1 0", "type": "action", "key": "grip1_0" },
        { "text": "", "type": "num", "key": "grip1_pose_0" },
        { "text": "", "type": "status_text", "key": "grip1_current" }],

       [{ "text": "Grip1 1", "type": "action", "key": "grip1_1" },
        { "text": "", "type": "num", "key": "grip1_pose_1" },
        { "text": "", "type": "status_text", "key": "grip1_current" }],

       [{ "text": "Grip2 0", "type": "action", "key": "grip2_0" },
        { "text": "", "type": "num", "key": "grip2_pose_0" },
        { "text": "", "type": "status_text", "key": "grip2_current" }],

       [{ "text": "Grip2 1", "type": "action", "key": "grip2_1" },
        { "text": "", "type": "num", "key": "grip2_pose_1" },
        { "text": "", "type": "status_text", "key": "grip2_current" }],

       [{ "text": "Base 0", "type": "action", "key": "base_0" },
        { "text": "", "type": "num", "key": "base_pose_0" },
        { "text": "", "type": "status_text", "key": "base_current" }],

       [{ "text": "Base 90", "type": "action", "key": "base_90" },
        { "text": "", "type": "num", "key": "base_pose_90" },
        { "text": "", "type": "status_text", "key": "base_current" }],

       [{ "text": "Base -90", "type": "action", "key": "base_minus90" },
        { "text": "", "type": "num", "key": "base_pose_minus90" },
        { "text": "", "type": "status_text", "key": "base_current" }],

       [{ "text": "Back", "type": "menu", "key": "tests" },
        { "text": "", "type": "text" },
        { "text": "", "type": "text" }]
    ]
  },
  "pose_groups": {
    "title": "Pose Groups", "subtitle": "Group actions", "footer": "Tap to run pose groups.", "columns": 3,
    "rows": [
      [{ "text": "Arms Vertical 0", "type": "action", "key": "arms_0" }, 
       { "text": "Arms Vertical Mid", "type": "action", "key": "arms_row1" }, 
       { "text": "Arms Vertical High", "type": "action", "key": "arms_row1" }],

      [{ "text": "Wrist Vertical 0", "type": "action", "key": "wrist_v0" }, 
       { "text": "Wrist Vertical 90", "type": "action", "key": "wrist_v90" }, 
       { "text": "Wrist Vertical -90", "type": "action", "key": "wrist_vminus90" }],

      [{ "text": "Back", "type": "menu", "key": "tests" }, 
       { "text": "", "type": "text" }, 
       { "text": "", "type": "text" }]
    ]
  },
  "sequences": {
    "title": "Sequences", "subtitle": "Motion Macros", "footer": "Tap to execute motion sequence.", "columns": 2,
    "rows": [
      [{ "text": "Bottom+", "type": "action", "key": "bottom_plus" }, 
       { "text": "Bottom-", "type": "action", "key": "bottom_minus" }],

      [{ "text": "Front to Base", "type": "action", "key": "front_to_base" }, 
       { "text": "Back to Base", "type": "action", "key": "back_to_base" }],

      [{ "text": "Left to Base", "type": "action", "key": "left_to_base" }, 
       { "text": "Right to Base", "type": "action", "key": "right_to_base" }],

      [{ "text": "Top to Base", "type": "action", "key": "top_to_base" }, 
       { "text": "", "type": "text", "key": "" }],

      [{ "text": "Rotate Down Face+", "type": "action", "key": "rotate_down_90" }, 
       { "text": "Rotate Down Face-", "type": "action", "key": "rotate_down_minus90" }],

      [{ "text": "Back", "type": "menu", "key": "tests" }, 
       { "text": "", "type": "text" }]
    ]
  },
  "cube_moves": {
    "title": "Cube Moves", "subtitle": "Standard rotations", "footer": "Tap to run a move.", "columns": 4,
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
       { "text": "Back", "type": "menu", "key": "tests" }, 
       { "text": "", "type": "text" }]
    ]
  }
}
)json";

// ----------------------------------------------------------
//                     LVGL INITIALIZATION
// ----------------------------------------------------------
void ui_init() {
  // 1. Initialize LVGL core
  lv_init();

  // 2. Initialize the display hardware
  static Arduino_H7_Video Display(800, 480, GigaDisplayShield);
  static Arduino_GigaDisplayTouch TouchDetector;

  Display.begin();  // sets up framebuffer and LVGL display driver
  TouchDetector.begin();

  // 3. Parse JSON AFTER display init
  DeserializationError err = deserializeJson(menuDoc, jsonBuffer);
  if (err) {
    __LOG_VAR("json error", err.c_str());
    return;  // avoid using empty doc
  }
  __LOG_VAR("json ok", err.c_str());
  __LOG_VAR("   top keys", menuDoc.as<JsonObject>().size());
  __LOG_VAR("   mem used", menuDoc.memoryUsage());

  buildMenu(currentMenu.c_str());
}

static void deselect_all_numeric_boxes(lv_obj_t *parent) {
  uint32_t child_count = lv_obj_get_child_cnt(parent);
  for (uint32_t i = 0; i < child_count; i++) {
    lv_obj_t *child = lv_obj_get_child(parent, i);
    if (!child) continue;

    // If the child is a numeric box in edit mode, clear it
    if (lv_obj_has_state(child, LV_STATE_CHECKED)) {
      lv_obj_clear_state(child, LV_STATE_CHECKED);

      // hide +/- if present
      lv_obj_t *btnMinus = lv_obj_get_child(child, 0);
      lv_obj_t *btnPlus = lv_obj_get_child(child, 2);
      if (btnMinus && btnPlus) {
        lv_obj_set_style_bg_opa(btnMinus, LV_OPA_TRANSP, 0);
        lv_obj_set_style_bg_opa(btnPlus, LV_OPA_TRANSP, 0);
      }
    }
  }
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
  Serial.begin(115200);
  while (!Serial && millis() < 9999) delay(111);
  delay(333);

  __LOG_VAR("setup", "start");

  ui_init();
  dynamixel_begin();

  __LOG_VAR("setup", "end");
}

void loop() {
  ui_loop();
  delay(5);
}

/*
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
