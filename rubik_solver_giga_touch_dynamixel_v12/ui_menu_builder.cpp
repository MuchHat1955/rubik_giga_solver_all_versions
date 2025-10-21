#include "ui_touch.h"
#include "logging.h"
#include "ui_theme.h"
#include "ui_touch.h"
#include "param_store.h"
#include <map>
#include <vector>
#include <algorithm>

String getSketchVersion();
String getSketchVersionWithDate();
void buildMenu(const char *menuName);

// ----------------------------------------------------------
//                     MENU BUILDER
// ----------------------------------------------------------
void buildMenu(const char *menuName) {

  LOG_SECTION_START_VAR("build menu", "menu", menuName);

  lv_obj_t *scr = lv_scr_act();
  if (!scr || !menuDoc.containsKey(menuName)) {
    LOG_SECTION_END();
    return;
  }

  // clear previous screen and reset pointers
  lv_obj_clean(scr);
  footLbl = nullptr;  // reset footer label so it is recreated
  statusWidgets.clear();
  numLabels.clear();

  JsonObject root = menuDoc[menuName];
  if (root.isNull()) {
    LOG_SECTION_END();
    return;
  }

  String fullTitle = String(root["title"] | "");

  // add version only for the main menu
  if (strcmp(menuName, "main") == 0) {
    String sketchVer = getSketchVersion();
    if (sketchVer.length()) fullTitle += " " + sketchVer;
  }

  const char *title = fullTitle.c_str();
  const char *footer = root["footer"] | "";
  const char *equalColumns = root["equal columns"] | "";
  int columns = root["columns"] | 1;
  JsonArray rows = root["rows"].as<JsonArray>();

  lv_obj_set_style_bg_color(scr, COLOR_BG, 0);
  lv_obj_set_style_text_color(scr, COLOR_TEXT, 0);

  // ----------------------------------------------------------
  //                     TITLE
  // ----------------------------------------------------------
  lv_obj_t *titleLbl = lv_label_create(scr);
  lv_label_set_text(titleLbl, title);
  lv_obj_set_style_text_font(titleLbl, FONT_TITLE_PTR, 0);
  lv_obj_align(titleLbl, LV_ALIGN_TOP_MID, 0, 6);

  const lv_font_t *btnFont = FONT_BTN_LARGE_PTR;

  // ----------------------------------------------------------
  //              SCROLLABLE CONTAINER AND FOOTER
  // ----------------------------------------------------------
  lv_obj_t *cont = lv_obj_create(scr);
  lv_obj_set_layout(cont, LV_LAYOUT_NONE);
  lv_obj_set_scroll_dir(cont, LV_DIR_VER);
  lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(cont, 0, 0);

  lv_obj_update_layout(titleLbl);
  int title_h = lv_obj_get_height(titleLbl);

  // recreate footer every time
  footLbl = lv_label_create(scr);
  lv_obj_set_style_text_font(footLbl, FONT_FOOT_PTR, 0);
  lv_label_set_text(footLbl, footer);
  lv_obj_align(footLbl, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_update_layout(footLbl);
  int footer_h = lv_obj_get_height(footLbl);

  // compute available height between title and footer
  int available_h = SCREEN_H - (title_h + footer_h + 60);
  if (available_h < 60) available_h = 60;

  // snap container height to full rows
  const int rowH = 65;
  int contH = available_h - (available_h % rowH);
  if (contH < rowH) contH = rowH;

  lv_obj_set_size(cont, SCREEN_W, contH);
  lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, title_h + 20);

  int adjustedRowH = rowH;  // fixed row pitch

  // ----------------------------------------------------------
  //                DYNAMIC COLUMN WIDTH
  // ----------------------------------------------------------
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
        txt = "-  0000  +";
        extraPad = 26;
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

  // ----------------------------------------------------------
  //                     BUILD ROWS
  // ----------------------------------------------------------
  extern lv_obj_t *selected_num_box;
  extern lv_style_t style_num_selected;

  int y = 0;
  for (JsonArray row : rows) {
    int numCols = row.size();

    // compute X start
    int totalRowW = 0;
    for (int ci = 0; ci < numCols && ci < columns; ci++)
      if (ci < (int)colWidths.size()) totalRowW += colWidths[ci] + 8;
    if (totalRowW > 0) totalRowW -= 8;

    int x = (numCols == 1)
              ? (SCREEN_W - totalRowW) / 2
              : xStartbase;

    for (int ci = 0; ci < numCols && ci < columns; ci++) {
      JsonObject it = row[ci];
      const char *txt = it["text"] | "";
      const char *type = it["type"] | "text";
      const char *key = it["key"] | "";
      int colW = colWidths[ci];

      // ---------- TEXT ----------
      if (strcmp(type, "text") == 0) {
        lv_obj_t *cell = lv_obj_create(cont);
        lv_obj_remove_style_all(cell);
        lv_obj_set_size(cell, colW, 48);
        lv_obj_set_pos(cell, x, y);
        lv_obj_set_style_bg_opa(cell, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(cell, 0, 0);

        lv_obj_t *lbl = lv_label_create(cell);
        lv_label_set_text(lbl, txt);
        lv_obj_set_style_text_font(lbl, btnFont, 0);
        lv_obj_set_style_text_color(lbl, COLOR_BTN_TEXT, 0);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_opa(lbl, LV_OPA_COVER, 0);
        lv_obj_center(lbl);

        LOG_VAR("create <text>", txt);
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

        LOG_VAR("create <action or menu>", txt);

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

        // Outer numeric container
        lv_obj_t *numBox = lv_obj_create(cont);
        lv_obj_remove_style_all(numBox);        // remove default theme style
        lv_obj_set_size(numBox, colW - 2, 44);  // slightly narrower
        lv_obj_set_pos(numBox, x + 1, y + 2);   // center vertically

        lv_obj_set_flex_flow(numBox, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(numBox,
                              LV_FLEX_ALIGN_SPACE_BETWEEN,
                              LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);

        // Visible outline for numeric box
        lv_obj_set_style_border_width(numBox, 2, 0);
        lv_obj_set_style_border_color(numBox, COLOR_BTN_NUM, 0);
        lv_obj_set_style_radius(numBox, CORNERS, 0);
        lv_obj_set_style_bg_opa(numBox, LV_OPA_TRANSP, 0);

        lv_obj_add_flag(numBox, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(numBox, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(numBox, LV_OBJ_FLAG_SCROLL_CHAIN);

        // --- MINUS button ---
        lv_obj_t *btnMinus = lv_btn_create(numBox);
        lv_obj_remove_style_all(btnMinus);
        lv_obj_set_size(btnMinus, 40, 40);
        lv_obj_set_style_radius(btnMinus, 20, 0);
        lv_obj_set_style_border_width(btnMinus, 2, 0);
        lv_obj_set_style_border_color(btnMinus, COLOR_BTN_NUM, 0);
        lv_obj_set_style_bg_opa(btnMinus, LV_OPA_TRANSP, 0);
        lv_obj_add_flag(btnMinus, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(btnMinus, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_add_style(btnMinus, &style_num_btn_pressed, LV_STATE_PRESSED);

        lv_obj_t *lblMinus = lv_label_create(btnMinus);
        lv_label_set_text(lblMinus, LV_SYMBOL_MINUS);
        lv_obj_set_style_text_font(lblMinus, FONT_BTN_SMALL_PTR, 0);
        lv_obj_set_style_text_color(lblMinus, COLOR_BTN_TEXT, 0);
        lv_obj_center(lblMinus);

        // --- VALUE label ---
        lv_obj_t *lblVal = lv_label_create(numBox);
        char buf[8];
        snprintf(buf, sizeof(buf), "%04d", getParamValue(key));
        lv_label_set_text(lblVal, buf);
        lv_obj_set_style_text_font(lblVal, FONT_BTN_SMALL_PTR, 0);
        lv_obj_set_style_text_color(lblVal, COLOR_BTN_TEXT, 0);
        lv_obj_set_style_text_align(lblVal, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_flag(lblVal, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(lblVal, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_clear_flag(lblVal, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(lblVal, LV_OBJ_FLAG_SCROLL_CHAIN);
        lv_obj_center(lblVal);

        // --- PLUS button ---
        lv_obj_t *btnPlus = lv_btn_create(numBox);
        lv_obj_remove_style_all(btnPlus);
        lv_obj_set_size(btnPlus, 40, 40);
        lv_obj_set_style_radius(btnPlus, 20, 0);
        lv_obj_set_style_border_width(btnPlus, 2, 0);
        lv_obj_set_style_border_color(btnPlus, COLOR_BTN_NUM, 0);
        lv_obj_set_style_bg_opa(btnPlus, LV_OPA_TRANSP, 0);
        lv_obj_add_flag(btnPlus, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(btnPlus, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_add_style(btnPlus, &style_num_btn_pressed, LV_STATE_PRESSED);

        lv_obj_t *lblPlus = lv_label_create(btnPlus);
        lv_label_set_text(lblPlus, LV_SYMBOL_PLUS);
        lv_obj_set_style_text_font(lblPlus, FONT_BTN_SMALL_PTR, 0);
        lv_obj_set_style_text_color(lblPlus, COLOR_BTN_TEXT, 0);
        lv_obj_center(lblPlus);

        // --- layout tweaks inside numBox ---
        lv_obj_set_style_pad_left(numBox, 4, 0);
        lv_obj_set_style_pad_right(numBox, 4, 0);
        lv_obj_set_style_pad_gap(numBox, 8, 0);
        lv_obj_set_style_translate_x(btnMinus, -4, 0);
        lv_obj_set_style_translate_x(btnPlus, 4, 0);

        LOG_VAR("create <num>", buf);

        // --- selection by tapping number ---
        lv_obj_add_event_cb(
          lblVal, [](lv_event_t *e) {
            if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
              lv_obj_t *val = (lv_obj_t *)lv_event_get_target(e);
              lv_obj_t *numBoxP = lv_obj_get_parent(val);
              select_num_pair(numBoxP, /*toggle=*/true);
            }
          },
          LV_EVENT_CLICKED, nullptr);

        // --- MINUS click ---
        lv_obj_add_event_cb(
          btnMinus, [](lv_event_t *e) {
            const char *keyUD = (const char *)lv_event_get_user_data(e);
            if (!keyUD || !*keyUD) return;
            select_num_pair(lv_obj_get_parent((lv_obj_t *)lv_event_get_target(e)), false);
            incrementParam(keyUD, -1);
            int val = getParamValue(keyUD);
            char b[8];
            snprintf(b, sizeof(b), "%04d", val);
            lv_obj_t *lbl = lv_obj_get_child(lv_obj_get_parent((lv_obj_t *)lv_event_get_target(e)), 1);
            lv_label_set_text(lbl, b);
          },
          LV_EVENT_CLICKED, (void *)key);

        // --- PLUS click ---
        lv_obj_add_event_cb(
          btnPlus, [](lv_event_t *e) {
            const char *keyUD = (const char *)lv_event_get_user_data(e);
            if (!keyUD || !*keyUD) return;
            select_num_pair(lv_obj_get_parent((lv_obj_t *)lv_event_get_target(e)), false);
            incrementParam(keyUD, +1);
            int val = getParamValue(keyUD);
            char b[8];
            snprintf(b, sizeof(b), "%04d", val);
            lv_obj_t *lbl = lv_obj_get_child(lv_obj_get_parent((lv_obj_t *)lv_event_get_target(e)), 1);
            lv_label_set_text(lbl, b);
          },
          LV_EVENT_CLICKED, (void *)key);
      }

      x += colW + 8;
    }
    y += adjustedRowH;
  }

  // force refresh
  lv_refr_now(NULL);

  LOG_SECTION_END();
}

// ----------------------------------------------------------
//                     MENU JSON DATA
// ----------------------------------------------------------
const char jsonBuffer[] = R"json(
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
        { "text": "", "type": "text", "key": "blank" }
      ],
      [
        { "text": "poses", "type": "menu", "key": "poses" },
        { "text": "pose groups", "type": "menu", "key": "pose_groups" }
      ],
      [
        { "text": "sequences", "type": "menu", "key": "sequences" },
        { "text": "cube moves", "type": "menu", "key": "cube_moves" }
      ],
      [ { "text": "back", "type": "menu", "key": "main" } ]
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
      [ { "text": "back", "type": "menu", "key": "tests" } ]
    ]
  },
  "servos_programming": {
    "title": "tests",
    "footer": "set servo id and reset to mid range",
    "columns": 4,
    "equal columns":"last",
    "rows": [
      [
        { "text": "arm1 (id11)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "arm1_program", "status": "program" },
        { "text": "test", "type": "action", "key": "arm1_test", "status": "test" },
        { "text": "status", "type": "text", "key": "arm1_status" }
      ],
      [
        { "text": "arm2 (id12)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "arm2_program", "status": "program" },
        { "text": "test", "type": "action", "key": "arm2_test", "status": "test" },
        { "text": "status", "type": "text", "key": "arm2_status" }
      ],
      [
        { "text": "wrist (id13)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "wrist_program", "status": "program" },
        { "text": "test", "type": "action", "key": "wrist_test", "status": "test" },
        { "text": "status", "type": "text", "key": "wrist_status" }
      ],
      [
        { "text": "grip l (id14)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "grip1_program", "status": "program" },
        { "text": "test", "type": "action", "key": "grip1_test", "status": "test" },
        { "text": "status", "type": "text", "key": "grip1_status" }
      ],
      [
        { "text": "grip r (id15)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "grip2_program", "status": "program" },
        { "text": "test", "type": "action", "key": "grip2_test", "status": "test" },
        { "text": "status", "type": "text", "key": "grip2_status" }
      ],
      [
        { "text": "base (id16)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "base_program", "status": "program" },
        { "text": "test", "type": "action", "key": "base_test", "status": "test" },
        { "text": "status", "type": "text", "key": "base_status" }
      ],
      [ { "text": "back", "type": "menu", "key": "tests" } ]
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
        { "text": "0mm", "type": "action", "key": "vertical_0mm" },
        { "text": "arm1", "type": "num", "key": "arm1_0mm" },
        { "text": "arm2", "type": "num", "key": "arm2_0mm" },
        { "text": "wrist", "type": "num", "key": "wrist_0mm" }
      ],
      [
        { "text": "5mm", "type": "action", "key": "vertical_5mm" },
        { "text": "arm1", "type": "num", "key": "arm1_5mm" },
        { "text": "arm2", "type": "num", "key": "arm2_5mm" },
        { "text": "wrist", "type": "num", "key": "wrist_5mm" }
      ],
      [
        { "text": "10mm", "type": "action", "key": "vertical_10mm" },
        { "text": "arm1", "type": "num", "key": "arm1_10mm" },
        { "text": "arm2", "type": "num", "key": "arm2_10mm" },
        { "text": "wrist", "type": "num", "key": "wrist_10mm" }
      ],
      [
        { "text": "15mm", "type": "action", "key": "vertical_15mm" },
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
      [ { "text": "back", "type": "menu", "key": "tests" } ]
    ]
  },
  "poses": {
    "title": "servo poses", "footer": "tap to edit pose values", 
    "columns": 3,
    "equal columns":"all",
    "rows": [
       [{ "text": "arm1 0", "type": "action", "key": "arm1_0" },
        { "text": "+0000-", "type": "num", "key": "arm1_pose_0" },
        { "text": "current na", "type": "text", "key": "arm1_current" }],

       [{ "text": "arm2 0", "type": "action", "key": "arm2_0" },
        { "text": "+0000-", "type": "num", "key": "arm2_pose_0" },
        { "text": "current na", "type": "text", "key": "arm2_current" }],

       [{ "text": "wrist 0", "type": "action", "key": "wrist_0" },
        { "text": "+0000-", "type": "num", "key": "wrist_pose_0" },
        { "text": "current na", "type": "text", "key": "wrist_current" }],

       [{ "text": "wrist 90", "type": "action", "key": "wrist_90" },
        { "text": "+0000-", "type": "num", "key": "wrist_pose_90" },
        { "text": "current na", "type": "text", "key": "wrist_current" }],

      [{ "text": "wrist -90", "type": "action", "key": "wrist_minus90" },
        { "text": "+0000-", "type": "num", "key": "wrist_pose_minus90" },
        { "text": "current na", "type": "text", "key": "wrist_current" }],

       [{ "text": "grip1 Open", "type": "action", "key": "grip1_open" },
        { "text": "+0000-", "type": "num", "key": "grip1_pose_0" },
        { "text": "current na", "type": "text", "key": "grip1_current" }],

       [{ "text": "grip1 Close", "type": "action", "key": "grip1_close" },
        { "text": "+0000-", "type": "num", "key": "grip1_pose_1" },
        { "text": "current na", "type": "text", "key": "grip1_current" }],

       [{ "text": "grip2 Open", "type": "action", "key": "grip2_open" },
        { "text": "+0000-", "type": "num", "key": "grip2_pose_0" },
        { "text": "current na", "type": "text", "key": "grip2_current" }],

       [{ "text": "grip2 Close", "type": "action", "key": "grip2_close" },
        { "text": "+0000-", "type": "num", "key": "grip2_pose_1" },
        { "text": "current na", "type": "text", "key": "grip2_current" }],

       [{ "text": "base 0", "type": "action", "key": "base_0" },
        { "text": "+0000-", "type": "num", "key": "base_pose_0" },
        { "text": "current na", "type": "text", "key": "base_current" }],

       [{ "text": "base 90", "type": "action", "key": "base_90" },
        { "text": "+0000-", "type": "num", "key": "base_pose_90" },
        { "text": "current na", "type": "text", "key": "base_current" }],

       [{ "text": "base -90", "type": "action", "key": "base_minus90" },
        { "text": "+0000-", "type": "num", "key": "base_pose_minus90" },
        { "text": "current na", "type": "text", "key": "base_current" }],

       [{ "text": "back", "type": "menu", "key": "tests" }]
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

      [{ "text": "back", "type": "menu", "key": "tests" }] 
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
       { "text": "", "type": "text", "key": "blank" }],

      [{ "text": "rotate down face+", "type": "action", "key": "rotate_down_90" }, 
       { "text": "rotate down face-", "type": "action", "key": "rotate_down_minus90" }],

      [{ "text": "back", "type": "menu", "key": "tests" }]
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
       { "text": "", "type": "text", "key": "blank" }, 
       { "text": "", "type": "text", "key": "blank" }],
      [{ "text": "back", "type": "menu", "key": "tests" }]
    ]
  }
}
)json";
