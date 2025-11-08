#include <map>
#include <vector>
#include <algorithm>

#include "ui_touch.h"
#include "logging.h"
#include "ui_theme.h"
#include "param_store.h"
#include "ui_status.h"
#include "rb_interface.h"

String getSketchVersion();
String getSketchVersionWithDate();
void buildMenu(const char *menuName);
extern RBInterface rb;

// ---- below is for actions to be done when a menu is displayed ---
void onBuildMenu(const char *menuName) {
  LOG_SECTION_START_VAR("on build menu", "for menu", menuName);

  if (strcmp(menuName, "poses") == 0) {
    LOG_SECTION_START_VAR("update servos for poses UI", "menu", menuName);
    rb.updateInfo();  //TODO check for error etc, now it checks for pos errors
    LOG_SECTION_END();
  }
  LOG_SECTION_END();
}

// ----------------------------------------------------------
//                     MENU BUILDER (safe version)
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
  footLbl = nullptr;
  statusWidgets.clear();
  numLabels.clear();
  uiStatusClear();  // clear persistent button map on rebuild

  JsonObject root = menuDoc[menuName];
  if (root.isNull()) {
    LOG_SECTION_END();
    return;
  }

  String fullTitle = String(root["title"] | "");
  if (strcmp(menuName, "main") == 0) {
    String sketchVer = getSketchVersion();
    if (sketchVer.length()) fullTitle += " " + sketchVer;
  }

  const char *title = fullTitle.c_str();
  const char *footer = root["footer"] | "";
  const char *equalColumns = root["equal_columns"] | "";
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

  footLbl = lv_label_create(scr);
  lv_obj_set_style_text_font(footLbl, FONT_FOOT_PTR, 0);
  lv_label_set_text(footLbl, footer);
  lv_obj_align(footLbl, LV_ALIGN_BOTTOM_MID, 0, -18);
  lv_obj_update_layout(footLbl);
  int footer_h = lv_obj_get_height(footLbl);

  int available_h = SCREEN_H - (title_h + footer_h + 50);
  if (available_h < 60) available_h = 60;

  const int rowH = 62;
  int contH = available_h - (available_h % rowH);
  if (contH < rowH) contH = rowH;
  contH += 8;
  lv_obj_set_size(cont, SCREEN_W, contH);
  lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, title_h + 18);

  int adjustedRowH = rowH;

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
      lv_point_t sz;
      int extraPad = textPad;
      if (strcmp(type, "num") == 0) {
        txt = " -0000+ ";
        extraPad = 24;
      }
      lv_txt_get_size(&sz, txt, btnFont, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
      if (strcmp(type, "error_status") == 0)
        extraPad = SCREEN_W - sz.x - textPad - 60;
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

        // register for status updates if has key
        if (strlen(key)) {
          statusWidgets[key] = lbl;
        }
        if (strcmp(it["status"] | "", "yes") == 0 && strlen(key))
          uiStatusRegisterButton(key, cell);

      }

      // ---------- ACTION / MENU ----------
      else if (strcmp(type, "action") == 0 || strcmp(type, "menu") == 0) {
        // Pick base color depending on type and label
        lv_color_t colorBtn = COLOR_BTN_ACTION;
        if (strcmp(type, "menu") == 0) colorBtn = COLOR_BTN_MENU;
        if (strcasecmp(txt, "back") == 0) colorBtn = COLOR_BTN_BACK;

        // --- Create the button ---
        lv_obj_t *btn = lv_btn_create(cont);
        lv_obj_remove_style_all(btn);  // clear any theme defaults
        lv_obj_set_size(btn, colW, 48);
        lv_obj_set_pos(btn, x, y);

        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_radius(btn, CORNERS, 0);
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_border_color(btn, colorBtn, 0);

        // mirror “active” / “issue” fill logic so color consistency is exact
        lv_obj_set_style_bg_color(btn, colorBtn, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, LV_PART_MAIN);

        // --- Label setup ---
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, txt);
        lv_obj_center(lbl);
        lv_obj_set_style_text_font(lbl, btnFont, 0);
        lv_obj_set_style_text_color(lbl, COLOR_BTN_TEXT, 0);

        // --- Register persistent status tracking if needed ---
        if (strcmp(it["status"] | "", "yes") == 0 && strlen(key))
          uiStatusRegisterButton(key, btn);

        // --- Skip empty keys entirely ---
        if (!key || !*key) {
          LOG_PRINTF("skipping button with empty key {%s}\n", txt);
          continue;
        }

        // --- Attach event callback directly without heap copies ---
        lv_obj_add_event_cb(
          btn,
          [](lv_event_t *e) {
            if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

            const char *keyUD = static_cast<const char *>(lv_event_get_user_data(e));
            if (!keyUD || !*keyUD) {
              LOG_PRINTF("click event: empty key\n");
              return;
            }

#if LV_USE_ASYNC_CALL
            // Safe async call, no free, key pointer is stable (JSON lives until menu rebuilt)
            lv_async_call(
              [](void *ud) {
                const char *k = static_cast<const char *>(ud);
                buttonAction(k);
              },
              (void *)keyUD);
#else
            // Direct call if async not enabled
            buttonAction(keyUD);
#endif
          },
          LV_EVENT_CLICKED, (void *)key);

      }

      // ---------- NUMERIC ----------
      else if (strcmp(type, "num") == 0) {

        lv_obj_t *numBox = lv_obj_create(cont);
        lv_obj_remove_style_all(numBox);
        lv_obj_set_size(numBox, colW - 2, 44);
        lv_obj_set_pos(numBox, x + 1, y + 2);
        lv_obj_set_flex_flow(numBox, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(numBox,
                              LV_FLEX_ALIGN_SPACE_BETWEEN,
                              LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);

        lv_obj_set_style_border_width(numBox, 2, 0);
        lv_obj_set_style_border_color(numBox, COLOR_BTN_NUM, 0);
        lv_obj_set_style_radius(numBox, CORNERS, 0);
        lv_obj_set_style_bg_opa(numBox, LV_OPA_TRANSP, 0);
        lv_obj_clear_flag(numBox, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(numBox, LV_OBJ_FLAG_SCROLL_CHAIN);
        lv_obj_add_flag(numBox, LV_OBJ_FLAG_CLICKABLE);

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

        lv_obj_t *lblVal = lv_label_create(numBox);
        char buf[8];
        int val = getParamValue(key);
        if (val >= 0) snprintf(buf, sizeof(buf), "%04d", val);
        else snprintf(buf, sizeof(buf), "-%03d", -val);
        lv_label_set_text(lblVal, buf);
        lv_obj_set_style_text_font(lblVal, FONT_BTN_SMALL_PTR, 0);
        lv_obj_set_style_text_color(lblVal, COLOR_BTN_TEXT, 0);
        lv_obj_set_style_text_align(lblVal, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_flag(lblVal, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(lblVal, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_clear_flag(lblVal, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(lblVal, LV_OBJ_FLAG_SCROLL_CHAIN);
        lv_obj_center(lblVal);

        // register numeric label by key for external updates
        if (strlen(key)) {
          numLabels[key] = lblVal;
        }

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

        lv_obj_set_style_pad_left(numBox, 4, 0);
        lv_obj_set_style_pad_right(numBox, 4, 0);
        lv_obj_set_style_pad_gap(numBox, 8, 0);
        lv_obj_set_style_translate_x(btnMinus, -4, 0);
        lv_obj_set_style_translate_x(btnPlus, 4, 0);

        lv_obj_add_event_cb(
          lblVal, [](lv_event_t *e) {
            if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
              lv_obj_t *val = (lv_obj_t *)lv_event_get_target(e);
              lv_obj_t *numBoxP = lv_obj_get_parent(val);
              select_num_pair(numBoxP, true);
            }
          },
          LV_EVENT_CLICKED, nullptr);

        lv_obj_add_event_cb(
          btnMinus, [](lv_event_t *e) {
            const char *keyUD = (const char *)lv_event_get_user_data(e);
            if (!keyUD || !*keyUD) return;
            select_num_pair(lv_obj_get_parent((lv_obj_t *)lv_event_get_target(e)), false);
            incrementParam((char *)keyUD, -1);
            int val = getParamValue(keyUD);
            char b[8];
            if (val >= 0) snprintf(b, sizeof(b), "%04d", val);
            else snprintf(b, sizeof(b), "-%03d", -val);
            lv_obj_t *lbl = lv_obj_get_child(lv_obj_get_parent((lv_obj_t *)lv_event_get_target(e)), 1);
            lv_label_set_text(lbl, b);
          },
          LV_EVENT_CLICKED, (void *)key);

        lv_obj_add_event_cb(
          btnPlus, [](lv_event_t *e) {
            const char *keyUD = (const char *)lv_event_get_user_data(e);
            if (!keyUD || !*keyUD) return;
            select_num_pair(lv_obj_get_parent((lv_obj_t *)lv_event_get_target(e)), false);
            incrementParam((char *)keyUD, +1);
            int val = getParamValue(keyUD);
            char b[8];
            if (val >= 0) snprintf(b, sizeof(b), "%04d", val);
            else snprintf(b, sizeof(b), "-%03d", -val);
            lv_obj_t *lbl = lv_obj_get_child(lv_obj_get_parent((lv_obj_t *)lv_event_get_target(e)), 1);
            lv_label_set_text(lbl, b);
          },
          LV_EVENT_CLICKED, (void *)key);
      }

      // ---------- ERROR STATUS ----------
      else if (strcmp(type, "error_status") == 0) {
        int textH = SCREEN_H - 250;
        if (textH < 60) textH = 60;

        lv_obj_t *ta = lv_textarea_create(cont);
        lv_obj_set_size(ta, colW, textH);
        lv_obj_set_pos(ta, x, y);

        lv_obj_set_style_height(ta, textH, 0);
        lv_obj_clear_flag(ta, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
        lv_obj_set_style_flex_grow(ta, 0, 0);

        lv_obj_set_scrollbar_mode(ta, LV_SCROLLBAR_MODE_AUTO);
        lv_obj_set_scroll_dir(ta, LV_DIR_VER);
        lv_obj_clear_flag(ta, LV_OBJ_FLAG_SCROLL_CHAIN);
        lv_obj_add_flag(ta, LV_OBJ_FLAG_SCROLL_ELASTIC);

        bool req_ok = rb.requestAllServoInfo();
        String servoText = "";
        if (req_ok) {
          servoText = String("#FFA500 servos info#\n") + rb.getAllServoInfoLines() + "\n\n";  //
        } else {
          servoText = String("#FFA500 servos info#\n") + String("⚠ no servos info") + "\n\n";  //
        }

        String systemText =
          String("#FFA500 build#\n") + getSketchVersionWithDate() + "\n\n" +  //
          String("#FFA500 log lines#\n") + rb.getAllErrorLines() + "\n\n" +   //
          servoText;                                                          //

        lv_textarea_set_text(ta, systemText.c_str());
        lv_textarea_set_cursor_click_pos(ta, false);
        lv_textarea_set_text_selection(ta, false);

        lv_obj_t *label = lv_textarea_get_label(ta);
        if (label) {
          lv_label_set_recolor(label, true);
          lv_obj_set_style_text_color(label, lv_color_white(), 0);
          lv_obj_set_style_text_opa(label, LV_OPA_COVER, 0);
          lv_obj_set_style_text_line_space(label, 4, 0);
        }

        lv_obj_set_style_bg_opa(ta, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(ta, 0, 0);
        lv_obj_set_style_text_font(ta, FONT_BTN_SMALL_PTR, 0);

        adjustedRowH = textH + 20;
      }
      x += colW + 8;
    }
    y += adjustedRowH;
  }

  onBuildMenu(menuName);
  lv_obj_invalidate(lv_scr_act());  // safe redraw scheduling
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
      [{ "text": "tests", "type": "menu", "key": "tests" }],
      [{ "text": "system", "type": "menu", "key": "system" }]
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
  "system": {
    "title": "system status",
    "footer": "scroll for details",
    "columns": 1,
    "equal_columns": "all",
    "rows": [
      [{ "type": "error_status", "text": "" }],
      [{ "text": "back", "type": "menu", "key": "main" }]
    ]
  },
  "tests": {
    "title": "tests",
    "footer": "perform hardware tests",
    "columns": 2,
    "equal_columns": "all",
    "rows": [
      [
        { "text": "poses", "type": "menu", "key": "poses" }
      ],
      [
        { "text": "sequences", "type": "menu", "key": "sequences" }
      ],
      [
        { "text": "cube moves", "type": "menu", "key": "cube_moves" }
      ],
      [{ "text": "back", "type": "menu", "key": "main" }]
    ]
  },
  "sequences": {
    "title": "sequences",
    "footer": "tap to execute motion sequence",
    "columns": 2,
    "equal_columns": "all",
    "rows": [
      [
        { "text": "bottom+", "type": "action", "key": "bottom_plus", "status": "yes" },
        { "text": "bottom-", "type": "action", "key": "bottom_minus", "status": "yes" }
      ],
      [
        { "text": "front to base", "type": "action", "key": "front_to_base", "status": "yes" },
        { "text": "back to base", "type": "action", "key": "back_to_base", "status": "yes" }
      ],
      [
        { "text": "left to base", "type": "action", "key": "left_to_base", "status": "yes" },
        { "text": "right to base", "type": "action", "key": "right_to_base", "status": "yes" }
      ],
      [
        { "text": "top to base", "type": "action", "key": "top_to_base", "status": "yes" },
        { "text": "", "type": "text" }
      ],
      [
        { "text": "rotate down face+", "type": "action", "key": "rotate_down_90", "status": "yes" },
        { "text": "rotate down face-", "type": "action", "key": "rotate_down_90minus", "status": "yes" }
      ],
      [{ "text": "back", "type": "menu", "key": "tests" }]
    ]
  },
  "cube_moves": {
    "title": "cube moves",
    "footer": "tap to run a move",
    "columns": 4,
    "equal_columns": "all",
    "rows": [
      [
        { "text": "f+", "type": "action", "key": "f_plus", "status": "yes" },
        { "text": "f-", "type": "action", "key": "f_minus", "status": "yes" },
        { "text": "b+", "type": "action", "key": "b_plus", "status": "yes" },
        { "text": "b-", "type": "action", "key": "b_minus", "status": "yes" }
      ],
      [
        { "text": "u+", "type": "action", "key": "u_plus", "status": "yes" },
        { "text": "u-", "type": "action", "key": "u_minus", "status": "yes" },
        { "text": "d+", "type": "action", "key": "d_plus", "status": "yes" },
        { "text": "d-", "type": "action", "key": "d_minus", "status": "yes" }
      ],
      [
        { "text": "l+", "type": "action", "key": "l_plus", "status": "yes" },
        { "text": "l-", "type": "action", "key": "l_minus", "status": "yes" },
        { "text": "r+", "type": "action", "key": "r_plus", "status": "yes" },
        { "text": "r-", "type": "action", "key": "r_minus", "status": "yes" }
      ],
      [
        { "text": "f++", "type": "action", "key": "f_plusplus", "status": "yes" },
        { "text": "b++", "type": "action", "key": "b_plusplus", "status": "yes" },
        { "text": "u++", "type": "action", "key": "u_plusplus", "status": "yes" },
        { "text": "d++", "type": "action", "key": "d_plusplus", "status": "yes" }
      ],
      [
        { "text": "l++", "type": "action", "key": "l_plusplus", "status": "yes" },
        { "text": "r++", "type": "action", "key": "r_plusplus", "status": "yes" },
        { "text": "", "type": "text" },
        { "text": "", "type": "text" }
      ],
      [{ "text": "back", "type": "menu", "key": "tests" }]
    ]
  },
  "poses": {
    "title": "poses",
    "footer": "tap to edit pose values",
    "columns": 3,
    "equal_columns": "all",
    "rows": [
      [
        { "text": "y zero", "type": "action", "key": "y_zero_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "y_zero_param" },
        { "text": "mm x10", "type": "text", "key": "" }
      ],
      [
        { "text": "y 1st", "type": "action", "key": "y_1st_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "y_1st_param" },
        { "text": "mm x10", "type": "text", "key": "" }
      ],
      [
        { "text": "y 2nd", "type": "action", "key": "y_2nd_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "y_2nd_param" },
        { "text": "mm x10", "type": "text", "key": "" }
      ],
      [
        { "text": "y 3rd", "type": "action", "key": "y_3rd_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "y_3rd_param" },
        { "text": "mm x10", "type": "text", "key": "" }
      ],
      [
        { "text": "x center", "type": "action", "key": "x_center_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "x_center_param" },
        { "text": "mm x10", "type": "text", "key": "" }
      ],
      [
        { "text": "x right", "type": "action", "key": "x_right_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "x_right_param" },
        { "text": "mm x10", "type": "text", "key": "" }
      ],
      [
        { "text": "x left", "type": "action", "key": "x_left_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "x_left_param" },
        { "text": "mm x10", "type": "text", "key": "" }
      ],
      [
        { "text": "y c2", "type": "action", "key": "y_c2_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "y_c2_param" },
        { "text": "mm x10", "type": "text", "key": "" }
      ],
      [
        { "text": "y c3", "type": "action", "key": "y_c3_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "y_c3_param" },
        { "text": "mm x10", "type": "text", "key": "" }
      ],
      [
        { "text": "wrist vert", "type": "action", "key": "wrist_vert_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "wrist_vert_param" },
        { "text": "deg", "type": "text", "key": "" }
      ],
      [
        { "text": "wrist h right", "type": "action", "key": "wrist_horiz_right_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "wrist_right_left_param" },
        { "text": "deg", "type": "text", "key": "" }
      ],
      [
        { "text": "wrist h left", "type": "action", "key": "wrist_horiz_left_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "wrist_horiz_left_param" },
        { "text": "deg", "type": "text", "key": "" }
      ],
            [
        { "text": "grippers open", "type": "action", "key": "grippers_open_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "grippers_open_param" },
        { "text": "per", "type": "text", "key": "" }
      ],
      [
        { "text": "gripper 1 open", "type": "action", "key": "gripper1_open_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "gripper1_open_param" },
        { "text": "per", "type": "text", "key": "" }
      ],
      [
        { "text": "gripper 1 close", "type": "action", "key": "gripper1_close_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "gripper1_close_param" },
        { "text": "per", "type": "text", "key": "" }
      ],
      [
        { "text": "gripper 2 open", "type": "action", "key": "gripper2_open_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "gripper2_open_param" },
        { "text": "per", "type": "text", "key": "" }
      ],
      [
        { "text": "gripper 2 close", "type": "action", "key": "gripper2_close_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "gripper2_close_param" },
        { "text": "per", "type": "text", "key": "" }
      ],
      [
        { "text": "base front", "type": "action", "key": "base_front_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "base_front_param" },
        { "text": "deg", "type": "text", "key": "" }
      ],
      [
        { "text": "base right", "type": "action", "key": "base_right_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "base_right_param" },
        { "text": "deg", "type": "text", "key": "" }
      ],
      [
        { "text": "base left", "type": "action", "key": "base_left_btn", "status": "yes" },
        { "text": "+0000-", "type": "num", "key": "base_left_param" },
        { "text": "deg", "type": "text", "key": "" }
      ],
      [{ "text": "back", "type": "menu", "key": "tests" }]
    ]
  }
}
)json";
