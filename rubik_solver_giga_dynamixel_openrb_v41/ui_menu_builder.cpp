#include <map>
#include <vector>
#include <algorithm>

#include "ui_touch.h"
#include "logging.h"
#include "ui_theme.h"
#include "ui_status.h"
#include "rb_interface.h"
#include "ui_button.h"

#define PARAM_VAL_NA 999.9

String getSketchVersion();
String getSketchVersionWithDate();
void buildMenu(const char *menuName);

extern RBInterface rb;

// ===========================================================
//  Static LVGL-compatible callbacks for numeric widgets
// ===========================================================
static void on_num_value_clicked(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  lv_obj_t *target = (lv_obj_t *)lv_event_get_target(e);
  lv_obj_t *parent = lv_obj_get_parent((const lv_obj_t *)target);

  // highlight/select this numeric pair
  select_num_pair(parent, true);
}

static void on_num_minus_clicked(lv_event_t *e) {
  const char *key = static_cast<const char *>(lv_event_get_user_data(e));
  if (!key || !*key) return;
  /*

  lv_obj_t *target = (lv_obj_t *)lv_event_get_target(e);
  lv_obj_t *parent = lv_obj_get_parent((const lv_obj_t *)target);

  select_num_pair(parent, false);
  increment_pose_param_in_pose_and_param_stores(key, -1);

  double val = PARAM_VAL_NA;
  pose_store.get_pose_params(key, &val);

  char buf[8];
  if (val >= 0) snprintf(buf, sizeof(buf), "+%.1f", val);
  else snprintf(buf, sizeof(buf), "-%.1f", -val);

  lv_obj_t *lbl = lv_obj_get_child(parent, 1);
  lv_label_set_text(lbl, buf);
  */
}

static void on_num_plus_clicked(lv_event_t *e) {
  const char *key = static_cast<const char *>(lv_event_get_user_data(e));
  if (!key || !*key) return;

  /*
  lv_obj_t *target = (lv_obj_t *)lv_event_get_target(e);
  lv_obj_t *parent = lv_obj_get_parent((const lv_obj_t *)target);

  select_num_pair(parent, false);
  increment_pose_param_in_pose_and_param_stores(key, +1);

  double val = PARAM_VAL_NA;
  pose_store.get_pose_params(key, &val);

  char buf[8];
  if (val >= 0) snprintf(buf, sizeof(buf), "+%.1f", val);
  else snprintf(buf, sizeof(buf), "-%.1f", -val);

  lv_obj_t *lbl = lv_obj_get_child(parent, 1);
  lv_label_set_text(lbl, buf);
  */
}

// ===========================================================
//  MAIN BUILD MENU FUNCTION
// ===========================================================
void buildMenu(const char *menuName) {

  LOG_PRINTF_MENU("---------------- build menu\n");
  LOG_SECTION_START_MENU("build menu {%s}", menuName);

  lv_obj_t *scr = lv_scr_act();
  if (!scr || !menuDoc.containsKey(menuName)) {
    LOG_SECTION_END_MENU();
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
    LOG_SECTION_END_MENU();
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

        if (strlen(key)) statusWidgets[key] = lbl;
      }

      // ---------- ACTION / MENU ----------
      else if (strcmp(type, "action") == 0 || strcmp(type, "menu") == 0) {
        lv_color_t colorBtn = COLOR_BTN_ACTION;
        if (strcmp(type, "menu") == 0) colorBtn = COLOR_BTN_MENU;
        if (strcasecmp(txt, "back") == 0) colorBtn = COLOR_BTN_BACK;

        lv_obj_t *btn = lv_btn_create(cont);
        lv_obj_remove_style_all(btn);
        lv_obj_set_size(btn, colW, 48);
        lv_obj_set_pos(btn, x, y);

        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_radius(btn, CORNERS, 0);
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_border_color(btn, colorBtn, 0);
        lv_obj_set_style_bg_color(btn, colorBtn, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, LV_PART_MAIN);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, txt);
        lv_obj_center(lbl);
        lv_obj_set_style_text_font(lbl, btnFont, 0);
        lv_obj_set_style_text_color(lbl, COLOR_BTN_TEXT, 0);

        if (strcmp(it["status"] | "", "yes") == 0 && strlen(key)) {
          updateButtonPtrByText(key, btn);
          // update overlay if it changed vs default build
          drawButtonOverlayByText(key);
        }

        // setup the click
        if (!key || !*key) {
          LOG_PRINTF_MENU("skipping button with empty key {%s}\n", key);
          continue;
        }

        // get the button id from the map
        UIButton *btn_ptr = find_button_by_key(key);
        int btn_id = 0;
        if (btn_ptr) btn_id = btn_ptr->get_id();

        lv_obj_add_event_cb(
          btn,
          [](lv_event_t *e) {
            if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

            // Retrieve button ID
            int id = (int)(intptr_t)lv_event_get_user_data(e);

#if LV_USE_ASYNC_CALL
            lv_async_call(
              [](void *param) {
                int bid = (int)(intptr_t)param;
                buttonAction(bid);
              },
              (void *)(intptr_t)id);
#else
            buttonAction(id);
#endif
          },
          LV_EVENT_CLICKED, (void *)(intptr_t)btn_id);
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
        lv_obj_add_flag(numBox, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_t *btnMinus = lv_btn_create(numBox);
        lv_obj_remove_style_all(btnMinus);
        lv_obj_set_size(btnMinus, 40, 40);
        lv_obj_set_style_radius(btnMinus, 20, 0);
        lv_obj_set_style_border_width(btnMinus, 2, 0);
        lv_obj_set_style_border_color(btnMinus, COLOR_BTN_NUM, 0);
        lv_obj_set_style_bg_opa(btnMinus, LV_OPA_TRANSP, 0);
        lv_obj_add_flag(btnMinus, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_style(btnMinus, &style_num_btn_pressed, LV_STATE_PRESSED);

        lv_obj_t *lblMinus = lv_label_create(btnMinus);
        lv_label_set_text(lblMinus, LV_SYMBOL_MINUS);
        lv_obj_set_style_text_font(lblMinus, FONT_BTN_SMALL_PTR, 0);
        lv_obj_set_style_text_color(lblMinus, COLOR_BTN_TEXT, 0);
        lv_obj_center(lblMinus);

        lv_obj_t *lblVal = lv_label_create(numBox);
        char buf[8];
        double val = PARAM_VAL_NA;

        //pose_store.get_pose_params(key, &val);
        //if (val >= 0) snprintf(buf, sizeof(buf), "+%.1f", val);
        //else snprintf(buf, sizeof(buf), "-%.1f", -val);

        lv_label_set_text(lblVal, buf);
        lv_obj_set_style_text_font(lblVal, FONT_BTN_SMALL_PTR, 0);
        lv_obj_set_style_text_color(lblVal, COLOR_BTN_TEXT, 0);
        lv_obj_set_style_text_align(lblVal, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_flag(lblVal, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_center(lblVal);

        if (strlen(key)) numLabels[key] = lblVal;

        lv_obj_t *btnPlus = lv_btn_create(numBox);
        lv_obj_remove_style_all(btnPlus);
        lv_obj_set_size(btnPlus, 40, 40);
        lv_obj_set_style_radius(btnPlus, 20, 0);
        lv_obj_set_style_border_width(btnPlus, 2, 0);
        lv_obj_set_style_border_color(btnPlus, COLOR_BTN_NUM, 0);
        lv_obj_set_style_bg_opa(btnPlus, LV_OPA_TRANSP, 0);
        lv_obj_add_flag(btnPlus, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_style(btnPlus, &style_num_btn_pressed, LV_STATE_PRESSED);

        lv_obj_t *lblPlus = lv_label_create(btnPlus);
        lv_label_set_text(lblPlus, LV_SYMBOL_PLUS);
        lv_obj_set_style_text_font(lblPlus, FONT_BTN_SMALL_PTR, 0);
        lv_obj_set_style_text_color(lblPlus, COLOR_BTN_TEXT, 0);
        lv_obj_center(lblPlus);

        lv_obj_set_style_pad_left(numBox, 4, 0);
        lv_obj_set_style_pad_right(numBox, 4, 0);
        lv_obj_set_style_pad_gap(numBox, 8, 0);

        // attach proper C-style callbacks
        lv_obj_add_event_cb(lblVal, on_num_value_clicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btnMinus, on_num_minus_clicked, LV_EVENT_CLICKED, (void *)key);
        lv_obj_add_event_cb(btnPlus, on_num_plus_clicked, LV_EVENT_CLICKED, (void *)key);
      }

      // ---------- ERROR STATUS ----------
      else if (strcmp(type, "error_status") == 0) {
        int textH = SCREEN_H - 250;
        if (textH < 60) textH = 60;

        lv_obj_t *ta = lv_textarea_create(cont);
        lv_obj_set_size(ta, colW, textH);
        lv_obj_set_pos(ta, x, y);
        lv_obj_set_scrollbar_mode(ta, LV_SCROLLBAR_MODE_AUTO);

        String servoText = "";
        servoText = String("#FFA500 servos info#\n") + "" + "\n\n";

        String systemText =
          String("#FFA500 build#\n") + getSketchVersionWithDate() + "\n\n" +  //
          String("#FFA500 log lines#\n") + getAllErrorLines() + "\n\n" + servoText;

        lv_textarea_set_text(ta, systemText.c_str());
        lv_textarea_set_cursor_click_pos(ta, false);
        lv_textarea_set_text_selection(ta, false);

        lv_obj_t *label = lv_textarea_get_label(ta);
        if (label) {
          lv_label_set_recolor(label, true);
          lv_obj_set_style_text_color(label, lv_color_white(), 0);
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

  lv_obj_invalidate(lv_scr_act());
  LOG_SECTION_END_MENU();
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
      [{ "text": "solve cube", "type": "menu", "key": "k_solve_cube" }],
      [{ "text": "read cube", "type": "menu", "key": "k_read_cube" }],
      [{ "text": "tests", "type": "menu", "key": "k_tests" }],
      [{ "text": "system", "type": "menu", "key": "k_system", "status": "yes" }]
    ]
  },
  "solve cube": {
    "title": "solve cube",
    "footer": "press to start solving",
    "columns": 4,
    "rows": [
        { "text": "read", "type": "action", "key": "k_solve_cube__read_cube_all", "status": "yes" },
        { "text": "solve", "type": "action", "key": "k_solve_cube_solve", "status": "yes" },
        { "text": "run", "type": "action", "key": "k_solve_cube_run", "status": "yes" },
        { "text": "back", "type": "menu", "key": "k_main" }
    ]
  },
  "read cube": {
    "title": "read cube",
    "footer": "read cube colors",
    "columns": 4,
    "rows": [
      [
        { "text": "all", "type": "action", "key": "k_read_cube_all","status": "yes" },
        { "text": "bottom", "type": "action", "key": "k_read_cube_bottom","status": "yes" },
        { "text": "centers", "type": "action", "key": "k_read_cube_centers","status": "yes" },
        { "text": "back", "type": "menu", "key": "k_main" }
      ]
    ]
  },
  "system": {
    "title": "system status",
    "footer": "scroll for details",
    "columns": 1,
    "equal_columns": "all",
    "rows": [
      [{ "type": "error_status", "text": "" }],
      [{ "text": "back", "type": "menu", "key": "k_main" }]
    ]
  },
  "tests": {
    "title": "tests",
    "footer": "perform hardware tests",
    "columns": 2,
    "equal_columns": "all",
    "rows": [
      [{ "text": "robot moves", "type": "menu", "key": "k_robot_moves" }],
      [{ "text": "cube moves", "type": "menu", "key": "k_cube_moves" }],
      [{ "text": "colors and orientation", "type": "menu", "key": "k_colors_and_orientation" }],
      [{ "text": "back", "type": "menu", "key": "k_main" }]
    ]
  },
  "robot moves": {
    "title": "robot moves",
    "footer": "tap to execute robot moves",
    "columns": 3,
    "equal_columns": "all",
    "rows": [
      [
        { "text": "front color", "type": "action", "key": "k_robot_move_front_color_text"  },
        { "text": "na", "type": "action", "key": "k_robot_move_front_color_value","status": "yes"  },
        { "text": "", "type": "text" }
      ],
      [
        { "text": "d+", "type": "action", "key": "k_robot_moves_d_plus","status": "yes" },
        { "text": "d-", "type": "action", "key": "k_robot_moves_d_minus","status": "yes"  },
        { "text": "d-", "type": "action", "key": "k_robot_moves_d2","status": "yes"  }
      ],
      [
        { "text": "z+", "type": "action", "key": "k_robot_moves_z_plus","status": "yes"  },
        { "text": "z-", "type": "action", "key": "k_robot_moves_z_minus","status": "yes"  },
        { "text": "z-", "type": "action", "key": "k_robot_moves_z2","status": "yes"  }
      ],
      [
        { "text": "y+", "type": "action", "key": "k_robot_moves_y_plus","status": "yes"  },
        { "text": "y-", "type": "action", "key": "k_robot_moves_y_minus","status": "yes"  },
        { "text": "y-", "type": "action", "key": "k_robot_moves_y2","status": "yes"  }
      ],
      [{ "text": "back", "type": "menu", "key": "k_tests" }]
    ]
  },
  "cube moves": {
    "title": "cube moves",
    "footer": "tap to run a cube move",
    "columns": 4,
    "equal_columns": "all",
    "rows": [
      [
        { "text": "front color", "type": "action", "key": "k_cube_moves_front_color_text"  },
        { "text": "na", "type": "action", "key": "k_cube_moves_front_color_value","status": "yes"  },
        { "text": "", "type": "text" },
        { "text": "", "type": "text" }
      ],
      [
        { "text": "f+", "type": "action", "key": "k_cube_moves_f_plus","status": "yes"  },
        { "text": "f-", "type": "action", "key": "k_cube_moves_f_minus","status": "yes"  },
        { "text": "b+", "type": "action", "key": "k_cube_moves_b_plus","status": "yes"  },
        { "text": "b-", "type": "action", "key": "k_cube_moves_b_minus","status": "yes"  }
      ],
      [
        { "text": "u+", "type": "action", "key": "k_cube_moves_u_plus","status": "yes"  },
        { "text": "u-", "type": "action", "key": "k_cube_moves_u_minus","status": "yes"  },
        { "text": "d+", "type": "action", "key": "k_cube_moves_d_plus","status": "yes"  },
        { "text": "d-", "type": "action", "key": "k_cube_moves_d_minus","status": "yes"  }
      ],
      [
        { "text": "l+", "type": "action", "key": "k_cube_moves_l_plus","status": "yes"  },
        { "text": "l-", "type": "action", "key": "k_cube_moves_l_minus","status": "yes"  },
        { "text": "r+", "type": "action", "key": "k_cube_moves_r_plus","status": "yes"  },
        { "text": "r-", "type": "action", "key": "k_cube_moves_r_minus","status": "yes"  }
      ],
      [
        { "text": "f++", "type": "action", "key": "k_cube_moves_f_plus_plus","status": "yes"  },
        { "text": "b++", "type": "action", "key": "k_cube_moves_b_plus_plus","status": "yes"  },
        { "text": "u++", "type": "action", "key": "k_cube_moves_u_plus_plus" ,"status": "yes" },
        { "text": "d++", "type": "action", "key": "k_cube_moves_d_plus_plus","status": "yes"  }
      ],
      [
        { "text": "l++", "type": "action", "key": "k_cube_moves_l_plus_plus","status": "yes"  },
        { "text": "r++", "type": "action", "key": "k_cube_moves_r_plus_plus","status": "yes"  },
        { "text": "", "type": "text" },
        { "text": "", "type": "text" }
      ],
      [{ "text": "back", "type": "menu", "key": "k_main" }]
    ]
  },
  "colors and orientation": {
    "title": "colors and orientation",
    "footer": "tap to execute colors and orientation commands",
    "columns": 3,
    "equal_columns": "all",
    "rows": [
      [
        { "text": "orientation", "type": "text", "key": "k_orientation_text","status": "yes"  },
        { "text": "", "type": "text", "key": "" },
        { "text": "", "type": "text", "key": "" }
      ],
      [
        { "text": "detect", "type": "action", "key": "k_orientation_detect","status": "yes"  },
        { "text": "check", "type": "action", "key": "k_orientation_check","status": "yes"  },
        { "text": "clear", "type": "action", "key": "k_orientation_clear","status": "yes"  }
      ],
      [
        { "text": "color read", "type": "text", "key": "k_orientation_color_read","status": "yes"  },
        { "text": "", "type": "text", "key": "" },
        { "text": "", "type": "text", "key": "" }
      ],
      [
        { "text": "all", "type": "action", "key": "k_orientation_color_read_all","status": "yes"  },
        { "text": "bottom", "type": "action", "key": "k_orientation_color_read_bottom","status": "yes"  },
        { "text": "sensor", "type": "action", "key": "k_orientation_color_read_sensor","status": "yes"  }
      ],
      [
        { "text": "color stickers", "type": "text", "key": "k_orientation_color_stickers","status": "yes"  },
        { "text": "", "type": "text", "key": "" },
        { "text": "", "type": "text", "key": "" }
      ],
      [
        { "text": "c1", "type": "action", "key": "k_color_c1","status": "yes"  },
        { "text": "c2", "type": "action", "key": "k_color_c2","status": "yes"  },
        { "text": "c3", "type": "action", "key": "k_color_c3","status": "yes"  }
      ],
      [
        { "text": "c4", "type": "action", "key": "k_color_c4","status": "yes"  },
        { "text": "c5", "type": "action", "key": "k_color_c5","status": "yes"  },
        { "text": "c6", "type": "action", "key": "k_color_c6","status": "yes"  }
      ],
      [
        { "text": "rotate", "type": "text", "key": "k_color_rotate" },
        { "text": "", "type": "text", "key": "" },
        { "text": "", "type": "text", "key": "" }
      ],
      [
        { "text": "y+", "type": "action", "key": "k_color_y_plus","status": "yes"  },
        { "text": "z+", "type": "action", "key": "k_color_z_plus","status": "yes"  },
        { "text": "", "type": "text", "key": "" }
      ],
      [{ "text": "back", "type": "menu", "key": "k_tests" }]
    ]
  }
}
)json";
