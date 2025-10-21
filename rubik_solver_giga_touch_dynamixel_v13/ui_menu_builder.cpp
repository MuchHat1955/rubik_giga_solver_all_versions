#include "ui_touch.h"
#include "logging.h"
#include "ui_theme.h"
#include "ui_touch.h"
#include "param_store.h"
#include <map>
#include <vector>
#include <algorithm>
#include "ui_status.h"
#include "servo_manager.h"    // for servoMgr

String getSketchVersion();
String getSketchVersionWithDate();
void buildMenu(const char *menuName);
extern ServoManager servoMgr;

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

  int available_h = SCREEN_H - (title_h + footer_h + 60);
  if (available_h < 60) available_h = 60;

  // snap container height to full rows
  const int rowH = 65;
  int contH = available_h - (available_h % rowH);
  if (contH < rowH) contH = rowH;

  lv_obj_set_size(cont, SCREEN_W, contH);
  lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, title_h + 20);

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

        // ✅ register servo name rows for persistent status
        if (strcmp(it["status"] | "", "yes") == 0)
          uiStatusRegisterButton(txt, cell);
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

        // ✅ Register button for persistent status tracking
        if (strcmp(it["status"] | "", "yes") == 0 && strlen(key))
          uiStatusRegisterButton(key, btn);

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
        // (your numeric code block here unchanged)
      }

      // ---------- ERROR STATUS ----------
      else if (strcmp(type, "error_status") == 0) {
        lv_obj_t *ta = lv_textarea_create(cont);
        lv_obj_set_size(ta, colW, lv_obj_get_height(cont) - 10);
        lv_obj_set_pos(ta, x, y);
        lv_textarea_set_text(ta,
                             (String("build ") + getSketchVersionWithDate() + "\n\n" +  //
                              servoMgr.getStartupTestErrorString() + "\n\n" +           //
                              servoMgr.getFullDiagnosticString())
                               .c_str());
        lv_textarea_set_cursor_click_pos(ta, false);
        lv_obj_set_scrollbar_mode(ta, LV_SCROLLBAR_MODE_AUTO);
        lv_obj_set_style_text_font(ta, FONT_BTN_SMALL_PTR, 0);
        lv_obj_set_style_bg_opa(ta, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(ta, 0, 0);
      }

      x += colW + 8;
    }
    y += adjustedRowH;
  }

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
    "rows": [
      [{ "type": "error_status", "text": "" }],
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
      [{ "text": "back", "type": "menu", "key": "main" }]
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
        { "text": "arm1", "type": "text", "status": "yes" },
        { "text": " -0000+ ", "type": "num", "key": "arm1_0" },
        { "text": " -0000+ ", "type": "num", "key": "arm1_min" },
        { "text": " -0000 +", "type": "num", "key": "arm1_max" }
      ],
      [
        { "text": "arm2", "type": "text", "status": "yes" },
        { "text": "-0000+", "type": "num", "key": "arm2_0" },
        { "text": "-0000+", "type": "num", "key": "arm2_min" },
        { "text": "-0000+", "type": "num", "key": "arm2_max" }
      ],
      [
        { "text": "wrist", "type": "text", "status": "yes" },
        { "text": "-0000+", "type": "num", "key": "wrist_0" },
        { "text": "-0000+", "type": "num", "key": "wrist_min" },
        { "text": "-0000+", "type": "num", "key": "wrist_max" }
      ],
      [
        { "text": "grip l", "type": "text", "status": "yes" },
        { "text": "-0000+", "type": "num", "key": "grip1_0" },
        { "text": "-0000+", "type": "num", "key": "grip1_min" },
        { "text": "-0000+", "type": "num", "key": "grip1_max" }
      ],
      [
        { "text": "grip r", "type": "text", "status": "yes" },
        { "text": "-0000+", "type": "num", "key": "grip2_0" },
        { "text": "-0000+", "type": "num", "key": "grip2_min" },
        { "text": "-0000+", "type": "num", "key": "grip2_max" }
      ],
      [
        { "text": "base", "type": "text", "status": "yes" },
        { "text": "-0000+", "type": "num", "key": "base_0" },
        { "text": "-0000+", "type": "num", "key": "base_min" },
        { "text": "-0000+", "type": "num", "key": "base_max" }
      ],
      [{ "text": "back", "type": "menu", "key": "tests" }]
    ]
  },
  "servos_programming": {
    "title": "tests",
    "footer": "set servo id and reset to mid range",
    "columns": 4,
    "equal columns": "last",
    "rows": [
      [
        { "text": "arm1 (id11)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "arm1_program", "status": "yes" },
        { "text": "test", "type": "action", "key": "arm1_test", "status": "yes" },
        { "text": "status", "type": "text", "key": "arm1_status" }
      ],
      [
        { "text": "arm2 (id12)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "arm2_program", "status": "yes" },
        { "text": "test", "type": "action", "key": "arm2_test", "status": "yes" },
        { "text": "status", "type": "text", "key": "arm2_status" }
      ],
      [
        { "text": "wrist (id13)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "wrist_program", "status": "yes" },
        { "text": "test", "type": "action", "key": "wrist_test", "status": "yes" },
        { "text": "status", "type": "text", "key": "wrist_status" }
      ],
      [
        { "text": "grip l (id14)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "grip1_program", "status": "yes" },
        { "text": "test", "type": "action", "key": "grip1_test", "status": "yes" },
        { "text": "status", "type": "text", "key": "grip1_status" }
      ],
      [
        { "text": "grip r (id15)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "grip2_program", "status": "yes" },
        { "text": "test", "type": "action", "key": "grip2_test", "status": "yes" },
        { "text": "status", "type": "text", "key": "grip2_status" }
      ],
      [
        { "text": "base (id16)", "type": "text", "key": "blank" },
        { "text": "program", "type": "action", "key": "base_program", "status": "yes" },
        { "text": "test", "type": "action", "key": "base_test", "status": "yes" },
        { "text": "status", "type": "text", "key": "base_status" }
      ],
      [{ "text": "back", "type": "menu", "key": "tests" }]
    ]
  },
  "vertical_tune": {
  "title": "tests",
  "footer": "adjust for vertical move arm1, arm2, wrist",
  "columns": 4,
  "equal columns": "last",
  "rows": [
    [
      { "text": "mm", "type": "text", "key": "grip_open" },
      { "text": "arm1", "type": "text" },
      { "text": "arm2", "type": "text", "key": "grip_close" },
      { "text": "wrist", "type": "text" }
    ],
    [
      { "text": "0mm", "type": "action", "key": "vertical_0mm", "status": "yes" },
      { "text": "arm1", "type": "num", "key": "arm1_0mm" },
      { "text": "arm2", "type": "num", "key": "arm2_0mm" },
      { "text": "wrist", "type": "num", "key": "wrist_0mm" }
    ],
    [
      { "text": "5mm", "type": "action", "key": "vertical_5mm", "status": "yes" },
      { "text": "arm1", "type": "num", "key": "arm1_5mm" },
      { "text": "arm2", "type": "num", "key": "arm2_5mm" },
      { "text": "wrist", "type": "num", "key": "wrist_5mm" }
    ],
    [
      { "text": "10mm", "type": "action", "key": "vertical_10mm", "status": "yes" },
      { "text": "arm1", "type": "num", "key": "arm1_10mm" },
      { "text": "arm2", "type": "num", "key": "arm2_10mm" },
      { "text": "wrist", "type": "num", "key": "wrist_10mm" }
    ],
    [
      { "text": "15mm", "type": "action", "key": "vertical_15mm", "status": "yes" },
      { "text": "arm1", "type": "num", "key": "arm1_15mm" },
      { "text": "arm2", "type": "num", "key": "arm2_15mm" },
      { "text": "wrist", "type": "num", "key": "wrist_15mm" }
    ],
    [
      { "text": "", "type": "text" },
      { "text": "g open", "type": "action", "key": "grip_open", "status": "yes" },
      { "text": "g close", "type": "action", "key": "grip_close", "status": "yes" },
      { "text": "", "type": "text" }
    ],
    [
      { "text": "back", "type": "menu", "key": "tests" },
      { "text": "", "type": "text", "key": "blank" },
      { "text": "", "type": "text", "key": "blank" },
      { "text": "", "type": "text", "key": "blank" }
    ]
  ]
  },
  "poses": {
    "title": "servo poses",
    "footer": "tap to edit pose values",
    "columns": 3,
    "equal columns": "all",
    "rows": [
      [{ "text": "arm1 0", "type": "action", "key": "arm1_0", "status": "yes" },
       { "text": "+0000-", "type": "num", "key": "arm1_pose_0" },
       { "text": "current na", "type": "text", "key": "arm1_current" }],

      [{ "text": "arm2 0", "type": "action", "key": "arm2_0", "status": "yes" },
       { "text": "+0000-", "type": "num", "key": "arm2_pose_0" },
       { "text": "current na", "type": "text", "key": "arm2_current" }],

      [{ "text": "wrist 0", "type": "action", "key": "wrist_0", "status": "yes" },
       { "text": "+0000-", "type": "num", "key": "wrist_pose_0" },
       { "text": "current na", "type": "text", "key": "wrist_current" }],

      [{ "text": "wrist 90", "type": "action", "key": "wrist_90", "status": "yes" },
       { "text": "+0000-", "type": "num", "key": "wrist_pose_90" },
       { "text": "current na", "type": "text", "key": "wrist_current" }],

      [{ "text": "wrist -90", "type": "action", "key": "wrist_minus90", "status": "yes" },
       { "text": "+0000-", "type": "num", "key": "wrist_pose_minus90" },
       { "text": "current na", "type": "text", "key": "wrist_current" }],

      [{ "text": "grip1 Open", "type": "action", "key": "grip1_open", "status": "yes" },
       { "text": "+0000-", "type": "num", "key": "grip1_pose_0" },
       { "text": "current na", "type": "text", "key": "grip1_current" }],

      [{ "text": "grip1 Close", "type": "action", "key": "grip1_close", "status": "yes" },
       { "text": "+0000-", "type": "num", "key": "grip1_pose_1" },
       { "text": "current na", "type": "text", "key": "grip1_current" }],

      [{ "text": "grip2 Open", "type": "action", "key": "grip2_open", "status": "yes" },
       { "text": "+0000-", "type": "num", "key": "grip2_pose_0" },
       { "text": "current na", "type": "text", "key": "grip2_current" }],

      [{ "text": "grip2 Close", "type": "action", "key": "grip2_close", "status": "yes" },
       { "text": "+0000-", "type": "num", "key": "grip2_pose_1" },
       { "text": "current na", "type": "text", "key": "grip2_current" }],

      [{ "text": "base 0", "type": "action", "key": "base_0", "status": "yes" },
       { "text": "+0000-", "type": "num", "key": "base_pose_0" },
       { "text": "current na", "type": "text", "key": "base_current" }],

      [{ "text": "base 90", "type": "action", "key": "base_90", "status": "yes" },
       { "text": "+0000-", "type": "num", "key": "base_pose_90" },
       { "text": "current na", "type": "text", "key": "base_current" }],

      [{ "text": "base -90", "type": "action", "key": "base_minus90", "status": "yes" },
       { "text": "+0000-", "type": "num", "key": "base_pose_minus90" },
       { "text": "current na", "type": "text", "key": "base_current" }],

      [{ "text": "back", "type": "menu", "key": "tests" }]
    ]
  }
}
)json";
