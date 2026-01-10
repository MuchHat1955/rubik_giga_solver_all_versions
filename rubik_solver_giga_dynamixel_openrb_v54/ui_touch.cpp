#include <Arduino_GigaDisplayTouch.h>
#include <map>
#include <vector>
#include <algorithm>

#include "ui_touch.h"
#include "logging.h"
#include "Arduino_H7_Video.h"
#include "ui_theme.h"
#include "ui_status.h"
#include "rb_interface.h"
#include "ui_button.h"
#include "run.h"

// Forward declarations
void drawButtonOverlay(int btn_id);

extern RBInterface rb;

// ----------------------------------------------------------
//                   LVGL GLOBALS
// ----------------------------------------------------------
StaticJsonDocument<12288> menuDoc;

extern lv_obj_t *footLbl_old;  // DO NOT USE
extern std::map<String, lv_obj_t *> numLabels;

extern lv_obj_t *selected_num_box;
extern lv_style_t style_num_selected;
extern lv_style_t style_num_btn_active;
extern lv_style_t style_num_btn_pressed;

// ----------------------------------------------------------
//                   GLOBAL UI STATE MAPS
// ----------------------------------------------------------
std::map<String, lv_obj_t *> statusWidgets;
std::map<String, lv_obj_t *> numLabels;

// ----------------------------------------------------------
//                SERVO BUTTON MAPPING & STATES
// ----------------------------------------------------------

struct ServoButtonInfo {
  const char *key;         // logical key (e.g. "arm1_status")
  lv_obj_t *btn;           // pointer to LVGL button
  bool hasError{ false };  // true = display diagonal error line
  bool atPos{ false };     // true = servo at target pose
};

static std::map<String, ServoButtonInfo> stateButtons;  // key → info

// ----------------------------------------------------------
//                   UTILITY HELPERS
// ----------------------------------------------------------
void setFooter_v1(const char *msg) {
  if (footLbl_old && msg) {
    char b[200];
    char *l = b;

    // Convert to lowercase before printing
    for (const char *p = msg; *p && (l - b) < (int)sizeof(b) - 1; ++p) {
      *l++ = tolower((unsigned char)*p);
    }
    *l = '\0';  // ✅ null-terminate correctly

    LOG_PRINTF_MENU("set footer {%s}\n", b);

    lv_label_set_text(footLbl_old, b);

    // force a visible redraw
    lv_obj_invalidate(footLbl_old);
    lv_refr_now(NULL);
    lv_timer_handler();
    lv_obj_invalidate(footLbl_old);
    lv_refr_now(NULL);
    __delay(15);

    LOG_PRINTF_MENU("calling lv timer handler 6 times\n");
    for (int i = 0; i < 6; ++i) {
      lv_timer_handler();
      delay(5);
    }
    __delay(15);
  }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~ SET FOOTER ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/* ----------------------------------------------------------
 * External helpers you already have
 * ---------------------------------------------------------- */
extern void ui_dim_overlay_show(bool show);

/* ----------------------------------------------------------
 * Internal objects
 * ---------------------------------------------------------- */
static lv_obj_t *foot_cont = NULL;
static lv_obj_t *foot_icon_lbl = NULL;
static lv_obj_t *foot_lbl = NULL;
static lv_obj_t *foot_stop_btn = NULL;

/*
LV_SYMBOL_OK        // ✔
LV_SYMBOL_CLOSE     // ✖
LV_SYMBOL_WARNING   // ⚠
LV_SYMBOL_INFO      // ℹ
LV_SYMBOL_REFRESH   // 🔄
LV_SYMBOL_PLAY      // ▶
LV_SYMBOL_STOP      // ■
LV_SYMBOL_PAUSE     // ❚❚
LV_SYMBOL_CHARGE    // ⚡
*/

/* ----------------------------------------------------------
 * STOP callback (placeholder)
 * ---------------------------------------------------------- */
static void footer_stop_cb(lv_event_t *e) {
  LV_UNUSED(e);

  LOG_PRINTF_MENU("footer stop button pressed\n");

  setFooter("sending... stop command", _RUNNING_NOSTOP);
  bool ok = rb.send_stop_command();

  if (ok) setFooter("stop command received", _DONE_SUCCESS);
  else setFooter("stop command not received", _DONE_ERROR);

  for (int i = 0; i < 111; i++) { delay(3); }
}

static inline void obj_set_hidden(lv_obj_t *obj, bool hide) {
  if (!obj) return;
  if (hide) lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
  else lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

/* ----------------------------------------------------------
 * Create footer (call from menu builder)
 * ---------------------------------------------------------- */
void createFooter(lv_obj_t *parent) {
  foot_cont = lv_obj_create(parent);
  lv_obj_remove_style_all(foot_cont);
  lv_obj_set_size(foot_cont, LV_PCT(100), 28);
  lv_obj_align(foot_cont, LV_ALIGN_BOTTOM_MID, 0, -15);

  lv_obj_set_style_pad_left(foot_cont, 8, 0);
  lv_obj_set_style_pad_right(foot_cont, 8, 0);
  lv_obj_set_style_pad_top(foot_cont, 4, 0);
  lv_obj_set_style_pad_bottom(foot_cont, 4, 0);
  lv_obj_set_style_pad_gap(foot_cont, 6, 0);

  lv_obj_set_flex_flow(foot_cont, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(foot_cont,
                        LV_FLEX_ALIGN_START,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  /* left spacer */
  lv_obj_t *left_spacer = lv_obj_create(foot_cont);
  lv_obj_remove_style_all(left_spacer);
  lv_obj_set_flex_grow(left_spacer, 1);

  /* center group */
  lv_obj_t *center_cont = lv_obj_create(foot_cont);
  lv_obj_remove_style_all(center_cont);
  lv_obj_set_flex_flow(center_cont, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(center_cont,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_gap(center_cont, 6, 0);

  /* IMPORTANT: limit center width */
  lv_obj_set_width(center_cont, LV_PCT(60));

  /* icon */
  foot_icon_lbl = lv_label_create(center_cont);
  lv_label_set_text(foot_icon_lbl, "");
  lv_obj_add_flag(foot_icon_lbl, LV_OBJ_FLAG_HIDDEN);

  /* make icon slightly larger than text */
  lv_obj_set_style_text_font(foot_icon_lbl, FONT_BTN_LARGE_PTR, 0);

  /* text */
  foot_lbl = lv_label_create(center_cont);
  lv_label_set_text(foot_lbl, "");
  lv_obj_set_style_text_align(foot_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(foot_lbl, FONT_FOOT_PTR, 0);

  /* right spacer */
  lv_obj_t *right_spacer = lv_obj_create(foot_cont);
  lv_obj_remove_style_all(right_spacer);
  lv_obj_set_flex_grow(right_spacer, 1);

  /* stop button */
  foot_stop_btn = lv_btn_create(foot_cont);
  lv_obj_set_size(foot_stop_btn, 26, 26);
  lv_obj_add_event_cb(foot_stop_btn, footer_stop_cb,
                      LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_opa(foot_stop_btn, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(foot_stop_btn, 2, 0);
  lv_obj_set_style_border_color(
    foot_stop_btn,
    lv_palette_main(LV_PALETTE_ORANGE), 0);
  lv_obj_set_style_radius(foot_stop_btn, 4, 0);

  lv_obj_t *lbl = lv_label_create(foot_stop_btn);
  lv_label_set_text(lbl, LV_SYMBOL_STOP);
  lv_obj_center(lbl);

  obj_set_hidden(foot_stop_btn, true);
}

/* ----------------------------------------------------------
 * Internal object
 * ---------------------------------------------------------- */
static lv_obj_t *ui_dim_overlay = NULL;

/* ----------------------------------------------------------
 * Create overlay (call once per screen)
 * ---------------------------------------------------------- */
void ui_dim_overlay_create(lv_obj_t *parent) {
  if (ui_dim_overlay && lv_obj_is_valid(ui_dim_overlay)) return;

  ui_dim_overlay = NULL;  // stale pointer protection

  ui_dim_overlay = lv_obj_create(parent);
  lv_obj_remove_style_all(ui_dim_overlay);

  lv_obj_set_size(ui_dim_overlay, LV_PCT(100), LV_PCT(100));
  lv_obj_set_pos(ui_dim_overlay, 0, 0);

  lv_obj_set_style_bg_color(ui_dim_overlay, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(ui_dim_overlay, LV_OPA_50, 0);

  lv_obj_add_flag(ui_dim_overlay, LV_OBJ_FLAG_CLICKABLE);
  obj_set_hidden(ui_dim_overlay, true);
}

/* ----------------------------------------------------------
 * Show / hide overlay
 * ---------------------------------------------------------- */
void ui_dim_overlay_show(bool show) {
  if (!ui_dim_overlay || !lv_obj_is_valid(ui_dim_overlay)) return;

  if (show) {
    lv_obj_clear_flag(ui_dim_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(ui_dim_overlay);
  } else {
    lv_obj_add_flag(ui_dim_overlay, LV_OBJ_FLAG_HIDDEN);
  }
}

/* ----------------------------------------------------------
 * Set footer (ONE semantic param)
 * ---------------------------------------------------------- */
void setFooter(const char *msg, footer_state_t state) {
  if (!foot_lbl || !msg) return;

  char b[200];
  char *l = b;

  /* lowercase conversion */
  for (const char *p = msg;
       *p && (l - b) < (int)sizeof(b) - 1;
       ++p) {
    *l++ = tolower((unsigned char)*p);
  }
  *l = '\0';

  // LOG_PRINTF_MENU("set footer start text{%s} state{%d} \n", msg, state);

  /* defaults */
  const char *icon = "";
  lv_color_t icon_color = lv_color_white();
  bool blocking = false;
  bool show_stop = false;

  switch (state) {
    case _INFO:
      icon = LV_SYMBOL_OK;
      icon_color = lv_palette_main(LV_PALETTE_BLUE);
      break;

    case _ERROR:
      icon = LV_SYMBOL_WARNING;
      icon_color = lv_palette_main(LV_PALETTE_RED);
      blocking = false;
      break;

    case _RUNNING_STOP:
      icon = LV_SYMBOL_REFRESH;
      icon_color = lv_palette_main(LV_PALETTE_YELLOW);
      blocking = true;
      show_stop = true;
      break;

    case _RUNNING_NOSTOP:
      icon = LV_SYMBOL_REFRESH;
      icon_color = lv_palette_main(LV_PALETTE_YELLOW);
      blocking = true;
      show_stop = false;
      break;

    case _DONE_SUCCESS:
      icon = LV_SYMBOL_OK;
      icon_color = lv_palette_main(LV_PALETTE_GREEN);
      break;

    case _DONE_ERROR:
      icon = LV_SYMBOL_CLOSE;
      icon_color = lv_palette_main(LV_PALETTE_RED);
      break;
  }

  /* icon */
  lv_label_set_text(foot_icon_lbl, icon);
  lv_obj_set_style_text_color(foot_icon_lbl, icon_color, 0);
  obj_set_hidden(foot_icon_lbl, icon[0] == '\0');

  /* text */
  lv_label_set_text(foot_lbl, b);

  /* STOP button */
  obj_set_hidden(foot_stop_btn, !show_stop);

  /* dim / block UI */
  ui_dim_overlay_show(blocking);

  // LOG_PRINTF_MENU("set footer end text{%s} state{%d} blocking{%d} show_stop{%d} \n",  //
  //                 msg, state, blocking, show_stop);

  if (foot_cont) {
    lv_obj_invalidate(foot_cont);
    lv_refr_now(NULL);
    lv_timer_handler();
    delay(5);

    lv_obj_invalidate(foot_cont);
    lv_refr_now(NULL);
    delay(5);

    for (int i = 0; i < 6; ++i) {
      lv_timer_handler();
      delay(5);
    }
  }
  __delay(15);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ---- below is for actions to be done when a menu is displayed ---
void buttonAction_executeAction(int btn_id) {
  LOG_SECTION_START_MENU("update buttons on click for menu {%d}", btn_id);
  UIButton *b = find_button_by_id(btn_id);
  if (!b) {
    LOG_PRINTF_MENU("[!] buttonAction_executeAction: no button found for id {%d}\n", btn_id);
    return;
  }

  const char *btn_key = b->get_key();
  const char *btn_txt = b->get_text();

  // haddle all status buttons
  if (b->get_is_status() && !b->get_is_menu()) {
    LOG_SECTION_START_MENU("update overlay for btn {%s}", btn_key);
    // show busy
    b->set_is_busy(true);
    drawButtonOverlayById(btn_id);
    //.................
    buttonAction_run(btn_key);
    //.................
    // reset busy
    b->set_is_busy(false);
    drawButtonOverlayById(btn_id);
    LOG_SECTION_END_MENU();
  }
  LOG_SECTION_END_MENU();
}

// ----------------------------------------------------------
//                   BUTTON ACTION LOGIC
// ----------------------------------------------------------
void buttonAction(int btn_id) {
  LOG_SECTION_START_MENU("buttonAction: id {%d}", btn_id);

  UIButton *b = find_button_by_id(btn_id);
  if (!b) {
    LOG_PRINTF_MENU("[!] buttonAction: no button found for id {%d}\n", btn_id);
    return;
  }

  static unsigned long lastClickTime = 0;
  static String lastClickKey;
  static unsigned long millisButtonBusy = 0;
  static String lastBusyKey = "";

  const char *key = b->get_key();
  unsigned long now = millis();

  // --- Prevent rapid re-clicks (debounce 500 ms)
  if (key && lastClickKey.equals(key) && now < lastClickTime + 500) {
    LOG_PRINTF_MENU("[!] rapid re-click ignored key {%s}\n", key);
    LOG_SECTION_END_MENU();
    return;
  }
  lastClickTime = now;
  lastClickKey = key;

  // --- Guard against null or empty key
  if (!key || !*key) {
    LOG_PRINTF_MENU("[!] button action empty or null key ignored\n");

    return;
  }

  /*
  
  // --- Busy guard (wait up to 33 s)
  if (millisButtonBusy > 0 && now < millisButtonBusy + 33000) {
    LOG_PRINTF_MENU("[!] button action ignored while busy on {%s}\n",
                    lastBusyKey.c_str());
    LOG_SECTION_END_MENU();
    return;
  }
  */

  // --- Mark this button as busy before running the long op
  millisButtonBusy = now;
  lastBusyKey = key;

  LOG_PRINTF_MENU("start action for button id {%d} key {%s}\n", btn_id, key);
  buttonAction_executeAction(btn_id);  // blocking call
  LOG_SECTION_END_MENU();

  millisButtonBusy = 0;

  // --- Navigate to submenu only after operation finished
  if (b->get_is_menu()) {
    currentMenu = key;
    // setFooter((String("switch menu to ") + key).c_str());
    buildMenu(currentMenu.c_str());
    LOG_SECTION_END_MENU();
    return;
  }

  // --- Explicit back to main menu
  if (strcmp(key, "k_main") == 0) {
    currentMenu = "k_main";
    // setFooter("back to main menu");
    buildMenu(currentMenu.c_str());
    LOG_SECTION_END_MENU();
    return;
  }

  LOG_SECTION_END_MENU();
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

void select_num_pair(lv_obj_t *numBox, bool toggle) {
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
//                   UI INITIALIZATION
// ----------------------------------------------------------
void validateMenuKeys() {
  for (JsonPair kv : menuDoc.as<JsonObject>()) {
    const char *menuKey = kv.key().c_str();
    JsonObject menu = kv.value().as<JsonObject>();
    JsonArray rows = menu["rows"].as<JsonArray>();
    for (JsonArray row : rows) {
      for (JsonObject it : row) {
        const char *type = it["type"] | "";
        const char *key = it["key"] | "";
        if ((strcmp(type, "action") == 0 || strcmp(type, "k_menu") == 0) && (!key || !*key)) {
          LOG_PRINTF_MENU("menu missing key {%s} type {%s}\n", menuKey, type);
        }
      }
    }
  }
}

extern const char jsonBuffer[];

void ui_init() {
  LOG_SECTION_START_MENU("ui_init");

  // 1. Initialize LVGL core
  lv_init();

  // 2. Initialize display and touch
  static Arduino_H7_Video Display(SCREEN_W, SCREEN_H, GigaDisplayShield);
  static Arduino_GigaDisplayTouch Touch;
  Display.begin();
  Touch.begin();

  // 3. Parse menu JSON AFTER initializing LVGL
  DeserializationError err = deserializeJson(menuDoc, jsonBuffer);
  if (err) {
    LOG_PRINTF_MENU("error on deserialize json {%s}\n", err.c_str());
    LOG_SECTION_END_MENU();
    return;  // avoid using empty doc
  }

  validateMenuKeys();

  // 4. Build the initial main menu screen
  buildMenu("k_main");

  LOG_SECTION_END_MENU();
}

// ----------------------------------------------------------
//                   UI LOOP AND REFRESH
// ----------------------------------------------------------
static unsigned long lastRefresh = 0;

void ui_refresh() {
  unsigned long now = millis();
  if (now - lastRefresh < 30000UL) return;  //ADJUST-> adjust this
  lastRefresh = now;

  // TODO-> TO CHECK if this is still needed
}

void ui_loop() {
  lv_timer_handler();
  ui_refresh();
}
