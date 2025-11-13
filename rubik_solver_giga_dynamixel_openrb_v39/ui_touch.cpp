#include <Arduino_GigaDisplayTouch.h>
#include <map>
#include <vector>
#include <algorithm>

#include "ui_touch.h"
#include "logging.h"
#include "param_store.h"
#include "Arduino_H7_Video.h"
#include "ui_theme.h"
#include "ui_status.h"
#include "rb_interface.h"
#include "pose_store.h"
#include "ui_button.h"

// Forward declarations
void drawButtonOverlay(int btn_id);

extern RBInterface rb;
extern PoseStore pose_store;

// ----------------------------------------------------------
//                   LVGL GLOBALS
// ----------------------------------------------------------
StaticJsonDocument<12288> menuDoc;

extern lv_obj_t *footLbl;
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
void setFooter(const char *msg) {
  if (footLbl && msg) {
    char b[200];
    char *l = b;

    // Convert to lowercase before printing
    for (const char *p = msg; *p && (l - b) < (int)sizeof(b) - 1; ++p) {
      *l++ = tolower((unsigned char)*p);
    }
    *l = '\0';  // ✅ null-terminate correctly

    LOG_PRINTF_MENU("set footer {%s}\n", b);

    lv_label_set_text(footLbl, b);

    // force a visible redraw
    lv_obj_invalidate(footLbl);
    lv_refr_now(NULL);
    lv_timer_handler();
    lv_obj_invalidate(footLbl);
    lv_refr_now(NULL);
    delay(15);

    LOG_PRINTF_MENU("calling lv timer handler 6 times\n");
    for (int i = 0; i < 6; ++i) {
      lv_timer_handler();
      delay(5);
    }
    delay(15);
  }
}

// ---- below is for actions to be done when a menu is displayed ---
void buttonAction_executeAction(int btn_id) {
  LOG_SECTION_START_MENU("update buttons on click for menu {%d}", btn_id);
  UIButton *b = find_button_by_id(btn_id);
  if (!b) {
    // LOG_PRINTF_MENU("[!] buttonAction_executeAction: no button found for id {%d}\n", btn_id);
    return;
  }

  const char *txt = b->get_text();

  if (strcmp(txt, "poses") == 0) {
    LOG_SECTION_START_MENU("update servos for btn {%s}", txt);
    b->set_is_busy(true);
    drawButtonOverlayById(btn_id);
    setFooter("reading servos...");
    rb.updateInfo();
    b->set_is_busy(false);
    drawButtonOverlayById(btn_id);
    LOG_SECTION_END_MENU();

  } else if (strcmp(txt, "system") == 0) {
    LOG_SECTION_START_MENU("update servos for btn {%s}", txt);
    b->set_is_busy(true);
    drawButtonOverlayById(btn_id);
    setFooter("reading servos...");
    if (rb.updateInfo()) {
      // also get the detailed info if above worked
      rb.requestAllServoInfo();
    }
    b->set_is_busy(false);
    drawButtonOverlayById(btn_id);
    LOG_SECTION_END_MENU();

  } else if (pose_store.is_pose(txt)) {
    LOG_SECTION_START_MENU("run pose {%s}", txt);
    b->set_is_busy(true);
    drawButtonOverlayById(btn_id);
    setFooter(txt);
    pose_store.run_pose(txt);
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

  const char *key = b->get_text();
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
    LOG_SECTION_END_MENU();
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
  buttonAction_executeAction(btn_id);  // blocking call
  millisButtonBusy = 0;

  // --- Navigate to submenu only after operation finished
  if (menuDoc.containsKey(key)) {
    currentMenu = key;
    setFooter((String("switch menu ") + key).c_str());
    buildMenu(currentMenu.c_str());
    LOG_SECTION_END_MENU();
    return;
  }

  // --- Explicit back to main menu
  if (strcmp(key, "main") == 0) {
    currentMenu = "main";
    setFooter("back to main menu");
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
    const char *menuName = kv.key().c_str();
    JsonObject menu = kv.value().as<JsonObject>();
    JsonArray rows = menu["rows"].as<JsonArray>();
    for (JsonArray row : rows) {
      for (JsonObject it : row) {
        const char *type = it["type"] | "";
        const char *key = it["key"] | "";
        if ((strcmp(type, "action") == 0 || strcmp(type, "menu") == 0) && (!key || !*key)) {
          LOG_PRINTF_MENU("menu missing key {%s} type {%s}\n", menuName, type);
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
  buildMenu("main");

  LOG_SECTION_END_MENU();
}

// ----------------------------------------------------------
//                   UI LOOP AND REFRESH
// ----------------------------------------------------------
static unsigned long lastRefresh = 0;

void ui_refresh() {
  unsigned long now = millis();
  if (now - lastRefresh < 30000UL) return;  //TODO adjust this
  lastRefresh = now;

  LOG_SECTION_START_MENU("refresh ui");
  pose_store.reflect_poses_last_run();
  // log_all_buttons(true);
  LOG_SECTION_END_MENU();
}

void ui_loop() {
  lv_timer_handler();
  ui_refresh();
}
