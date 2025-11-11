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

// Forward declarations
void drawButtonOverlayByPtr(lv_obj_t *btn, const char *key, bool is_menu, bool issue, bool active, bool busy);

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

    LOG_PRINTF("set footer {%s}\n", b);

    lv_label_set_text(footLbl, b);
    lv_obj_invalidate(footLbl);
    lv_timer_handler();  // ✅ force redraw immediately
  }
}

// ---- below is for actions to be done when a menu is displayed ---
void updateButtonAndRefreshServosOnClick(const char *key) {
  LOG_SECTION_START("update buttons on click", "for menu {%s}", key);
  // logButtonMap(true);
  if (!key) {
    LOG_PRINTF("invalik key {null}\n");
    LOG_SECTION_END();
    return;
  }

  if (strcmp(key, "poses") == 0) {
    LOG_SECTION_START("update servos for poses UI", "btn {%s}", key);
    setFooter("reading servos...");
    updateButtonStateByKey("poses", false, false, true);
    rb.updateInfo();
    updateButtonStateByKey("poses", false, false, false);
    lv_timer_handler();
    LOG_SECTION_END();
  }
  if (strcmp(key, "system") == 0) {
    LOG_SECTION_START("update servos for poses UI", "btn {%s}", key);
    setFooter("reading servos...");
    updateButtonStateByKey("system", true, false, true);
    rb.updateInfo();
    updateButtonStateByKey("system", true, false, false);
    lv_timer_handler();
    LOG_SECTION_END();
  }

  logButtonMap(true);
  LOG_SECTION_END();
}

// ----------------------------------------------------------
//                   BUTTON ACTION LOGIC
// ----------------------------------------------------------
void buttonAction(const char *key) {
  LOG_SECTION_START("buttonAction", "key {%s}", key ? key : "(null)");

  static unsigned long lastClickTime = 0;
  static String lastClickKey;

  unsigned long now = millis();
  if (key && *key && key == lastClickKey && now - lastClickTime < 500) {
    LOG_PRINTF("button action rapid re-click ignored key {%s}\n", key);
    LOG_SECTION_END();
    return;
  }
  lastClickTime = now;
  lastClickKey = key;

  // --- Safety: guard against null or empty keys ---
  if (!key || !*key) {
    LOG_PRINTF("button action empty or null key ignored\n");
    LOG_SECTION_END();
    return;
  }

  if (strcmp(key, "system") == 0) on_system_menu_opened();

  updateButtonAndRefreshServosOnClick(key);

  // --- Navigate to a submenu if the key matches a menu name ---
  if (menuDoc.containsKey(key)) {
    currentMenu = key;
    setFooter((String("switch menu ") + key).c_str());
    buildMenu(currentMenu.c_str());
    LOG_SECTION_END();
    return;
  }

  // --- Explicit back to main menu ---
  if (strcmp(key, "main") == 0) {
    currentMenu = "main";
    setFooter("back to main menu");
    buildMenu(currentMenu.c_str());
    LOG_SECTION_END();
    return;
  }

  // --- Regular action (pose / servo / group / sequence) ---
  setFooter((String("action ") + key).c_str());
  runAction((char *)key);

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
          LOG_PRINTF("menu missing key {%s} type {%s}\n", menuName, type);
        }
      }
    }
  }
}

extern const char jsonBuffer[];

void ui_init() {
  LOG_SECTION_START("ui_init","");

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
    LOG_PRINTF("error on deserialize json {%s}\n", err.c_str());
    LOG_SECTION_END();
    return;  // avoid using empty doc
  }

  validateMenuKeys();

  // 4. Build the initial main menu screen
  buildMenu("main");

  LOG_SECTION_END();
}

// ----------------------------------------------------------
//                   UI LOOP AND REFRESH
// ----------------------------------------------------------
static unsigned long lastRefresh = 0;

void ui_refresh() {
  unsigned long now = millis();
  logButtonMap(false);
  if (now - lastRefresh < 30000UL) return;  //TODO adjust this
  lastRefresh = now;

  LOG_SECTION_START("refresh ui","");
  pose_store.reflect_poses_ui();
  logButtonMap(true);
  LOG_SECTION_END();
}

void ui_loop() {
  lv_timer_handler();
  ui_refresh();
}
