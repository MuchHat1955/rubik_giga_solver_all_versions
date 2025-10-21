#include "ui_touch.h"
#include "logging.h"
#include "param_store.h"
#include "servos.h"
#include "Arduino_H7_Video.h"
#include <Arduino_GigaDisplayTouch.h>
#include "ui_theme.h"
#include <map>
#include <vector>
#include <algorithm>

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
//                   UTILITY HELPERS
// ----------------------------------------------------------
lv_color_t colorFromName(const char *name) {
  if (!name) return lv_color_hex(0xE0E0E0);
  if (!strcasecmp(name, "green")) return lv_color_hex(0x00FF00);
  if (!strcasecmp(name, "red")) return lv_color_hex(0xFF4040);
  if (!strcasecmp(name, "yellow")) return lv_color_hex(0xFFFF00);
  if (!strcasecmp(name, "orange")) return lv_color_hex(0xFFA000);
  if (!strcasecmp(name, "gray")) return lv_color_hex(0xE0E0E0);
  return lv_color_hex(0xE0E0E0);
}

void setFooter(const char *msg) {
  if (footLbl) lv_label_set_text(footLbl, msg);
}

void updateStatus(const char *key, const char *value, const char *colorName) {
  if (!statusWidgets.count(key)) return;
  lv_obj_t *obj = statusWidgets[key];
  lv_color_t col = colorFromName(colorName);
  lv_label_set_text(obj, value);
  lv_obj_set_style_text_color(obj, col, 0);
}

// ----------------------------------------------------------
//                   BUTTON ACTION LOGIC
// ----------------------------------------------------------
void buttonAction(const char *key) {
  LOG_SECTION_START_VAR("buttonAction", "key", key);

  if (menuDoc.containsKey(key)) {  // navigate to submenu
    currentMenu = key;
    setFooter((String("switch menu ") + key).c_str());
    buildMenu(currentMenu.c_str());
  } else if (strcmp(key, "main") == 0) {  // explicit back to main
    currentMenu = "main";
    setFooter("back to main menu");
    buildMenu(currentMenu.c_str());
  } else {
    setFooter((String("action ") + key).c_str());
    runAction(key);
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
extern const char jsonBuffer[];

void ui_init() {
  LOG_SECTION_START("ui_init");

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
    LOG_VAR("error on deserialize json", err.c_str());
    LOG_SECTION_END();
    return;  // avoid using empty doc
  }

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
  if (now - lastRefresh < 2000UL) return;
  lastRefresh = now;

  // Example dummy status update
  static int dummyStall = 250;
  static int dummyTemp = 30;

  dummyStall += random(-2, 3);
  dummyTemp += random(-1, 2);

  dummyStall = constrain(dummyStall, 230, 270);
  dummyTemp = constrain(dummyTemp, 28, 35);

  char buf[32];
  snprintf(buf, sizeof(buf), "%d mA", dummyStall);
  updateStatus("stall", buf, "orange");

  snprintf(buf, sizeof(buf), "%d C", dummyTemp);
  updateStatus("temp", buf, "green");
}

void ui_loop() {
  lv_timer_handler();
  ui_refresh();
}
