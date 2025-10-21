#include "ui_status.h"
#include "ui_touch.h"
#include "logging.h"
#include <lvgl.h>

static std::map<String, ButtonState> buttonMap;

// Color definitions
static const lv_color_t COLOR_OK = lv_color_hex(0x40C040);      // green
static const lv_color_t COLOR_ACTIVE = lv_color_hex(0x4040FF);  // blue
static const lv_color_t COLOR_ERROR = lv_color_hex(0xC02020);   // red

// ----------------------------------------------------------
//                       CORE LOGIC
// ----------------------------------------------------------

void uiStatusClear() {
  buttonMap.clear();
}

// ----------------------------------------------------------
// Register a button when created in buildMenu()
// ----------------------------------------------------------
void uiStatusRegisterButton(const String &buttonKey, lv_obj_t *btn) {
  if (!btn) return;
  ButtonState &b = buttonMap[buttonKey];
  b.btn = btn;

  // Restore previous visual state
  if (b.issue || b.active)
    updateButtonStateByPtr(btn, b.issue, b.active);
}

// ----------------------------------------------------------
// Update button visuals by pointer (error + active)
// ----------------------------------------------------------
void updateButtonStateByPtr(lv_obj_t *btn, bool issue, bool active) {
  if (!btn) return;

  lv_color_t borderColor = COLOR_OK;
  if (issue) borderColor = COLOR_ERROR;
  else if (active) borderColor = COLOR_ACTIVE;

  // Apply visual border
  lv_obj_set_style_border_color(btn, borderColor, 0);
  lv_obj_set_style_border_width(btn, 3, 0);
  lv_obj_set_style_radius(btn, 8, 0);

  // Remove any previous overlays
  lv_obj_t *existing = lv_obj_get_child(btn, 0);
  if (existing && lv_obj_get_user_data(existing) == (void *)0xDEAD)
    lv_obj_del(existing);

  // Add a red diagonal overlay for issue
  if (issue) {
    lv_obj_t *line = lv_obj_create(btn);
    lv_obj_remove_style_all(line);
    lv_obj_set_user_data(line, (void *)0xDEAD);
    lv_obj_set_size(line, lv_obj_get_width(btn), lv_obj_get_height(btn));
    lv_obj_set_style_bg_opa(line, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(line, 3, 0);
    lv_obj_set_style_border_color(line, COLOR_ERROR, 0);
    lv_obj_set_style_transform_angle(line, 45, 0);
    lv_obj_center(line);
  }

  lv_obj_invalidate(btn);
}

// ----------------------------------------------------------
// Update state by key (used by ServoManager and UI refresh)
// ----------------------------------------------------------
void updateButtonStateByKey(const String &buttonKey, bool issue, bool active) {
  ButtonState &b = buttonMap[buttonKey];
  b.issue = issue;
  b.active = active;
  if (b.btn)
    updateButtonStateByPtr(b.btn, issue, active);
}

// ----------------------------------------------------------
// Random state simulation (for testing without hardware)
// ----------------------------------------------------------
void updateAllServoVisualsRandom() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  if (now - lastUpdate < 2000) return;
  lastUpdate = now;

  for (auto &kv : buttonMap) {
    bool issue = (random(0, 10) < 2);   // 20% chance
    bool active = (random(0, 10) < 3);  // 30% chance
    updateButtonStateByKey(kv.first, issue, active);
  }
}

// ----------------------------------------------------------
// Individual toggles (optional external helpers)
// ----------------------------------------------------------
void setButtonIssueOverlay(ButtonState &s, bool issue) {
  s.issue = issue;
  if (s.btn)
    updateButtonStateByPtr(s.btn, issue, s.active);
}

void setButtonActiveOutline(ButtonState &s, bool active) {
  s.active = active;
  if (s.btn)
    updateButtonStateByPtr(s.btn, s.issue, active);
}
