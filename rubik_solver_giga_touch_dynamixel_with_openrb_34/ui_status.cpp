#include <lvgl.h>

#include "ui_status.h"
#include "ui_touch.h"
#include "ui_theme.h"
#include "logging.h"
#include "rb_interface.h"

static std::map<String, ButtonState> buttonMap;

// Color definitions
static const lv_color_t COLOR_OK = lv_color_hex(0x40C040);      // green
static const lv_color_t COLOR_ACTIVE = lv_color_hex(0x4040FF);  // blue
static const lv_color_t COLOR_ERROR = lv_color_hex(0xC02020);   // red

// ----------------------------------------------------------
//                       CORE LOGIC
// ----------------------------------------------------------

void uiStatusClear() {
  // fully reset to avoid dangling pointers and map growth
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
// Register a button when created in buildMenu()
// ----------------------------------------------------------
void uiStatusRegisterButtonByKey(const String &buttonKey, bool issue) {
  ButtonState &b = buttonMap[buttonKey];
  b.btn = nullptr;
  b.issue = issue;
}

// ----------------------------------------------------------
// Update button visuals by pointer (error + active)
// ----------------------------------------------------------
void updateButtonStateByPtr(lv_obj_t *btn, bool issue, bool active) {
  if (!btn) return;

  //LOG_SECTION_START("updateButtonStateByPtr");
  //LOG_PRINTF("issue {%d} active {%d}\n", issue, active);

  constexpr int CORNERS = 20;
  constexpr int BORDER_WIDTH_NORMAL = 2;
  constexpr int BORDER_WIDTH_ACTIVE = 2;
  constexpr int BORDER_WIDTH_ISSUE = 2;

  // Capture base color (used for normal border)
  lv_color_t baseColor = lv_obj_get_style_border_color(btn, LV_PART_MAIN);

  // Remove any overlay children (skip if LV_USE_USER_DATA disabled)
#if LV_USE_USER_DATA
  uint32_t child_cnt = lv_obj_get_child_cnt(btn);
  for (int i = child_cnt - 1; i >= 0; --i) {
    lv_obj_t *child = lv_obj_get_child(btn, i);
    void *ud = lv_obj_get_user_data(child);
    if (ud == (void *)0xDEAD || ud == (void *)0xBEEF)
      lv_obj_del(child);
  }
#endif

  lv_obj_update_layout(btn);
  lv_coord_t w = lv_obj_get_width(btn);
  lv_coord_t h = lv_obj_get_height(btn);
  if (w == 0 || h == 0) {
    lv_coord_t pw = lv_obj_get_width(lv_obj_get_parent(btn));
    lv_coord_t ph = lv_obj_get_height(lv_obj_get_parent(btn));
    if (w == 0) w = pw;
    if (h == 0) h = ph;
  }

  lv_obj_add_flag(btn, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

  // --- ISSUE: solid red fill, no white border ---
  if (issue) {
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xC0392B), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, lv_color_hex(0xC0392B), LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, BORDER_WIDTH_ISSUE, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, CORNERS, LV_PART_MAIN);
  }

  // --- ACTIVE: solid blue fill, no white border ---
  else if (active) {
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x4040FF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, lv_color_hex(0x4040FF), LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, BORDER_WIDTH_ACTIVE, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, CORNERS, LV_PART_MAIN);
  }

  // --- NORMAL: transparent background, restore base border ---
  else {
    lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, baseColor, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, BORDER_WIDTH_NORMAL, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, CORNERS, LV_PART_MAIN);
  }

  lv_obj_invalidate(btn);
  //LOG_SECTION_END();
}

// ----------------------------------------------------------
// Update state by key (used by UI refresh)
// ----------------------------------------------------------
void updateButtonStateByKey(const String &buttonKey, bool issue, bool active) {  // TODO fix this one and use the new rb interface
  auto it = buttonMap.find(buttonKey);
  if (it == buttonMap.end()) {
    // LOG_PRINTF("add button with no ptr for key{%s} issue {%d}\n", buttonKey.c_str(), issue); //TODO too noisy logging
    return;  // nothing to update
  }

  ButtonState &b = it->second;
  b.issue = issue;
  b.active = active;

  if (b.btn) {
    updateButtonStateByPtr(b.btn, issue, active);
  } else {
    LOG_PRINTF("button entry has null ptr key{%s}\n", buttonKey.c_str());
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
