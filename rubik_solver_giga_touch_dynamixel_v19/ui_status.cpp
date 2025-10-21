#include "ui_status.h"
#include "ui_touch.h"
#include "ui_theme.h"
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
  for (auto &entry : buttonMap) {
    entry.second.btn = nullptr;  // ✅ clear only the pointer
    entry.second.issue = false;  // optional: reset flags too
    entry.second.active = false;
  }
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

  LOG_SECTION_START("updateButtonStateByPtr");
  LOG_VAR2("issue", issue, "active", active);

  const int CORNERS = 20;  // fixed rounded corner radius

  /* 1️⃣  Get the button's current outline color */
  lv_color_t baseColor = lv_obj_get_style_border_color(btn, LV_PART_MAIN);

  /* 2️⃣  Remove any previous overlays (tagged 0xDEAD / 0xBEEF) */
  uint32_t child_cnt = lv_obj_get_child_cnt(btn);
  for (int i = child_cnt - 1; i >= 0; --i) {
    lv_obj_t *child = lv_obj_get_child(btn, i);
    void *ud = lv_obj_get_user_data(child);
    if (ud == (void *)0xDEAD || ud == (void *)0xBEEF) {
      LOG_VAR("removing old overlay", (int)(uintptr_t)ud);
      lv_obj_del(child);
    }
  }

  /* Force layout update to ensure valid size */
  lv_obj_update_layout(btn);
  lv_coord_t w = lv_obj_get_width(btn);
  lv_coord_t h = lv_obj_get_height(btn);
  if (w == 0 || h == 0) {
    lv_coord_t pw = lv_obj_get_width(lv_obj_get_parent(btn));
    lv_coord_t ph = lv_obj_get_height(lv_obj_get_parent(btn));
    if (w == 0) w = pw;
    if (h == 0) h = ph;
  }
  LOG_VAR2("button size after layout w", w, "h", h);

  /* Allow overlay to cover full border area */
  lv_obj_add_flag(btn, LV_OBJ_FLAG_OVERFLOW_VISIBLE);  // ✅ LVGL 9 replacement

  /* 3️⃣  ISSUE overlay : red translucent fill */
  if (issue) {
    LOG_VAR("creating ISSUE overlay", "");
    lv_obj_t *overlay = lv_obj_create(btn);
    lv_obj_remove_style_all(overlay);
    lv_obj_set_user_data(overlay, (void *)0xDEAD);
    lv_obj_set_size(overlay, w, h);
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0xFF3399), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_radius(overlay, CORNERS, LV_PART_MAIN);
    lv_obj_align(overlay, LV_ALIGN_CENTER, 0, 0);  // perfect alignment
    lv_obj_move_foreground(overlay);
  }

  /* 4️⃣  ACTIVE overlay : semi-transparent tint of button color */
  else if (active) {
    LOG_VAR("creating ACTIVE overlay", "");
    lv_obj_t *overlay = lv_obj_create(btn);
    lv_obj_remove_style_all(overlay);
    lv_obj_set_user_data(overlay, (void *)0xBEEF);
    lv_obj_set_size(overlay, w, h);
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(overlay, baseColor, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_20, LV_PART_MAIN);  // 30% tint
    lv_obj_set_style_radius(overlay, CORNERS, LV_PART_MAIN);
    lv_obj_align(overlay, LV_ALIGN_CENTER, 0, 0);
    lv_obj_move_foreground(overlay);
  }

  /* 5️⃣  Restore button border */
  lv_obj_set_style_border_color(btn, baseColor, LV_PART_MAIN);
  lv_obj_set_style_border_width(btn, 2, LV_PART_MAIN);
  lv_obj_set_style_radius(btn, CORNERS, LV_PART_MAIN);

  lv_obj_invalidate(btn);
  LOG_SECTION_END();
}

// ----------------------------------------------------------
// Update state by key (used by ServoManager and UI refresh)
// ----------------------------------------------------------
void updateButtonStateByKey(const String &buttonKey, bool issue, bool active) {
  auto it = buttonMap.find(buttonKey);
  if (it == buttonMap.end()) {
    LOG_VAR("no button found for key", buttonKey);
    return;  // nothing to update
  }

  ButtonState &b = it->second;
  b.issue = issue;
  b.active = active;

  if (b.btn) {
    updateButtonStateByPtr(b.btn, issue, active);
  } else {
    LOG_VAR("button entry has null ptr", buttonKey);
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
