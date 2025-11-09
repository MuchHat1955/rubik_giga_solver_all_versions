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
  LOG_PRINTF("START clear button map\n");
  int count = 0;
  int countIssue = 0;
  int countActive = 0;
  for (auto &entry : buttonMap) {    // ✅ no const
    ButtonState &st = entry.second;  // now writable
    st.btn = nullptr;
    count++;
    if (st.issue) countIssue++;
    if (st.active) countActive++;
  }
  LOG_PRINTF("END clear button map with size {%d} issues {%d} active {%d}\n", count, countIssue, countActive);
}

int countOfButtonIssues() {
  if (!buttonMap.empty()) return 0;
  int countIssues = 0;
  for (const auto &entry : buttonMap) {
    const String &key = entry.first;
    const ButtonState &st = entry.second;
    if (st.issue) countIssues++;
  }
  return countIssues;
}

int countOfButtonActive() {
  if (!buttonMap.empty()) return 0;
  int countActive = 0;
  for (const auto &entry : buttonMap) {
    const String &key = entry.first;
    const ButtonState &st = entry.second;
    if (st.active) countActive++;
  }
  return countActive;
}

static int countIssuesLastLog = -1;
static int countActiveLastLog = -1;
unsigned long millisLastLog = 0;

void logButtonMap(bool alwaysLog) {

  if (millis() < millisLastLog + 6000) return;
  millisLastLog = millis();

  int countIssues = 0;
  int countActive = 0;

  if (!buttonMap.empty()) {
    for (const auto &entry : buttonMap) {
      const String &key = entry.first;
      const ButtonState &st = entry.second;
      if (st.issue) countIssues++;
      if (st.active) countActive++;
    }
  }

  if (countIssues == countIssuesLastLog &&  //
      countActive == countActiveLastLog &&  //
      !alwaysLog) return;

  countIssuesLastLog = countIssues;
  countActiveLastLog = countActive;
  millisLastLog = millis();

  LOG_SECTION_START("log button map");

  if (buttonMap.empty()) {
    LOG_PRINTF("buttonMap is empty\n");
    LOG_SECTION_END();
    return;
  }
  int count = 1;

  LOG_PRINTF("Button map contains {%u} entries and {%d} with issues\n", (unsigned)buttonMap.size(), countIssues);
  for (const auto &entry : buttonMap) {
    const String &key = entry.first;
    const ButtonState &st = entry.second;
    LOG_PRINTF("%02d.  key{%s} btn_ptr{%p} active{%s} issue{%s}\n",
               count,
               key.c_str(),
               (void *)st.btn,
               st.active ? "yes" : "no",
               st.issue ? "yes" : "no");
    count++;
  }

  LOG_SECTION_END();
}

// ----------------------------------------------------------
// Register a button when created in buildMenu()
// ----------------------------------------------------------
void uiStatusRegisterButton(const String &buttonKey, lv_obj_t *btn) {
  if (!btn) return;

  auto it = buttonMap.find(buttonKey);
  if (it != buttonMap.end()) {
    // already exists → only update pointer
    it->second.btn = btn;
    LOG_PRINTF("existing button {%s} reused (issue=%d active=%d)\n",
               buttonKey.c_str(),
               it->second.issue,
               it->second.active);
    setButtonOverlayByPtr(btn, it->second.issue, it->second.active);
    return;
  }

  // brand new entry (create only once)
  ButtonState st;
  st.btn = btn;
  st.issue = false;
  st.active = false;
  buttonMap.emplace(buttonKey, std::move(st));
  LOG_PRINTF("new button {%s} added to map\n", buttonKey.c_str());
}

// ----------------------------------------------------------
// Update button visuals by pointer (error + active)
// ----------------------------------------------------------
void setButtonOverlayByPtr(lv_obj_t *btn, bool issue, bool active) {
  if (!btn) return;

  //LOG_SECTION_START("setButtonOverlayByPtr");
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
// log button by ptr
// ----------------------------------------------------------
void log_lv_obj_info(const lv_obj_t *obj, const char *prefix) {
  if (!obj) {
    LOG_PRINTF("key{%s} lv_obj: (null)\n", prefix ? prefix : "");
    return;
  }

  /* ---------- Object type name (LVGL 8 way) ---------- */
  const char *name = "(unknown)";
#if LVGL_VERSION_MAJOR == 8
  lv_obj_type_t type;
  lv_obj_get_type(obj, &type);  // fills an array of type strings
  name = type.type[0];          // primary type (e.g. "lv_btn")
#endif

  /* ---------- Position & size ---------- */
  lv_area_t coords;
  lv_obj_get_coords(obj, &coords);

  /* ---------- Hidden / focused state ---------- */
  bool hidden = lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN);
  bool focused = false;
#if LV_USE_GROUP
  lv_group_t *grp = lv_obj_get_group(obj);
  if (grp && lv_group_get_focused_obj(grp) == obj)
    focused = true;
#endif

  LOG_PRINTF("    log for btn{%s} type{%s} pos{%d,%d} size{%d,%d} hidden{%s} focused{%s}\n",
             prefix ? prefix : "",
             name ? name : "(unknown)",
             coords.x1, coords.y1,
             lv_obj_get_width(obj), lv_obj_get_height(obj),
             hidden ? "yes" : "no",
             focused ? "yes" : "no");
}
// ----------------------------------------------------------
// Update state by key (used by UI refresh)
// ----------------------------------------------------------
void updateButtonStateByKey(const String &buttonKey, bool issue, bool active) {
  auto it = buttonMap.find(buttonKey);
  if (it == buttonMap.end()) {
    if (issue) LOG_PRINTF("[!] button key{%s} not found for reflect UI issue {%d}\n", buttonKey.c_str(), issue);  //TODO too noisy logging
    return;                                                                                                       // nothing to update
  }

  ButtonState &b = it->second;
  b.issue = issue;
  b.active = active;

  if (b.btn) {
    setButtonOverlayByPtr(b.btn, issue, active);
    if (issue || active) {
      LOG_PRINTF("[!] updating button WITH PTR for key {%s} issue {%d}\n", buttonKey.c_str(), issue);
      log_lv_obj_info(b.btn, buttonKey.c_str());
    }
  } else {
    if (issue || active) LOG_PRINTF("[!] updating button WITH NO PTR for key{%s} issue {%d}\n", buttonKey.c_str(), issue);
  }
}
