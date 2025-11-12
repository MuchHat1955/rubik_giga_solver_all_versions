#include <lvgl.h>

#include "ui_status.h"
#include "ui_touch.h"
#include "ui_theme.h"
#include "logging.h"
#include "rb_interface.h"
#include "ui_button.h"

std::map<String, ButtonState> buttonMap;

// Color definitions
static const lv_color_t COLOR_OK = lv_color_hex(0x40C040);      // green
static const lv_color_t COLOR_ACTIVE = lv_color_hex(0x4040FF);  // blue
static const lv_color_t COLOR_ERROR = lv_color_hex(0xC02020);   // red

// ----------------------------------------------------------
//                       CORE LOGIC
// ----------------------------------------------------------

void uiStatusClear() {
  // fully reset to avoid dangling pointers and map growth
  LOG_PRINTF_MENU("start clear button map\n");
  clear_all_button_ptrs();
  log_all_buttons();
  LOG_PRINTF_MENU("end clear button map\n");
}

static int countIssuesLastLog = -1;
static int countActiveLastLog = -1;
unsigned long millisLastLog = 0;

void logButtonMap(bool alwaysLog) {

  if (millis() < millisLastLog + 500) return;
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

  LOG_SECTION_START_MENU("log button map");

  if (buttonMap.empty()) {
    LOG_PRINTF_MENU("buttonMap is empty\n");
    LOG_SECTION_END_MENU();
    return;
  }
  int count = 1;

  LOG_PRINTF_MENU("Button map contains {%u} entries and {%d} with issues\n", (unsigned)buttonMap.size(), countIssues);
  for (const auto &entry : buttonMap) {
    const String &key = entry.first;
    const ButtonState &st = entry.second;
    LOG_PRINTF_MENU("    ---- {%02d} | key {%s} | btn_ptr {%p} | active {%s} | issue {%s}\n",
                    count,
                    key.c_str(),
                    (void *)st.btn,
                    st.active ? "yes" : "no",
                    st.issue ? "yes" : "no");
    log_lv_obj_info(st.btn, key.c_str());
    count++;
  }

  LOG_SECTION_END_MENU();
}

// ----------------------------------------------------------
// Add a button ptr when created in buildMenu()
// ----------------------------------------------------------
void updateButtonPtr(const String &buttonText, lv_obj_t *btn) {
  if (!btn) return;

  UIButton *btn_ptr = find_button_by_text(buttonText);

  if (!btn_ptr) {
    LOG_ERROR("invalid button text in updateButtonPtr {%s}", buttonText);
    return;  // nothing to update
  }
  btn_ptr->set_ptr(btn);
}

// ----------------------------------------------------------
// Update button visuals by pointer (error + active)
// ----------------------------------------------------------
void drawButtonOverlayById(int btn_id) {
  UIButton *btn_ptr = find_button_by_id(btn_id);
  if (!btn_ptr) {
    LOG_ERROR("[!] drawButtonOverlay: invalid button id {%d}", btn_id);
    return;
  }

  lv_obj_t *btn = btn_ptr->get_ptr();
  if (!btn) {
    LOG_ERROR("[!] drawButtonOverlay: button object is null for id {%d}", btn_id);
    return;
  }

  LOG_PRINTF_MENU("drawButtonOverlay for id {%d} (%s)\n", btn_id, btn_ptr->get_text());
  log_button_by_id(btn_id);

  bool busy = btn_ptr->get_is_busy();
  bool issue = btn_ptr->get_has_issue();
  bool active = btn_ptr->get_is_active();
  bool is_menu = btn_ptr->get_is_menu();

  // Logic normalization
  if (busy || is_menu) {
    issue = false;
    active = false;
  } else if (issue) {
    active = false;
  }

  constexpr int CORNERS = 20;
  constexpr int BORDER_WIDTH_NORMAL = 2;
  constexpr int BORDER_WIDTH_ACTIVE = 2;
  constexpr int BORDER_WIDTH_ISSUE = 2;

  // Base color by type
  lv_color_t baseColor = is_menu ? COLOR_BTN_MENU : COLOR_BTN_ACTION;

#if LV_USE_USER_DATA
  // --- remove old overlay children tagged as overlays
  uint32_t child_cnt = lv_obj_get_child_cnt(btn);
  for (int i = child_cnt - 1; i >= 0; --i) {
    lv_obj_t *child = lv_obj_get_child(btn, i);
    void *ud = lv_obj_get_user_data(child);
    if (ud == (void *)0xDEAD || ud == (void *)0xBEEF) {
      lv_obj_del(child);
    }
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

  // --- BUSY state ---
  if (busy) {
    LOG_PRINTF_MENU("overlay BUSY\n");
    lv_obj_set_style_bg_color(btn, baseColor, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, baseColor, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, BORDER_WIDTH_ISSUE, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, CORNERS, LV_PART_MAIN);

    lv_obj_t *lbl = lv_obj_get_child(btn, 0);
    if (lbl) lv_label_set_text(lbl, "busy...");
  }

  // --- ISSUE state ---
  else if (issue) {
    LOG_PRINTF_MENU("overlay ISSUE\n");
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xC0392B), LV_PART_MAIN);  // red
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, lv_color_hex(0xC0392B), LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, BORDER_WIDTH_ISSUE, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, CORNERS, LV_PART_MAIN);

    lv_obj_t *lbl = lv_obj_get_child(btn, 0);
    if (lbl) lv_label_set_text(lbl, btn_ptr->get_text());
  }

  // --- ACTIVE state ---
  else if (active) {
    LOG_PRINTF_MENU("overlay ACTIVE\n");
    lv_obj_set_style_bg_color(btn, baseColor, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, baseColor, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, BORDER_WIDTH_ACTIVE, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, CORNERS, LV_PART_MAIN);

    lv_obj_t *lbl = lv_obj_get_child(btn, 0);
    if (lbl) lv_label_set_text(lbl, btn_ptr->get_text());
  }

  // --- NORMAL state ---
  else {
    LOG_PRINTF_MENU("overlay NORMAL\n");
    lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, baseColor, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, BORDER_WIDTH_NORMAL, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, CORNERS, LV_PART_MAIN);

    lv_obj_t *lbl = lv_obj_get_child(btn, 0);
    if (lbl) lv_label_set_text(lbl, btn_ptr->get_text());
  }

  lv_obj_invalidate(btn);
  lv_refr_now(NULL);

  // Force redraw passes to ensure UI visibly updates on hardware
  for (int i = 0; i < 6; ++i) {
    lv_timer_handler();
    delay(5);
  }
  delay(15);
}

void drawButtonOverlayByText(const char *btn_txt) {
  UIButton *btn_ptr = find_button_by_text(btn_txt);
  if (!btn_ptr) {
    LOG_ERROR("[!] drawButtonOverlay: invalid button text {%s}", btn_txt);
    return;
  }
  drawButtonOverlayByid(b->get_id());
}

// ----------------------------------------------------------
// log button by ptr
// ----------------------------------------------------------
void log_lv_obj_info(const lv_obj_t *obj, const char *prefix) {
  if (!obj) {
    LOG_PRINTF_MENU("key {%s} lv_obj {null}\n", prefix ? prefix : "");
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

  LOG_PRINTF_MENU("     ---- log for btn {%s} | type {%s} | pos {%d,%d} | size {%d,%d} | hidden {%s} | focused {%s}\n",
                  prefix ? prefix : "",
                  name ? name : "(unknown)",
                  coords.x1, coords.y1,
                  lv_obj_get_width(obj), lv_obj_get_height(obj),
                  hidden ? "yes" : "no",
                  focused ? "yes" : "no");
}
