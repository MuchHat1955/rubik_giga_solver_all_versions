#include "ui_button.h"
#include "logging.h"
#include "ui_theme.h"

// ============================================================================
// UIButton implementation
// ============================================================================
UIButton::UIButton() {}

UIButton::UIButton(int id, const char* text, lv_obj_t* ptr, bool is_menu)
  : id_(id), text_(text ? text : ""), ptr_(ptr), is_menu_(is_menu) {}

// --- Getters ---
int UIButton::get_id() const {
  return id_;
}
const char* UIButton::get_text() const {
  return text_.c_str();
}
lv_obj_t* UIButton::get_ptr() const {
  return ptr_;
}
bool UIButton::get_is_menu() const {
  return is_menu_;
}
bool UIButton::get_is_active() const {
  return is_active_;
}
bool UIButton::get_has_issue() const {
  return has_issue_;
}
bool UIButton::get_is_busy() const {
  return is_busy_;
}

// --- Setters ---
void UIButton::set_id(int id) {
  id_ = id;
}
void UIButton::set_text(const char* txt) {
  text_ = txt ? txt : "";
}
void UIButton::set_ptr(lv_obj_t* obj) {
  ptr_ = obj;
}
void UIButton::set_is_menu(bool val) {
  is_menu_ = val;
}
void UIButton::set_is_active(bool val) {
  is_active_ = val;
}
void UIButton::set_has_issue(bool val) {
  has_issue_ = val;
}
void UIButton::set_is_busy(bool val) {
  is_busy_ = val;
}

// ============================================================================
// Global button registry
// ============================================================================

// All interactive/menu buttons extracted from your JSON (no num/text types)
UIButton ui_buttons[] = {
  { 1, "k_solve_cube", nullptr, false },
  { 2, "k_read_cube", nullptr, false },
  { 3, "k_tests", nullptr, false },
  { 4, "k_system", nullptr, true },

  { 5, "k_find_solution", nullptr, true },
  { 6, "k_main", nullptr, false },

  { 7, "k_robot_moves", nullptr, false },
  { 8, "k_cube_moves", nullptr, false },
  { 9, "k_colors_and_orientation", nullptr, false },

  { 10, "k_d_plus", nullptr, true },
  { 11, "k_d_minus", nullptr, true },
  { 12, "k_d2", nullptr, true },

  { 13, "k_z_plus", nullptr, true },
  { 14, "k_z_minus", nullptr, true },
  { 15, "k_z2", nullptr, true },

  { 16, "k_y_plus", nullptr, true },
  { 17, "k_y_minus", nullptr, true },
  { 18, "k_y2", nullptr, true },

  { 19, "k_f_plus", nullptr, true },
  { 20, "k_f_minus", nullptr, true },
  { 21, "k_b_plus", nullptr, true },
  { 22, "k_b_minus", nullptr, true },

  { 23, "k_u_plus", nullptr, true },
  { 24, "k_u_minus", nullptr, true },

  { 25, "k_l_plus", nullptr, true },
  { 26, "k_l_minus", nullptr, true },
  { 27, "k_r_plus", nullptr, true },
  { 28, "k_r_minus", nullptr, true },

  { 29, "k_f_plus_plus", nullptr, true },
  { 30, "k_b_plus_plus", nullptr, true },
  { 31, "k_u_plus_plus", nullptr, true },
  { 32, "k_d_plus_plus", nullptr, true },
  { 33, "k_l_plus_plus", nullptr, true },
  { 34, "k_r_plus_plus", nullptr, true },

  { 35, "k_orientation", nullptr, true },
  { 36, "k_detect", nullptr, true },
  { 37, "k_check", nullptr, true },
  { 38, "k_clear", nullptr, true },

  { 39, "k_color_read", nullptr, true },

  { 40, "k_all", nullptr, true },
  { 41, "k_bottom", nullptr, true },
  { 42, "k_sensor", nullptr, true },

  { 43, "k_color_stickers", nullptr, true },

  { 44, "k_c1", nullptr, true },
  { 45, "k_c2", nullptr, true },
  { 46, "k_c3", nullptr, true },
  { 47, "k_c4", nullptr, true },
  { 48, "k_c5", nullptr, true },
  { 49, "k_c6", nullptr, true },

  { 50, "k_rotate", nullptr, false }
};

const int UI_BUTTON_COUNT = sizeof(ui_buttons) / sizeof(ui_buttons[0]);

// ============================================================================
// Helper functions
// ============================================================================
UIButton* find_button_by_text(const char* txt) {
  if (!txt || !*txt) return nullptr;
  for (int i = 0; i < UI_BUTTON_COUNT; i++) {
    if (strcmp(ui_buttons[i].get_text(), txt) == 0)
      return &ui_buttons[i];
  }
  LOG_PRINTF_MENU("[!] find_button_by_text: no match for {%s}\n", txt);
  return nullptr;
}

UIButton* find_button_by_id(int id) {
  for (int i = 0; i < UI_BUTTON_COUNT; i++) {
    if (ui_buttons[i].get_id() == id)
      return &ui_buttons[i];
  }
  LOG_PRINTF_MENU("[!] find_button_by_id: no match for id {%d}\n", id);
  return nullptr;
}

void clear_all_button_ptrs() {
  for (int i = 0; i < UI_BUTTON_COUNT; i++)
    ui_buttons[i].set_ptr(nullptr);
  LOG_PRINTF_MENU("all button LVGL ptrs cleared\n");
}

// ============================================================================
// LOG BUTTON HELPERS
// ============================================================================

void log_button_by_id(int id) {
  UIButton* b = find_button_by_id(id);
  if (!b) {
    LOG_PRINTF_MENU("[!] logButton: no button found for id {%d}\n", id);
    return;
  }

  LOG_PRINTF_MENU("button log by ID {%d} | text {%s} | menu {%s} | active {%s} | issue {%s} | busy {%s} | ptr {%s}\n",
                  b->get_id(),
                  b->get_text(),
                  b->get_is_menu() ? "yes" : "no",
                  b->get_is_active() ? "yes" : "no",
                  b->get_has_issue() ? "yes" : "no",
                  b->get_is_busy() ? "yes" : "no",
                  b->get_ptr() ? "set" : "null");
}

void log_button_by_text(const char* txt) {
  if (!txt || !*txt) {
    LOG_PRINTF_MENU("[!] logButton: invalid or empty text\n");
    return;
  }

  UIButton* b = find_button_by_text(txt);
  if (!b) {
    LOG_PRINTF_MENU("[!] logButton: no button found for text {%s}\n", txt);
    return;
  }

  LOG_PRINTF_MENU("button log by TEXT {%s} | id {%d} | menu {%s} | active {%s} | issue {%s} | busy {%s} | ptr {%s}\n",
                  b->get_text(),
                  b->get_id(),
                  b->get_is_menu() ? "yes" : "no",
                  b->get_is_active() ? "yes" : "no",
                  b->get_has_issue() ? "yes" : "no",
                  b->get_is_busy() ? "yes" : "no",
                  b->get_ptr() ? "set" : "null");
}


// ============================================================================
// LOG ALL BUTTONS
// ============================================================================
void log_all_buttons(bool only_if_not_on_default) {
  LOG_SECTION_START_MENU("log_all_buttons with {%s}", only_if_not_on_default ? "only if not on default" : "all");

  for (int i = 0; i < UI_BUTTON_COUNT; i++) {
    bool to_log = true;
    if (only_if_not_on_default) {
      to_log = false;
      if (ui_buttons[i].get_is_active()) to_log = true;
      if (ui_buttons[i].get_has_issue()) to_log = true;
      if (ui_buttons[i].get_is_busy()) to_log = true;
    }
    if (to_log) {
      LOG_PRINTF_MENU("[%d] id {%d} | text {%s} | menu {%s} | active {%s} | issue {%s} | busy {%s} | ptr {%s}\n",
                      i + 1,
                      ui_buttons[i].get_id(),
                      ui_buttons[i].get_text(),
                      ui_buttons[i].get_is_menu() ? "yes" : "no",
                      ui_buttons[i].get_is_active() ? "yes" : "no",
                      ui_buttons[i].get_has_issue() ? "yes" : "no",
                      ui_buttons[i].get_is_busy() ? "yes" : "no",
                      ui_buttons[i].get_ptr() ? "set" : "null");
    }
  }
  LOG_SECTION_END_MENU();
}
