#include "ui_button.h"
#include "logging.h"
#include "ui_theme.h"

// ============================================================================
// UIButton implementation
// ============================================================================
UIButton::UIButton() {}

UIButton::UIButton(int id, const char* text, const char* key, lv_obj_t* ptr, bool is_menu)
  : id_(id), text_(text ? text : "err"), key_(key ? key : "k_err"), ptr_(ptr), is_menu_(is_menu) {}

// --- Getters ---
int UIButton::get_id() const {
  return id_;
}
const char* UIButton::get_key() const {
  return key_.c_str();
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
void UIButton::set_key(const char* key) {
  key_ = key ? key : "k_err";
}
void UIButton::set_text(const char* text) {
  text_ = text ? text : "err";
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

  { 1, "solve cube", "k_solve_cube", nullptr, false },
  { 2, "read cube", "k_read_cube", nullptr, false },
  { 3, "tests", "k_tests", nullptr, false },
  { 4, "system", "k_system", nullptr, true },

  { 5, "read", "k_solve_cube__read_cube_all", nullptr, true },
  { 6, "solve", "k_solve_cube_solve", nullptr, true },
  { 7, "run", "k_solve_cube_run", nullptr, true },
  { 8, "back", "k_main", nullptr, false },

  { 9, "all", "k_read_cube_all", nullptr, true },
  { 10, "bottom", "k_read_cube_bottom", nullptr, true },
  { 11, "centers", "k_read_cube_centers", nullptr, true },

  { 12, "robot moves", "k_robot_moves", nullptr, false },
  { 13, "cube moves", "k_cube_moves", nullptr, false },
  { 14, "colors and orientation", "k_colors_and_orientation", nullptr, false },

  { 15, "front color", "k_robot_move_front_color_text", nullptr, false },
  { 16, "na", "k_robot_move_front_color_value", nullptr, true },

  { 17, "d+", "k_robot_moves_d_plus", nullptr, true },
  { 18, "d-", "k_robot_moves_d_minus", nullptr, true },
  { 19, "d-", "k_robot_moves_d2", nullptr, true },

  { 20, "z+", "k_robot_moves_z_plus", nullptr, true },
  { 21, "z-", "k_robot_moves_z_minus", nullptr, true },
  { 22, "z-", "k_robot_moves_z2", nullptr, true },

  { 23, "y+", "k_robot_moves_y_plus", nullptr, true },
  { 24, "y-", "k_robot_moves_y_minus", nullptr, true },
  { 25, "y-", "k_robot_moves_y2", nullptr, true },

  { 26, "front color", "k_cube_moves_front_color_text", nullptr, false },
  { 27, "na", "k_cube_moves_front_color_value", nullptr, true },

  { 28, "f+", "k_cube_moves_f_plus", nullptr, true },
  { 29, "f-", "k_cube_moves_f_minus", nullptr, true },
  { 30, "b+", "k_cube_moves_b_plus", nullptr, true },
  { 31, "b-", "k_cube_moves_b_minus", nullptr, true },

  { 32, "u+", "k_cube_moves_u_plus", nullptr, true },
  { 33, "u-", "k_cube_moves_u_minus", nullptr, true },
  { 34, "d+", "k_cube_moves_d_plus", nullptr, true },
  { 35, "d-", "k_cube_moves_d_minus", nullptr, true },

  { 36, "l+", "k_cube_moves_l_plus", nullptr, true },
  { 37, "l-", "k_cube_moves_l_minus", nullptr, true },
  { 38, "r+", "k_cube_moves_r_plus", nullptr, true },
  { 39, "r-", "k_cube_moves_r_minus", nullptr, true },

  { 40, "f++", "k_cube_moves_f_plus_plus", nullptr, true },
  { 41, "b++", "k_cube_moves_b_plus_plus", nullptr, true },
  { 42, "u++", "k_cube_moves_u_plus_plus", nullptr, true },
  { 43, "d++", "k_cube_moves_d_plus_plus", nullptr, true },

  { 44, "l++", "k_cube_moves_l_plus_plus", nullptr, true },
  { 45, "r++", "k_cube_moves_r_plus_plus", nullptr, true },

  { 46, "orientation", "k_orientation_text", nullptr, true },
  { 47, "detect", "k_orientation_detect", nullptr, true },
  { 48, "check", "k_orientation_check", nullptr, true },
  { 49, "clear", "k_orientation_clear", nullptr, true },

  { 50, "color read", "k_orientation_color_read", nullptr, true },
  { 51, "all", "k_orientation_color_read_all", nullptr, true },
  { 52, "bottom", "k_orientation_color_read_bottom", nullptr, true },
  { 53, "sensor", "k_orientation_color_read_sensor", nullptr, true },

  { 54, "color stickers", "k_orientation_color_stickers", nullptr, true },

  { 55, "c1", "k_color_c1", nullptr, true },
  { 56, "c2", "k_color_c2", nullptr, true },
  { 57, "c3", "k_color_c3", nullptr, true },
  { 58, "c4", "k_color_c4", nullptr, true },
  { 59, "c5", "k_color_c5", nullptr, true },
  { 60, "c6", "k_color_c6", nullptr, true },

  { 61, "rotate", "k_color_rotate", nullptr, false },
  { 62, "y+", "k_color_y_plus", nullptr, true },
  { 63, "z+", "k_color_z_plus", nullptr, true }
};


const int UI_BUTTON_COUNT = sizeof(ui_buttons) / sizeof(ui_buttons[0]);

// ============================================================================
// Helper functions
// ============================================================================
UIButton* find_button_by_key(const char* key) {
  if (!key || !*key) return nullptr;
  for (int i = 0; i < UI_BUTTON_COUNT; i++) {
    if (strcmp(ui_buttons[i].get_key(), key) == 0)
      return &ui_buttons[i];
  }
  LOG_PRINTF_MENU("[!] find_button_by_key: no match for {%s}\n", key);
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
                  b->get_key(),
                  b->get_is_menu() ? "yes" : "no",
                  b->get_is_active() ? "yes" : "no",
                  b->get_has_issue() ? "yes" : "no",
                  b->get_is_busy() ? "yes" : "no",
                  b->get_ptr() ? "set" : "null");
}

void log_button_by_key(const char* txt) {
  if (!txt || !*txt) {
    LOG_PRINTF_MENU("[!] logButton: invalid or empty text\n");
    return;
  }

  UIButton* b = find_button_by_key(txt);
  if (!b) {
    LOG_PRINTF_MENU("[!] logButton: no button found for text {%s}\n", txt);
    return;
  }

  LOG_PRINTF_MENU("button log by TEXT {%s} | id {%d} | menu {%s} | active {%s} | issue {%s} | busy {%s} | ptr {%s}\n",
                  b->get_key(),
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
                      ui_buttons[i].get_key(),
                      ui_buttons[i].get_is_menu() ? "yes" : "no",
                      ui_buttons[i].get_is_active() ? "yes" : "no",
                      ui_buttons[i].get_has_issue() ? "yes" : "no",
                      ui_buttons[i].get_is_busy() ? "yes" : "no",
                      ui_buttons[i].get_ptr() ? "set" : "null");
    }
  }
  LOG_SECTION_END_MENU();
}
