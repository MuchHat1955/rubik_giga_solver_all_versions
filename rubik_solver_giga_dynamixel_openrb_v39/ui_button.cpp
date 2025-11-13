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
  // Main menu
  { 1, "solve cube", nullptr, true },
  { 2, "read cube", nullptr, true },
  { 3, "scramble cube", nullptr, true },
  { 4, "tests", nullptr, true },
  { 5, "system", nullptr, true },
  { 61, "main", nullptr, true },

  // Solve/read/random/system menus
  { 6, "start", nullptr, false },
  { 7, "start read", nullptr, false },
  { 8, "scramble (12)", nullptr, false },
  { 9, "scramble (20)", nullptr, false },

  // Tests
  { 11, "poses", nullptr, true },
  { 12, "sequences", nullptr, true },
  { 13, "cube moves", nullptr, true },

  // Sequences
  { 14, "bottom+", nullptr, false },
  { 15, "bottom-", nullptr, false },
  { 16, "front to base", nullptr, false },
  { 17, "back to base", nullptr, false },
  { 18, "left to base", nullptr, false },
  { 19, "right to base", nullptr, false },
  { 20, "top to base", nullptr, false },
  { 21, "rotate down face+", nullptr, false },
  { 22, "rotate down face-", nullptr, false },

  // Cube moves
  { 23, "f+", nullptr, false },
  { 24, "f-", nullptr, false },
  { 25, "b+", nullptr, false },
  { 26, "b-", nullptr, false },
  { 27, "u+", nullptr, false },
  { 28, "u-", nullptr, false },
  { 29, "d+", nullptr, false },
  { 30, "d-", nullptr, false },
  { 31, "l+", nullptr, false },
  { 32, "l-", nullptr, false },
  { 33, "r+", nullptr, false },
  { 34, "r-", nullptr, false },
  { 35, "f++", nullptr, false },
  { 36, "b++", nullptr, false },
  { 37, "u++", nullptr, false },
  { 38, "d++", nullptr, false },
  { 39, "l++", nullptr, false },
  { 40, "r++", nullptr, false },

  // Poses menu (only action-type)
  { 41, "y zero", nullptr, false },
  { 42, "y 1st", nullptr, false },
  { 43, "y 2nd", nullptr, false },
  { 44, "y 3rd", nullptr, false },
  { 45, "x center", nullptr, false },
  { 46, "x right", nullptr, false },
  { 47, "x left", nullptr, false },
  { 48, "y c2", nullptr, false },
  { 49, "y c3", nullptr, false },
  { 50, "wrist vert", nullptr, false },
  { 51, "wrist horiz right", nullptr, false },
  { 52, "wrist horiz left", nullptr, false },
  { 53, "grippers open", nullptr, false },
  { 61, "grippers close", nullptr, false },
  { 54, "gripper 1 open", nullptr, false },
  { 55, "gripper 1 close", nullptr, false },
  { 56, "gripper 2 open", nullptr, false },
  { 57, "gripper 2 close", nullptr, false },
  { 58, "base front", nullptr, false },
  { 59, "base right", nullptr, false },
  { 60, "base left", nullptr, false }
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
