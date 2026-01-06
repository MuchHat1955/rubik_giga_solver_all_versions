#include "ui_button.h"
#include "logging.h"
#include "ui_theme.h"
#include "rb_interface.h"
#include "run.h"
#include "ui_status.h"

// ============================================================================
// UIButton implementation
// ============================================================================
UIButton::UIButton()
  : id_(0), text_(""), key_(""), ptr_(nullptr),
    is_status_(false), is_menu_(false),
    is_active_(false), has_issue_(false), is_busy_(false) {}

UIButton::UIButton(int id, const char* text, const char* key, lv_obj_t* ptr, bool is_status, bool is_menu)
  : id_(id), text_(text ? text : "err"), key_(key ? key : "k_err"), ptr_(ptr), is_status_(is_status), is_menu_(is_menu) {}

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
bool UIButton::get_is_status() const {
  return is_status_;
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
void UIButton::set_is_status(bool val) {
  is_status_ = val;
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

static char face_to_color(char face) {
  face = tolower(face);
  switch (face) {
    case 'u': return 'W';
    case 'r': return 'R';
    case 'f': return 'G';
    case 'd': return 'Y';
    case 'l': return 'O';
    case 'b': return 'B';
    default: return '?';
  }
}

String ori_text_to_compact(const char* ori_text) {
  char f_phys = '?';
  char r_phys = '?';

  if (!ori_text || !*ori_text) {
    return String("f:? r:?");
  }

  const char* p = ori_text;

  // Look for patterns like "f->x" and "r->x"
  while ((p = strstr(p, "->")) != nullptr) {

    // char before "->" is the cube face
    char cube_face = *(p - 1);

    // char after "->" is the physical face (skip _ or space)
    const char* q = p + 2;
    while (*q == '_' || *q == ' ') {
      q++;
    }

    if (*q == '\0') break;  // safety

    char phys_face = *q;

    if (cube_face == 'f') f_phys = phys_face;
    if (cube_face == 'r') r_phys = phys_face;

    p = q + 1;  // advance to avoid infinite loop
  }

  char f_color = face_to_color(f_phys);
  char r_color = face_to_color(r_phys);

  char out[16];
  snprintf(out, sizeof(out), "f:%c r:%c", f_color, r_color);

  return String(out);
}

void buttons_set_text_ori(const char* ori_text) {
  UIButton* btn_ptr = nullptr;

  btn_ptr = find_button_by_key("k_orientation_val");
  // convert ori text to small format
  String ori_text_compact = ori_text_to_compact(ori_text);
  if (btn_ptr) btn_ptr->set_text(ori_text_compact.c_str());
  set_last_orientation(ori_text);
}

void buttons_set_text_front_color(char* clr_text) {
  UIButton* btn_ptr = nullptr;

  btn_ptr = find_button_by_key("k_cube_moves_front_color_value");
  if (btn_ptr) btn_ptr->set_text(clr_text);
  btn_ptr = find_button_by_key("k_robot_move_front_color_value");
  if (btn_ptr) btn_ptr->set_text(clr_text);
  btn_ptr = find_button_by_key("k_color_read_front_color_value");
  if (btn_ptr) btn_ptr->set_text(clr_text);
}

void buttons_set_text_by_key(const char* key, const char* a_text, bool refresh_ui) {
  if (!key || !a_text) return;

  UIButton* btn_ptr = find_button_by_key(key);
  if (!btn_ptr) return;

  LOG_PRINTF("buttons_set_text_by_key \nkey{%s} \ntext{%s} \nrefresh ui{%d}\n", key, a_text, refresh_ui);

  btn_ptr->set_text(a_text);
  lv_obj_t* lv_btn = btn_ptr->get_ptr();
  lv_obj_t* lbl = lv_obj_get_child(lv_btn, 0);
  if (lbl) lv_label_set_text(lbl, a_text);

  if (!refresh_ui) return;

  lv_obj_invalidate(lv_btn);
  lv_refr_now(NULL);

  // Force redraw passes to ensure UI visibly updates on hardware
  for (int i = 0; i < 6; ++i) {
    lv_timer_handler();
    delay(5);
  }
  delay(15);
}

void buttons_set_color_string(const char* color_string) {
  set_last_color_string_54(color_string);
}
void buttons_set_one_color_string(const char one_color) {
  char clr_char = one_color;

  String key_str = String("k_color_c") + String(last_onecolor_read_slot);
  buttons_set_text_by_key(key_str.c_str(), String(clr_char).c_str());
}
// ============================================================================
// Global button registry
// ============================================================================

// All interactive/menu buttons extracted from your JSON (no num/text types)
//  { TEXT, KEY, nullptr, STATUS, MENU);
UIButton ui_buttons[] = {

  { 1, "solve cube", "k_solve_cube", nullptr, false, true },
  { 2, "read cube", "k_read_cube", nullptr, false, true },
  { 3, "tests", "k_tests", nullptr, false, true },
  { 4, "system", "k_system", nullptr, false, true },

  { 5, "read", "k_solve_cube_read_cube_all", nullptr, true, false },
  { 6, "solve", "k_solve_cube_find_solution", nullptr, true, false },
  { 7, "run", "k_solve_cube_run_solution", nullptr, true, false },
  { 8, "back", "k_main", nullptr, false, true },

  { 9, "all", "k_read_cube_all", nullptr, true, false },
  { 10, "bottom", "k_read_cube_bottom", nullptr, true, false },
  { 11, "centers", "k_read_cube_centers", nullptr, true, false },

  { 12, "robot moves", "k_robot_moves", nullptr, false, true },
  { 13, "cube moves", "k_cube_moves", nullptr, false, true },
  { 14, "colors and orientation", "k_colors_and_orientation", nullptr, false, true },

  { 15, "front color is", "", nullptr, false, false },
  { 16, "color na", "k_robot_move_front_color_value", nullptr, false, false },

  { 17, "d+", "k_robot_moves_d_plus", nullptr, true, false },
  { 18, "d-", "k_robot_moves_d_minus", nullptr, true, false },
  { 19, "d-", "k_robot_moves_d2", nullptr, true, false },

  { 20, "z+", "k_robot_moves_z_plus", nullptr, true, false },
  { 21, "z-", "k_robot_moves_z_minus", nullptr, true, false },
  { 22, "z-", "k_robot_moves_z2", nullptr, true, false },

  { 23, "y+", "k_robot_moves_y_plus", nullptr, true, false },
  { 24, "y-", "k_robot_moves_y_minus", nullptr, true, false },
  { 25, "y-", "k_robot_moves_y2", nullptr, true, false },

  { 26, "front color is:", "", nullptr, false, false },
  { 27, "color na", "k_cube_moves_front_color_value", nullptr, true, false },

  { 28, "f+", "k_cube_moves_f_plus", nullptr, true, false },
  { 29, "f-", "k_cube_moves_f_minus", nullptr, true, false },
  { 30, "f2", "k_cube_moves_f2", nullptr, true, false },

  { 31, "r+", "k_cube_moves_r_plus", nullptr, true, false },
  { 32, "r-", "k_cube_moves_r_minus", nullptr, true, false },
  { 33, "r2", "k_cube_moves_r2", nullptr, true, false },

  { 34, "u+", "k_cube_moves_u_plus", nullptr, true, false },
  { 35, "u-", "k_cube_moves_u_minus", nullptr, true, false },
  { 36, "u2", "k_cube_moves_u2", nullptr, true, false },

  { 37, "b+", "k_cube_moves_b_plus", nullptr, true, false },
  { 38, "b-", "k_cube_moves_b_minus", nullptr, true, false },
  { 39, "b2", "k_cube_moves_b2", nullptr, true, false },

  { 40, "l+", "k_cube_moves_l_plus", nullptr, true, false },
  { 41, "l-", "k_cube_moves_l_minus", nullptr, true, false },
  { 42, "l2", "k_cube_moves_l2", nullptr, true, false },

  { 43, "d+", "k_cube_moves_d_plus", nullptr, true, false },
  { 44, "d-", "k_cube_moves_d_minus", nullptr, true, false },
  { 45, "d2", "k_cube_moves_d2", nullptr, true, false },

  { 46, "orientation", "", nullptr, false, false },
  { 47, "orientation na", "k_orientation_val", nullptr, true, false },

  { 48, "detect", "k_orientation_detect", nullptr, true, false },
  { 49, "check", "k_orientation_check", nullptr, true, false },
  { 50, "restore", "k_orientation_restore", nullptr, true, false },

  { 51, "color read", "", nullptr, false, false },
  { 52, "front is", "k_color_read_front_color_value", nullptr, true, false },

  { 53, "all", "k_orientation_color_read_all", nullptr, true, false },
  { 54, "bottom", "k_orientation_color_read_bottom", nullptr, true, false },
  { 55, "sensor", "k_orientation_color_read_sensor", nullptr, true, false },

  { 56, "color stickers", "", nullptr, false, false },

  { 57, "c1", "k_color_c1", nullptr, true, false },
  { 58, "c2", "k_color_c2", nullptr, true, false },
  { 59, "c3", "k_color_c3", nullptr, true, false },
  { 60, "c4", "k_color_c4", nullptr, true, false },
  { 61, "c5", "k_color_c5", nullptr, true, false },
  { 62, "c6", "k_color_c6", nullptr, true, false },

  { 63, "rotate", "k_color_rotate", nullptr, false, false },
  { 64, "y+", "k_color_y_plus", nullptr, true, false },
  { 65, "z+", "k_color_z_plus", nullptr, true, false },

  { 66, "info", "k_servos_info", nullptr, true, false },
  { 67, "zero", "k_run_zero", nullptr, true, false },
  { 68, "stop", "k_set_stop_all", nullptr, true, false },
  { 69, "reboot", "k_reboot_all", nullptr, true, false },

  { 70, "", "k_system_info_text", nullptr, false, false },  // this is the text for servos info
  { 71, "back", "k_tests", nullptr, false, true }
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
  LOG_ERR("[MENU] find_button_by_key: no match for {%s}\n", key);
  return nullptr;
}

UIButton* find_button_by_id(int id) {
  for (int i = 0; i < UI_BUTTON_COUNT; i++) {
    if (ui_buttons[i].get_id() == id)
      return &ui_buttons[i];
  }
  LOG_ERR("[MENU] find_button_by_id: no match for id {%d}\n", id);
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
    LOG_ERR("[MENU] logButton: no button found for id {%d}\n", id);
    return;
  }

  LOG_PRINTF_MENU("button log by ID {%d} | text {%s} | status {%s}  | menu {%s} | active {%s} | issue {%s} | busy {%s} | ptr {%s}\n",
                  b->get_id(),
                  b->get_key(),
                  b->get_is_status() ? "status" : "not status",
                  b->get_is_menu() ? "menu" : "not menu",
                  b->get_is_active() ? "active" : "not active",
                  b->get_has_issue() ? "has issue" : "no issue",
                  b->get_is_busy() ? "busy" : "not busy",
                  b->get_ptr() ? "ptr set" : "null");
}

void log_button_by_key(const char* txt) {
  if (!txt || !*txt) {
    LOG_ERR("[MENU] logButton: invalid or empty text\n");
    return;
  }

  UIButton* b = find_button_by_key(txt);
  if (!b) {
    LOG_ERR("[MENU] logButton: no button found for text {%s}\n", txt);
    return;
  }

  LOG_PRINTF_MENU("button log by KEY {%s} | id {%d} | menu {%s} | active {%s} | issue {%s} | busy {%s} | ptr {%s}\n",
                  b->get_key(),
                  b->get_id(),
                  b->get_is_status() ? "yes" : "no",
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
                      ui_buttons[i].get_is_status() ? "yes" : "no",
                      ui_buttons[i].get_is_active() ? "yes" : "no",
                      ui_buttons[i].get_has_issue() ? "yes" : "no",
                      ui_buttons[i].get_is_busy() ? "yes" : "no",
                      ui_buttons[i].get_ptr() ? "set" : "null");
    }
  }
  LOG_SECTION_END_MENU();
}
