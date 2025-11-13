#pragma once
#include <Arduino.h>
#include <lvgl.h>

// ============================================================================
// UIButton class
// ============================================================================
class UIButton {
public:
  UIButton();
  UIButton(int id, const char* text, lv_obj_t* ptr = nullptr, bool is_menu = false);

  // --- Getters ---
  int get_id() const;
  const char* get_text() const;
  lv_obj_t* get_ptr() const;
  bool get_is_menu() const;
  bool get_is_active() const;
  bool get_has_issue() const;
  bool get_is_busy() const;

  // --- Setters ---
  void set_id(int id);
  void set_text(const char* txt);
  void set_ptr(lv_obj_t* obj);
  void set_is_menu(bool val);
  void set_is_active(bool val);
  void set_has_issue(bool val);
  void set_is_busy(bool val);

  // --- Helpers ---
  bool is_button(const char* txt) const;
  bool is_button(int id) const;

private:
  int id_ = -1;
  String text_;
  lv_obj_t* ptr_ = nullptr;

  bool is_menu_ = false;
  bool is_active_ = false;
  bool has_issue_ = false;
  bool is_busy_ = false;
};

// ============================================================================
// Global functions and registry
// ============================================================================
extern UIButton ui_buttons[];
extern const int UI_BUTTON_COUNT;

UIButton* find_button_by_text(const char* txt);
UIButton* find_button_by_id(int id);
void clear_all_button_ptrs();
void log_button_by_id(int id);
void log_button_by_text(const char* txt);
void log_all_buttons(bool only_if_not_on_default = false);
