#pragma once
#include "utils.h"

// ----------------------------------------------------------
//                  UI STATUS MANAGER
// ----------------------------------------------------------
// Handles visual state (error/active) of UI buttons flagged
// with "status":"yes" in JSON.  The state persists even if
// menus are cleared and rebuilt later.
// ----------------------------------------------------------

#pragma once
#include <lvgl.h>
#include <map>
#include <arduino.h>

struct ButtonState {
  lv_obj_t *btn{ nullptr };
  bool issue{ false };
  bool active{ false };
};

extern std::map<String, ButtonState> buttonMap;

void uiStatusClear();
void uiStatusRegisterButton(const String &buttonKey, lv_obj_t *btn);
void updateButtonStateByKey(const String &buttonKey, bool issue, bool active, bool busy);
void drawButtonOverlayByPtr(lv_obj_t *btn, const char *key, bool is_menu, bool issue, bool active, bool busy);
void logButtonMap(bool alwaysLog);
int countOfButtonIssues();
int countOfButtonActive();
void log_lv_obj_info(const lv_obj_t *obj, const char *prefix);
