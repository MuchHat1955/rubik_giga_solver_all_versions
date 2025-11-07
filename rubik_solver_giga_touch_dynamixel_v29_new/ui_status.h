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

void uiStatusClear();
void uiStatusRegisterButton(const String &buttonKey, lv_obj_t *btn);
void updateButtonStateByKey(const String &buttonKey, bool issue, bool active);
void updateButtonStateByPtr(lv_obj_t* btn, bool issue, bool active);

