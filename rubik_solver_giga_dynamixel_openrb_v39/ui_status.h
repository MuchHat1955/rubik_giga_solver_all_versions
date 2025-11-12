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
void updateButtonPtrByText(const char* btn_text, lv_obj_t *btn);
void drawButtonOverlayById(int btn_id);
void drawButtonOverlayByText(const char* btn_text);
void log_lv_obj_info(const lv_obj_t *obj, const char *prefix);
