#pragma once
#include <lvgl.h>

// Create cube widget inside parent
lv_obj_t* ui_cube_view_create(lv_obj_t* parent);

// Update cube from 54-char color string
void ui_cube_view_set_colors(const String& colors54);
