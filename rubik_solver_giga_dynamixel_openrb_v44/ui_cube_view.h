#pragma once
#include <lvgl.h>
#include <Arduino.h>

// Create cube widget inside parent
lv_obj_t* ui_cube_view_create(lv_obj_t* parent);

// Update cube from 54-char color string
void ui_cube_view_set_colors(const String& colors54);

lv_obj_t* ui_moves_progress_create(lv_obj_t* parent, int w, int h);

void ui_moves_progress_set(String moves_str,
                           String colors_str);

void ui_moves_progress_set_index(int idx);
