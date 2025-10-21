#pragma once
#include <lvgl.h>
#include <string>
#include <vector>

class Arduino_GigaDisplayTouch;
class Arduino_H7_Video;

void ui_touch_attach(Arduino_H7_Video* disp, Arduino_GigaDisplayTouch* touch);
void ui_init();
void ui_loop_once();

void ui_set_title(const char* t);
void ui_show_main_menu();
void ui_show_solve();
void ui_show_read();
void ui_show_scramble();

// Progress + stats
void drawProgressBarGeneric(int y_offset,
                            const std::vector<std::string>& items,
                            int currentIndex,
                            const std::vector<uint8_t>& states,
                            bool showCounter);
void drawSolveStats(int y, int current, int total, unsigned long elapsed_ms);
void cube_draw_to_lvgl(const char face[6][9], const char faceToColor[6],
                       const std::string& highlight);

// Exported flow flags
extern bool solvingActive;
extern bool readingActive;
