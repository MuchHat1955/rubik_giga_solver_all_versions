#pragma once
#include <lvgl.h>
#include <map>
#include <vector>

// ----------------------------------------------------------
//                   SCREEN GEOMETRY
// ----------------------------------------------------------
extern const int SCREEN_W;
extern const int SCREEN_H;

// ----------------------------------------------------------
//                   FONT DEFINITIONS
// ----------------------------------------------------------
#define FONT_TITLE_PTR      &lv_font_montserrat_40
#define FONT_BTN_LARGE_PTR  &lv_font_montserrat_30
#define FONT_BTN_SMALL_PTR  &lv_font_montserrat_26
#define FONT_SUBTITLE_PTR   &lv_font_montserrat_26
#define FONT_FOOT_PTR       &lv_font_montserrat_26

// ----------------------------------------------------------
//                   COLOR PALETTE
// ----------------------------------------------------------
static const lv_color_t COLOR_BG          = lv_color_hex(0x000000);
static const lv_color_t COLOR_TEXT        = lv_color_hex(0xE0E0E0);
static const lv_color_t COLOR_BTN_TEXT    = lv_color_hex(0xE0E0E0);
static const lv_color_t COLOR_BTN_ACTION  = lv_color_hex(0x2070FF);  // blue
static const lv_color_t COLOR_BTN_MENU    = lv_color_hex(0x40C040);  // green
static const lv_color_t COLOR_BTN_NUM     = lv_color_hex(0xFF4000);  // orange
static const lv_color_t COLOR_BTN_STATUS  = lv_color_hex(0xE0E0E0);  // gray
static const lv_color_t COLOR_BTN_BACK    = lv_color_hex(0xC04040);  // reddish for back

// ----------------------------------------------------------
//                   LAYOUT CONSTANTS
// ----------------------------------------------------------
constexpr int CORNERS = 20;   // corner radius
constexpr int ROW_H   = 65;   // row pitch for snapping
constexpr int HORIZONTAL_SPACING = 20;
constexpr int VERTICAL_SPACING   = 70;
