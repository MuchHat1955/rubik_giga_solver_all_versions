#pragma once
#include <Arduino.h>
#include <lvgl.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include "ui_status.h"

// ----------------------------------------------------------
// CONFIGURATION
// ----------------------------------------------------------
extern const int SCREEN_W;
extern const int SCREEN_H;

// main entry points
void ui_init();
void ui_loop();
void ui_refresh();
void buildMenu(const char *menuName);
void buttonAction(const char *key);

// to change UI status
void updateButtonStateByPtr(lv_obj_t* btn, bool issue, bool active);

// utility functions
void setFooter(const char *msg);
int getParamValue(const char *name);
void setParamValue(const char *name, int val);
const char *getStatusText(const char *name);
const char *getStatusColor(const char *name);
void select_num_pair(lv_obj_t *numBox, bool toggle);
lv_color_t colorFromName(const char *name);

// ----------------------------------------------------------
// EXTERNALLY USED OBJECTS
// ----------------------------------------------------------
extern StaticJsonDocument<12288> menuDoc;
extern String currentMenu;
extern lv_obj_t *footLbl;
extern std::map<String, lv_obj_t *> statusWidgets;
extern std::map<String, lv_obj_t *> numLabels;
extern const char jsonBuffer[];

// Styles for numeric buttons
extern lv_style_t style_num_selected;
extern lv_style_t style_num_btn_active;
extern lv_style_t style_num_btn_pressed;

// Version helpers
String getSketchVersion();
String getSketchVersionWithDate();

extern std::map<String, lv_obj_t*> statusWidgets;
extern std::map<String, lv_obj_t*> numLabels;

