#pragma once
#include "utils.h"

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
void buttonAction(int btn_id);

// to change UI status
void drawButtonOverlayById(int btn_id);
void updateButtonAndRefreshServosOnClick(int btn_id);

// utility functions
void setFooter(const char *msg);
double getParamValue(const char *name);
void setParamValue(const char *name, double val);
const char *getStatusText(const char *name);
const char *getStatusColor(const char *name);
void select_num_pair(lv_obj_t *numBox, bool toggle);

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

// Declare shared widget maps (no definitions here)
extern std::map<String, lv_obj_t *> statusWidgets;
extern std::map<String, lv_obj_t *> numLabels;
