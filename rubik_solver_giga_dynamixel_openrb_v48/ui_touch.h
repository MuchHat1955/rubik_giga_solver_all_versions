#pragma once
#include "utils.h"
#include <stdbool.h>

// ----------------------------------------------------------
// CONFIGURATION
// ----------------------------------------------------------
extern const int SCREEN_W;
extern const int SCREEN_H;

// main entry points
void ui_init();
void ui_loop();
void ui_refresh();
void buildMenu(const char *menuKey);
void buttonAction(int btn_id);

// to change UI status
void drawButtonOverlayById(int btn_id);
void updateButtonAndRefreshServosOnClick(int btn_id);

// utility functions
void setFooter_v1(const char *msg);
// double getParamValue(const char *name);
// void setParamValue(const char *name, double val);
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

/* ----------------------------------------------------------
 * Footer semantic states
 * ONE param controls icon, color, blocking, stop button
 * ---------------------------------------------------------- */
typedef enum {
  _INFO,
  _ERROR,
  _RUNNING_STOP,    // running, STOP allowed
  _RUNNING_NOSTOP,  // running, cannot stop
  _DONE_SUCCESS,
  _DONE_ERROR
} footer_state_t;

/* ----------------------------------------------------------
 * Public API
 * ---------------------------------------------------------- */

/* Call once from your menu / screen builder */
void createFooter(lv_obj_t *parent);

/* Update footer state + message */
void setFooter(const char *msg, footer_state_t state = _INFO);

/* Show or hide the dim / input-blocking overlay */
void ui_dim_overlay_create(lv_obj_t *parent);
void ui_dim_overlay_show(bool show);
