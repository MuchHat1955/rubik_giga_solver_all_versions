// ---------------- See a desciption of the project at the end of this file  ----------------
#include <Arduino.h>
#include "Arduino_H7_Video.h"
#include "lvgl.h"
#include "Arduino_GigaDisplayTouch.h"
#include <ArduinoJson.h>
#include <map>

// ----------------------------------------------------------
//                  NOTES
// ----------------------------------------------------------
// --- update "C:\Users\gigel\AppData\Local\Arduino15\packages\arduino\hardware\mbed_giga\4.4.1\libraries\Arduino_H7_Video\src\lv_conf_8.h" for fonts
// --- update "C:\Users\gigel\AppData\Local\Arduino15\packages\arduino\hardware\mbed_giga\4.4.1\libraries\Arduino_H7_Video\src\lv_conf_9.h" for fonts
// --- see a desciption of the project at the end of this file  ----------------

// ----------------------------------------------------------
//                  CONFIGURATION & COLORS
// ----------------------------------------------------------
static const int SCREEN_W = 800;
static const int SCREEN_H = 480;

// Font sizes
static const int FONT_TITLE_SIZE = 22;
static const int FONT_SUBTITLE_SIZE = 14;
static const int FONT_FOOT_SIZE = 12;
static const int FONT_BTN_LARGE = 18;
static const int FONT_BTN_SMALL = 14;

#define FONT_TITLE_PTR &lv_font_montserrat_22
#define FONT_SUBTITLE_PTR &lv_font_montserrat_14
#define FONT_FOOT_PTR &lv_font_montserrat_12
#define FONT_BTN_LARGE_PTR &lv_font_montserrat_18
#define FONT_BTN_SMALL_PTR &lv_font_montserrat_14

// Base colors
static const lv_color_t COLOR_BG = lv_color_hex(0x000000);
static const lv_color_t COLOR_TEXT = lv_color_hex(0xE0E8FF);
static const lv_color_t COLOR_OUTLINE = lv_color_hex(0x556677);
static const lv_color_t COLOR_ACTIVE = lv_color_hex(0x2070FF);

// Type-specific colors
static const lv_color_t COLOR_BTN_ACTION = lv_color_hex(0x2070FF);  // blue
static const lv_color_t COLOR_BTN_MENU = lv_color_hex(0x40C040);    // green
static const lv_color_t COLOR_BTN_NUM = lv_color_hex(0xC09020);     // orange
static const lv_color_t COLOR_BTN_STATUS = lv_color_hex(0x808080);  // gray

// Refresh interval (in seconds)
#define UI_REFRESH_INTERVAL_SEC 2  // <-- change as needed

#define __LOG_VAR(_n, _v) \
  Serial.print(_n); \
  Serial.print(" {"); \
  Serial.print(_v); \
  Serial.println("}");

// ----------------------------------------------------------
//                   GLOBAL STATE / FORWARD DECLS
// ----------------------------------------------------------
static String currentMenu = "main";
StaticJsonDocument<12288> menuDoc;

static lv_obj_t *footLbl = nullptr;
static std::map<String, lv_obj_t *> numLabels;
static std::map<String, lv_obj_t *> statusWidgets;

void buildMenu(const char *menuName);
void buttonAction(const char *key);
int incrementValue(const char *key, int delta);
void updateStatus(const char *key, const char *value, const char *colorName);

// ----------------------------------------------------------
//                    HELPER UTILITIES
// ----------------------------------------------------------
static lv_color_t colorFromName(const char *name) {
  if (!name) return COLOR_TEXT;
  if (!strcasecmp(name, "green")) return lv_color_hex(0x00FF00);
  if (!strcasecmp(name, "red")) return lv_color_hex(0xFF4040);
  if (!strcasecmp(name, "yellow")) return lv_color_hex(0xFFFF00);
  if (!strcasecmp(name, "orange")) return lv_color_hex(0xFFA000);
  if (!strcasecmp(name, "gray")) return lv_color_hex(0x808080);
  return COLOR_TEXT;
}

static void setFooter(const char *msg) {
  if (footLbl) lv_label_set_text(footLbl, msg);
  // __LOG_VAR("footer", msg);
}

// Increment storage (simple)
int incrementValue(const char *key, int delta) {
  static std::map<String, int> values;
  values[key] += delta;
  if (values[key] < 0) values[key] = 0;
  return values[key];
}

// Update status label or button text/color
void updateStatus(const char *key, const char *value, const char *colorName) {
  if (!statusWidgets.count(key)) return;
  lv_obj_t *obj = statusWidgets[key];
  lv_color_t col = colorFromName(colorName);
  lv_label_set_text(obj, value);
  lv_obj_set_style_text_color(obj, col, 0);
}

// ----------------------------------------------------------
//                     BUTTON ACTION LOGIC
// ----------------------------------------------------------
void buttonAction(const char *key) {
  if (menuDoc.containsKey(key)) {  // navigate to submenu
    currentMenu = key;
    setFooter((String("Switch menu -> ") + key).c_str());
    buildMenu(currentMenu.c_str());
  } else if (strcmp(key, "main") == 0) {  // explicit back to main
    currentMenu = "main";
    setFooter("Back to main menu");
    buildMenu(currentMenu.c_str());
  } else {
    setFooter((String("Action: ") + key).c_str());
  }
  __LOG_VAR("button action", key);
}

// ----------------------------------------------------------
//                     MENU BUILDER
// ----------------------------------------------------------
void buildMenu(const char *menuName) {

  __LOG_VAR("build menu start", menuName);

  lv_obj_t *scr = lv_scr_act();
  if (scr == nullptr) {
    __LOG_VAR("lvgl screen not ready yet", "null");
    return;
  }

  lv_obj_clean(lv_scr_act());
  statusWidgets.clear();
  numLabels.clear();
  JsonObject root = menuDoc[menuName];

  if (root.isNull()) {
    __LOG_VAR("root", "null");
    return;
  }

  const char *title = root["title"] | "";
  const char *subtitle = root["subtitle"] | "";
  const char *footer = root["footer"] | "";

  int columns = root["columns"] | 1;
  JsonArray rows = root["rows"].as<JsonArray>();

  lv_obj_set_style_bg_color(scr, COLOR_BG, 0);
  lv_obj_set_style_text_color(scr, COLOR_TEXT, 0);

  // Title
  lv_obj_t *titleLbl = lv_label_create(scr);
  __LOG_VAR("   title", title);
  lv_label_set_text(titleLbl, title);
  lv_obj_set_style_text_font(titleLbl, FONT_TITLE_PTR, 0);
  lv_obj_align(titleLbl, LV_ALIGN_TOP_MID, 0, 6);

  // Subtitle
  lv_obj_t *subLbl = lv_label_create(scr);
  __LOG_VAR("   sub title", subtitle);
  lv_label_set_text(subLbl, subtitle);
  lv_obj_set_style_text_font(subLbl, FONT_SUBTITLE_PTR, 0);
  lv_obj_align(subLbl, LV_ALIGN_TOP_MID, 0, 30);

  const lv_font_t *btnFont = (columns <= 2) ? FONT_BTN_LARGE_PTR   //TODO use the #define
                                            : FONT_BTN_SMALL_PTR;  //TODO use the #define

  // Scrollable area for tall menus
  lv_obj_t *cont = lv_obj_create(scr);
  lv_obj_set_size(cont, SCREEN_W, SCREEN_H - 80);
  lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, 55);
  lv_obj_set_scroll_dir(cont, LV_DIR_VER);
  lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(cont, 0, 0);

  int margin = 10;
  int availW = SCREEN_W - margin * 2;
  int colW = availW / columns - 10;
  int y = 0;

  for (JsonArray row : rows) {
    int numCols = row.size();
    int xStart = (numCols < columns)
                   ? (SCREEN_W - (numCols * (colW + 10))) / 2
                   : margin;

    for (int ci = 0; ci < numCols; ci++) {
      JsonObject it = row[ci];
      const char *txt = it["text"] | "";
      const char *type = it["type"] | "text";
      const char *key = it["key"] | "";
      int x = xStart + ci * (colW + 10);

      __LOG_VAR("    type", type);

      // ---------- TEXT ----------
      if (strcmp(type, "text") == 0) {
        lv_obj_t *lbl = lv_label_create(cont);
        lv_label_set_text(lbl, txt);
        lv_obj_set_style_text_font(lbl, btnFont, 0);
        lv_obj_set_pos(lbl, x, y);
      }

      // ---------- ACTION / MENU ----------
      else if (strcmp(type, "action") == 0 || strcmp(type, "menu") == 0) {
        lv_obj_t *btn = lv_btn_create(cont);
        lv_obj_set_size(btn, colW, 48);
        lv_obj_set_pos(btn, x, y);
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_border_color(
          btn,
          (strcmp(type, "menu") == 0) ? COLOR_BTN_MENU : COLOR_BTN_ACTION,
          0);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, txt);
        lv_obj_center(lbl);
        lv_obj_add_event_cb(
          btn, [](lv_event_t *e) {
            const char *k = (const char *)lv_event_get_user_data(e);
            setFooter((String("Press ") + k).c_str());
            buttonAction(k);
          },
          LV_EVENT_CLICKED, (void *)key);
      }

      // ---------- NUMERIC ----------
      else if (strcmp(type, "num") == 0) {
        int v = it["value"] | 0;
        lv_obj_t *lbl = lv_label_create(cont);
        lv_label_set_text_fmt(lbl, "%s: %d", txt, v);
        lv_obj_set_pos(lbl, x, y);
        lv_obj_set_style_text_color(lbl, COLOR_BTN_NUM, 0);
        lv_obj_set_style_text_font(lbl, btnFont, 0);
        numLabels[key] = lbl;

        lv_obj_t *plus = lv_btn_create(cont);
        lv_obj_set_size(plus, 30, 30);
        lv_obj_set_pos(plus, x + colW - 35, y);
        lv_obj_set_style_bg_opa(plus, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_color(plus, COLOR_BTN_NUM, 0);
        lv_obj_set_style_border_width(plus, 2, 0);
        lv_obj_t *pl = lv_label_create(plus);
        lv_label_set_text(pl, "+");
        lv_obj_center(pl);
        lv_obj_add_event_cb(
          plus, [](lv_event_t *e) {
            const char *k = (const char *)lv_event_get_user_data(e);
            int newV = incrementValue(k, +1);
            lv_label_set_text_fmt(numLabels[k], "%s: %d", k, newV);
            setFooter((String("Increment ") + k).c_str());
          },
          LV_EVENT_CLICKED, (void *)key);

        lv_obj_t *minus = lv_btn_create(cont);
        lv_obj_set_size(minus, 30, 30);
        lv_obj_set_pos(minus, x + colW - 70, y);
        lv_obj_set_style_bg_opa(minus, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_color(minus, COLOR_BTN_NUM, 0);
        lv_obj_set_style_border_width(minus, 2, 0);
        lv_obj_t *ml = lv_label_create(minus);
        lv_label_set_text(ml, "-");
        lv_obj_center(ml);
        lv_obj_add_event_cb(
          minus, [](lv_event_t *e) {
            const char *k = (const char *)lv_event_get_user_data(e);
            int newV = incrementValue(k, -1);
            lv_label_set_text_fmt(numLabels[k], "%s: %d", k, newV);
            setFooter((String("Decrement ") + k).c_str());
          },
          LV_EVENT_CLICKED, (void *)key);
      }

      // ---------- STATUS BUTTON ----------
      else if (strcmp(type, "status_button") == 0) {
        const char *status = it["status"] | "";
        const char *colStr = it["color"] | "gray";
        lv_color_t sColor = colorFromName(colStr);

        lv_obj_t *btn = lv_btn_create(cont);
        lv_obj_set_size(btn, colW, 48);
        lv_obj_set_pos(btn, x, y);
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_color(btn, COLOR_BTN_STATUS, 0);
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_radius(btn, 10, 0);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text_fmt(lbl, "%s: %s", txt, status);
        lv_obj_set_style_text_color(lbl, sColor, 0);
        lv_obj_center(lbl);
        statusWidgets[key] = lbl;

        lv_obj_add_event_cb(
          btn, [](lv_event_t *e) {
            const char *k = (const char *)lv_event_get_user_data(e);
            setFooter((String("Press status ") + k).c_str());
            buttonAction(k);
          },
          LV_EVENT_CLICKED, (void *)key);
      }

      // ---------- STATUS TEXT ----------
      else if (strcmp(type, "status_text") == 0) {
        const char *status = it["status"] | "";
        const char *colStr = it["color"] | "gray";
        lv_color_t sColor = colorFromName(colStr);
        lv_obj_t *lbl = lv_label_create(cont);
        lv_label_set_text_fmt(lbl, "%s: %s", txt, status);
        lv_obj_set_style_text_color(lbl, sColor, 0);
        lv_obj_set_pos(lbl, x, y);
        statusWidgets[key] = lbl;
      }
    }
    y += 60;
  }

  // Footer
  footLbl = lv_label_create(scr);
  lv_label_set_text(footLbl, footer);
  lv_obj_set_style_text_font(footLbl, &lv_font_montserrat_12, 0);
  lv_obj_align(footLbl, LV_ALIGN_BOTTOM_MID, 0, -4);

  __LOG_VAR("build menu", "end");
}
// ----------------------------------------------------------
//                     Example JSON with all types
// ----------------------------------------------------------
static char jsonBuffer[] = R"json(
  {
    "main": {
      "title": "Rubik Solver",
      "subtitle": "v5.0",
      "footer": "Ready.",
      "columns": 2,
      "rows": [
        [
          {"text": "Solve Cube",  "type": "action", "key": "solve"},
          {"text": "Read Colors", "type": "menu",   "key": "tests"}
        ],
        [
          {"text": "Diagnostics", "type": "menu",   "key": "diag"},
          {"text": "Settings",    "type": "menu",   "key": "settings"}
        ]
      ]
    },
    "tests": {
      "title": "Rubik Solver",
      "subtitle": "Tests Menu",
      "footer": "Perform tests",
      "columns": 3,
      "rows": [
        [
          {"text": "Motor", "type": "status_button", "key": "motor", "status": "Not run", "color": "gray"},
          {"text": "Color", "type": "status_button", "key": "color", "status": "N/A", "color": "gray"},
          {"text": "Back",  "type": "menu", "key": "main"}
        ],
        [
          {"text": "Value A", "type": "num", "key": "valA", "value": 5},
          {"text": "Value B", "type": "num", "key": "valB", "value": 25},
          {"text": "NA", "type": "text"}
        ],
        [
          {"text": "Current", "type": "status_text", "key": "stall", "status": "250 mA", "color": "orange"},
          {"text": "Temp",    "type": "status_text", "key": "temp", "status": "30 C", "color": "green"},
          {"text": "NA", "type": "text"}
        ]
      ]
    },
    "settings": {
      "title": "Settings",
      "subtitle": "Adjustments",
      "footer": "Scroll for more...",
      "columns": 1,
      "rows": [
        [{"text": "Brightness", "type": "num", "key": "bright", "value": 70}],
        [{"text": "Volume", "type": "num", "key": "volume", "value": 5}],
        [{"text": "Mode", "type": "action", "key": "mode"}],
        [{"text": "Network", "type": "status_text", "key": "net", "status": "Connected", "color": "green"}],
        [{"text": "Back", "type": "menu", "key": "main"}]
      ]
    },
    "diag": {
      "title": "Diagnostics",
      "subtitle": "Scroll Demo",
      "footer": "Scroll down...",
      "columns": 2,
      "rows": [
        [{"text": "Row 1A", "type": "action", "key": "r1a"}, {"text": "Row 1B", "type": "action", "key": "r1b"}],
        [{"text": "Row 2A", "type": "action", "key": "r2a"}, {"text": "Row 2B", "type": "action", "key": "r2b"}],
        [{"text": "Row 3A", "type": "action", "key": "r3a"}, {"text": "Row 3B", "type": "action", "key": "r3b"}],
        [{"text": "Row 4A", "type": "action", "key": "r4a"}, {"text": "Row 4B", "type": "action", "key": "r4b"}],
        [{"text": "Row 5A", "type": "action", "key": "r5a"}, {"text": "Row 5B", "type": "action", "key": "r5b"}],
        [{"text": "Row 6A", "type": "action", "key": "r6a"}, {"text": "Row 6B", "type": "action", "key": "r6b"}],
        [{"text": "Back", "type": "menu", "key": "main"}, {"text": "NA", "type": "text"}]
      ]
    }
  })json";

// ----------------------------------------------------------
//                     LVGL INITIALIZATION
// ----------------------------------------------------------
void ui_init() {
  // 1. Initialize LVGL core
  lv_init();

  // 2. Initialize the display hardware
  static Arduino_H7_Video Display(800, 480, GigaDisplayShield);
  static Arduino_GigaDisplayTouch TouchDetector;

  Display.begin();  // sets up framebuffer and LVGL display driver
  TouchDetector.begin();

  // 3. Parse JSON AFTER display init
 DeserializationError err = deserializeJson(menuDoc, jsonBuffer);
  if (err) {
    __LOG_VAR("json error", err.c_str());
    return;  // avoid using empty doc
  }
  __LOG_VAR("json ok", err.c_str());
  __LOG_VAR("   top keys", menuDoc.as<JsonObject>().size());
  __LOG_VAR("   mem used", menuDoc.memoryUsage());

  buildMenu(currentMenu.c_str());
}

// ----------------------------------------------------------
//              PERIODIC REFRESH FUNCTION
// ----------------------------------------------------------

// Track last refresh time
static unsigned long lastRefresh = 0;

void ui_refresh() {
  unsigned long now = millis();
  if (now - lastRefresh < UI_REFRESH_INTERVAL_SEC * 1000UL) return;
  lastRefresh = now;

  // Example: update dynamic fields
  static int dummyStall = 250;
  static int dummyTemp = 30;

  dummyStall += (random(-2, 3));  // simulate current fluctuation
  dummyTemp += (random(-1, 2));   // simulate temp fluctuation

  // Keep values in range
  if (dummyStall < 230) dummyStall = 230;
  if (dummyStall > 270) dummyStall = 270;
  if (dummyTemp < 28) dummyTemp = 28;
  if (dummyTemp > 35) dummyTemp = 35;

  // Update displayed values
  char buf[32];
  snprintf(buf, sizeof(buf), "%d mA", dummyStall);
  updateStatus("stall", buf, "orange");

  snprintf(buf, sizeof(buf), "%d C", dummyTemp);
  updateStatus("temp", buf, "green");

  // Optional footer note to see refresh activity
  setFooter("refreshed status data");
}

void ui_loop() {
  lv_timer_handler();
  ui_refresh();  // <-- call refresh periodically
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 9999) delay(111);
  delay(333);

  __LOG_VAR("setup", "start");

  ui_init();

  __LOG_VAR("setup", "end");
}

void loop() {
  ui_loop();
  delay(5);
}

/**
 * ================================================================
 *  Rubik Solver — System Architecture Overview
 *  Target: Arduino GIGA R1 WiFi + GIGA Display Shield (ASX00039)
 * ================================================================
 *
 *  Hardware:
 *    - 800x480 LVGL touchscreen (Arduino_H7_Video + GigaDisplayTouch)
 *    - Dynamixel XL430 servos via Serial1 + DIR pin
 *    - Adafruit TCS34725 color sensor on I2C (SDA=20, SCL=21)
 *    - No rotary encoder; touch-only interface
 *
 * ---------------------------------------------------------------
 *  Purpose:
 *    A self-contained Rubik’s Cube solving robot with a full
 *    touchscreen UI.  Uses LVGL for menus, Dynamixel2Arduino
 *    for motion, and Adafruit TCS34725 for color detection.
 *
 * ---------------------------------------------------------------
 *  Core Architecture
 *
 *  1. Hardware Control Layer
 *     - servos.cpp/.h:
 *         Controls 6 Dynamixel servos (arm1, arm2, wrist,
 *         grip1, grip2, base).  Provides tick <-> angle
 *         helpers and goal position movement.
 *     - color_sensor.cpp/.h:
 *         Reads RGB data from TCS34725, converts to HSV,
 *         classifies as W/R/G/Y/O/B cube colors.
 *     - Timing:
 *         Uses Arduino millis()/delay() for sequencing.
 *
 *  2. Parameter Management Layer
 *     - param_store.cpp/.h:
 *         Holds all calibration data: servo limits, poses,
 *         derived values (e.g. base ±30°, ±60°).
 *         Provides get/set functions and automatic updates.
 *
 *  3. Action & Sequence Layer
 *     - action_store.cpp/.h:
 *         Maps text commands ("arms_row1", "wrist_90", etc.)
 *         to servo poses or sequences (POSE/GROUP/SEQUENCE).
 *     - group_motion.cpp/.h:
 *         Moves multiple servos together with cosine easing.
 *     - orientation_map.cpp:
 *         Tracks cube orientation (F/R/U etc.) and remaps
 *         logical moves to current physical axes.
 *
 *  4. Solver Logic Layer
 *     - read_cube.cpp:
 *         Uses color sensor to read cube faces, updates model.
 *     - solve.cpp:
 *         Executes move list, shows progress bar.
 *     - scramble.cpp:
 *         Generates random scramble sequences.
 *     - cube_colors.cpp/.h:
 *         Holds 6 faces (3x3 each), draws cube net in LVGL.
 *
 *  5. User Interface Layer (Touch + LVGL)
 *     - ui_touch.cpp/.h:
 *         Touch menus built with LVGL 8.2.
 *         Buttons respond instantly to taps.
 *         Numeric fields enter edit mode showing
 *         small +/- buttons for value change.
 *         Long menus show ▲/▼ arrows for scrolling.
 *     - Theme:
 *         Dark background (0x101215), bright text (0xF5F8FF),
 *         buttons with LV_SYMBOL icons.
 *
 *  6. Main Control Layer
 *     - rubik_solver_giga_touch.ino:
 *         Initializes Display, LVGL, touch, servos, sensor.
 *         Loads main menu and runs LVGL event loop.
 *
 * ---------------------------------------------------------------
 *  System Flow:
 *     1. Setup
 *        - LVGL, touch, and hardware initialize.
 *        - Params and actions load.
 *        - Servos centered, UI shown.
 *
 *     2. User Interaction
 *        - Tap a main menu button:
 *          "Solve Cube", "Read Colors",
 *          "Random Cube", or "Tests".
 *
 *     3. Execution
 *        - runGroupPoseServos() drives coordinated motion.
 *        - Progress and cube visuals update live.
 *
 *     4. Completion
 *        - Returns to idle screen; user can start again.
 *
 * ---------------------------------------------------------------
 *  UI Design Principles:
 *    - Touch-first, no rotary encoder.
 *    - Minimal navigation depth (2 levels max).
 *    - Immediate feedback for each tap.
 *    - High-contrast dark theme for readability.
 *
 * ---------------------------------------------------------------
 *  Hardware I/O Summary:
 *    - Dynamixel bus: Serial1 (TX1=PA9, RX1=PA10) + D2 (DIR)
 *    - Color sensor: SDA=20, SCL=21 (I2C)
 *    - Display Shield: internal DPI + I2C (no pin conflict)
 *    - Power: 5V logic, separate 12V for servos, shared GND
 *
 * ---------------------------------------------------------------
 *  Development Environment:
 *    - Arduino IDE 2.x
 *    - Board package: Arduino mbed GIGA (>= 4.1.x)
 *    - Libraries:
 *        Arduino_H7_Video
 *        Arduino_GigaDisplayTouch
 *        Arduino_GigaDisplay_GFX  (LVGL 8.2)
 *        Dynamixel2Arduino
 *        Adafruit_TCS34725
 *        ArduinoJson (optional)
 *
 * ---------------------------------------------------------------
 *  Future Extensions:
 *    - Save/load parameters to Flash or SD.
 *    - Implement on-board solver algorithm (Kociemba).
 *    - Real-time 3D cube visualization in LVGL.
 *    - Wi-Fi or USB telemetry.
 *
 * ---------------------------------------------------------------
 *  Summary:
 *    - Clean separation of UI, control logic, and hardware.
 *    - Runs entirely on Arduino GIGA R1 + Display Shield.
 *    - Provides a responsive, touch-based Rubik Solver system.
 *
 * ================================================================
 */
