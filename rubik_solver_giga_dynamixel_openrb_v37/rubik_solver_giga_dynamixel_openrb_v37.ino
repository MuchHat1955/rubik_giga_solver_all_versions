/* ---------------------------------------------------------------
 *  Rubik Solver — System Architecture Overview
 *  Target: Arduino GIGA R1 WiFi + GIGA Display Shield (ASX00039)
 * ---------------------------------------------------------------
 *  Development Environment:
 *    - Arduino IDE 2.x
 *    - Board package: Arduino mbed GIGA (>= 4.1.x)
 *    - Libraries:
 *        Arduino_H7_Video
 *        Arduino_GigaDisplayTouch
 *        Arduino_GigaDisplay_GFX  (LVGL 9.)
 *        Adafruit_TCS34725
 *        ArduinoJson
 *
 * ---------------------------------------------------------------
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
 * --------------------------------------------------------------- */
#include <arduino.h>
#include <Arduino_H7_Video.h>
#include <lvgl.h>
#include <Arduino_GigaDisplayTouch.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include <algorithm>

#include "utils.h"
#include "logging.h"
#include "param_store.h"
#include "ui_theme.h"
#include "ui_touch.h"
#include "ui_status.h"
#include "rb_interface.h"

extern RBInterface rb;

// --------------------------------------------------------------------------
//                  NOTES
// --------------------------------------------------------------------------
// NOTE 1: to enable the necessay fonts:
//      -update "C:\Users\gigel\AppData\Local\Arduino15\packages\arduino\hardware\mbed_giga\4.4.1\libraries\Arduino_H7_Video\src\lv_conf_8.h" for fonts
//        AND
//      -update "C:\Users\gigel\AppData\Local\Arduino15\packages\arduino\hardware\mbed_giga\4.4.1\libraries\Arduino_H7_Video\src\lv_conf_9.h" for fonts
//          -#define LV_FONT_MONTSERRAT_18 1
//          -#define LV_FONT_MONTSERRAT_22 1
//          -#define LV_FONT_MONTSERRAT_26 1
//          -#define LV_FONT_MONTSERRAT_28 1
//          -#define LV_FONT_MONTSERRAT_30 1
// NOTE 2: see a desciption of the project at the end of this file  ---------
// --------------------------------------------------------------------------

// Refresh interval (in seconds)
#define UI_REFRESH_INTERVAL_SEC 2  // <-- change as needed

// ----------------------------------------------------------
//                   GLOBAL STATE / FORWARD DECLS
// ----------------------------------------------------------
String currentMenu = "main";

const int SCREEN_W = 800;
const int SCREEN_H = 480;

lv_obj_t *footLbl = nullptr;

lv_obj_t *selected_num_box = nullptr;
lv_style_t style_num_selected;     // orange border + light background
lv_style_t style_num_btn_active;   // filled yellow (selected)
lv_style_t style_num_btn_pressed;  // bright flash on press

void buttonAction(const char *key);
int incrementValue(const char *key, int delta);
void initPoseStore();
bool runStartupTests();

// ----------------------------------------------------------
//                   VERSION HELPERS
// ----------------------------------------------------------
String getSketchVersion() {
  String name = F(__FILE__);
  name.toLowerCase();
  int vpos = name.lastIndexOf("_v");
  if (vpos < 0) return "";
  int start = vpos + 2;
  String digits;
  while (start < (int)name.length()) {
    char c = name.charAt(start);
    if (isdigit(c)) digits += c;
    else break;
    start++;
  }
  return digits.length() ? "v" + digits : "";
}

String getSketchVersionWithDate() {
  String ver = getSketchVersion();
  String d = __DATE__;  // e.g. "Oct 20 2025"
  String t = __TIME__;  // e.g. "14:33:59"
  return ver + " " + d + " " + t;
}

void setup() {
  // ----------------------------------------------------------
  // SERIAL INIT (for logs)
  // ----------------------------------------------------------
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 5000) delay(50);  // shorter wait
  delay(200);                                                // small settle delay

  // ----------------------------------------------------------
  // UI + DISPLAY INIT
  // ----------------------------------------------------------
  init_logging();

  LOG_RESET();
  LOG_PRINTF(" ---- setup start ----\n");

  ui_init();  // sets up LVGL + touch + draws initial menu

  // ----------------------------------------------------------
  // STYLE INITIALIZATION (LVGL)
  // ----------------------------------------------------------
  lv_style_init(&style_num_btn_active);
  lv_style_set_bg_opa(&style_num_btn_active, LV_OPA_TRANSP);
  lv_style_set_border_color(&style_num_btn_active, COLOR_BTN_NUM);
  lv_style_set_border_width(&style_num_btn_active, 2);
  lv_style_set_radius(&style_num_btn_active, 20);

  lv_style_init(&style_num_btn_pressed);
  lv_style_set_bg_opa(&style_num_btn_pressed, LV_OPA_TRANSP);
  lv_style_set_border_color(&style_num_btn_pressed, COLOR_BTN_NUM);
  lv_style_set_border_width(&style_num_btn_pressed, 3);
  lv_style_set_radius(&style_num_btn_pressed, 20);

  // ----------------------------------------------------------
  // SERVO SYSTEM INIT
  // ----------------------------------------------------------
  for (int i = 0; i < 10; i++) {
    lv_timer_handler();
    delay(20);
  }
  initParamStore();  // load or create parameter storage
  initPoseStore();   // register servos + group poses

  // ----------------------------------------------------------
  // STARTUP SELF TEST
  // ----------------------------------------------------------
  bool rb_ok = !rb.begin();
  if (!rb_ok) {
    setFooter("rb interface issues detected");
  } else {
    if (!runStartupTests()) {
      setFooter("startup issues detected");
    } else {
      setFooter("startup test ok");
    }
  }
  LOG_PRINTF(" ---- setup end ----\n");
}

// ----------------------------------------------------------
//                     MAIN LOOP
// ----------------------------------------------------------
void loop() {
  LOG_RESET();
  update_bee_logging();

  // Run LVGL event handler
  ui_loop();

  // Optional background refresh (status updates, servo state)
  ui_refresh();

  delay(5);  // ~200Hz main loop
}

/*
 * ----------------------------------------------------------------
 *  Rubik Solver — System Architecture Overview
 *  Target: Arduino GIGA R1 WiFi + GIGA Display Shield (ASX00039)
 * ----------------------------------------------------------------
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
 *         gripper1, gripper2, base).  Provides tick <-> angle
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
 * ----------------------------------------------------------------
 */
