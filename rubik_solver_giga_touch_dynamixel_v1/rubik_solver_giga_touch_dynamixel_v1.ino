#include <Arduino.h>
#ifdef abs
#undef abs
#endif
#include "Arduino_H7_Video.h"
#include "lvgl.h"
#include "Arduino_GigaDisplayTouch.h"

#include "ui_touch.h"
#include "param_store.h"
#include "action_store.h"
#include "color_sensor.h"
#include "servos.h"

// ---------------- See a desciption of the project at the end of this file  ----------------

// ---------------- Display + Touch ----------------
static Arduino_H7_Video Display(800, 480, GigaDisplayShield);
static Arduino_GigaDisplayTouch Touch;

// Exported flags used by flows
bool solvingActive = false;
bool readingActive = false;

// ---------------- compatibility helpers  ----------------
#ifndef clamp
template<typename T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
  return (v < lo) ? lo : (v > hi) ? hi
                                  : v;
}
#endif

// ---------------- LVGL tick ----------------
static void lv_tick_cb() {
  lv_tick_inc(5);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.print("LVGL version: ");
  Serial.print(lv_version_major());
  Serial.print(".");
  Serial.print(lv_version_minor());
  Serial.print(".");
  Serial.println(lv_version_patch());


  // Video + LVGL
  Display.begin();  // init GIGA Display
  lv_init();

  // The Display.begin() call already attaches LVGL internally
  Display.begin();

  // Adjust backlight / brightness if available
  Display.brightness(255);  // or Display.setBrightness(255);

  // ----- Apply dark background theme -----
  lv_obj_t* scr = lv_scr_act();
  lv_color_t bg_color = lv_color_hex(0x101215);    // deep grey-black
  lv_color_t text_color = lv_color_hex(0xE0E8FF);  // light bluish-white text
  lv_obj_set_style_bg_color(scr, bg_color, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_set_style_text_color(scr, text_color, 0);

  // Touch
  Touch.begin();  // Init GIGA Display touch
  ui_touch_attach(&Display, &Touch);

  // Periodic LVGL tick
  // Timer1.begin(lv_tick_cb, 5000);  // 5ms tick

  // Subsystems
  initParamStore();
  initActionStore();
  color_sensor_begin();
  dynamixel_begin();

  // UI
  ui_init();
  ui_set_title("Rubik Solver (GIGA Touch)");
  ui_show_main_menu();
}

void loop() {

  ui_loop_once();  // handles LVGL tasks + touch driver
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
