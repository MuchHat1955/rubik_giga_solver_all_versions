/********************************************************************
 * Rubik Cube Solver UI (ESP32 + ST7796S + Encoder)
 ********************************************************************/

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7796S_kbv.h>
#include <Encoder.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>
#include <Preferences.h>
#include <Dynamixel2Arduino.h>
#include <Adafruit_TCS34725.h>

// ---------- Pins (adjust as needed) ----------
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 17

#define ENC_A 18
#define ENC_B 19
#define ENC_BTN 21

Adafruit_ST7796S_kbv tft(TFT_CS, TFT_DC, TFT_RST);
Encoder rotary(ENC_A, ENC_B);

// ---------- Dynamixel (stubs/ready for later) ----------
#define DXL_SERIAL Serial2
#define DXL_DIR_PIN 15
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ---------- Preferences (stubs/ready for later) ----------
Preferences preferences;

// ---------- Visual config ----------
#define TITLE_COLOR ST7796S_WHITE
#define SUBTITLE_COLOR ST7796S_YELLOW
#define ROW_TEXT_COLOR ST7796S_WHITE
#define HILIGHT_COLOR ST7796S_CYAN
#define SELECT_COLOR ST7796S_CYAN
#define BG_COLOR ST7796S_BLACK
#define BOX_FILL_COLOR ST7796S_BLACK
#define BOX_FRAME_COLOR ST7796S_DARKGREY
#define ACTIVE_DOT_COLOR ST7796S_GREEN
#define INACTIVE_DOT_COLOR ST7796S_DARKGREY
#define PRESSED_SHADE ST7796S_NAVY

#define TITLE_SIZE 3
#define SUBTITLE_SIZE 2
#define ROW_TEXT_SIZE 2

static const int VISIBLE_ROWS = 3;
static const int ROW_H = 44;    // per-row height
static const int CELL_PAD = 6;  // inner pad
static const int CORNER_R = 6;  // round rect radius

// ---------- color sensor ----------

// Integration time: longer = more light collection, more delay
#define COLOR_INTEGRATION_TIME TCS34725_INTEGRATIONTIME_154MS
#define COLOR_GAIN TCS34725_GAIN_4X

// Thresholds for classification (tune based on your lighting)
#define MIN_VALID_CLEAR 100  // minimum brightness to accept a reading

Adafruit_TCS34725 tcs = Adafruit_TCS34725(COLOR_INTEGRATION_TIME, COLOR_GAIN);

// ---------- Forward declarations ----------
void moveServoSmooth(int id, int target, int minv, int maxv);
class CubeOrientationTracker;
class RubikCubeColors;

// ---------- Global vars ----------
CubeOrientationTracker cubeOri;
RubikCubeColors cubeColors;

/********************************************************************
 * Setup / Loop
 ********************************************************************/

// ---------- setup ----------
void setup() {
  Serial.begin(115200);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(BG_COLOR);

  pinMode(ENC_BTN, INPUT_PULLUP);

  // parse JSON
  DeserializationError err = deserializeJson(doc, menu_json);
  if (err) {
    Serial.print("JSON error: ");
    Serial.println(err.c_str());
    while (1) delay(1000);
  }

  initParamStore();
  loadParamsOnce();
  initActionStore();
  initColorSensor();

  // open landing page
  JsonObject root = doc["root"].as<JsonObject>();
  current_page = root["screen"].as<JsonObject>();
  page_stack.clear();
  page_stack.push_back(current_page);

  rebuildSelectableList();
  drawMenuPage(current_page);

  // encoder baseline
  lastEnc = rotary.read() / encoderDiv;
}

// ---------- loop ----------
void loop() {
  long enc = rotary.read() / encoderDiv;
  long delta = enc - lastEnc;
  if (delta != 0) {
    applyEncoderDelta(delta);
    lastEnc = enc;
  }

  bool btn = (digitalRead(ENC_BTN) == LOW);
  if (btn && !lastBtnState) { /* ignore bounce */
  }
  if (btn && lastBtnState == HIGH) {
    handleButtonPress();
  }
  lastBtnState = btn;

  delay(10);
}
