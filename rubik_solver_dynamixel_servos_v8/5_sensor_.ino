/********************************************************************
 * COLOR SENSOR HARDWARE
 ********************************************************************/

#include <Adafruit_TCS34725.h>

/********************************************************************
 * COLOR SENSOR MODULE (TCS34725)
 * Used by read.ino to detect cube facelet colors
 ********************************************************************/

#include <Adafruit_TCS34725.h>

// -----------------------------------------------------
// CONFIGURATION
// -----------------------------------------------------

// Integration time: longer = more light collection, slower read
#define COLOR_INTEGRATION_TIME  TCS34725_INTEGRATIONTIME_154MS
#define COLOR_GAIN              TCS34725_GAIN_4X

// Threshold for rejecting dark / invalid readings
#define MIN_VALID_CLEAR  100

// Global color sensor instance
Adafruit_TCS34725 tcs = Adafruit_TCS34725(COLOR_INTEGRATION_TIME, COLOR_GAIN);

// -----------------------------------------------------
// INITIALIZATION
// -----------------------------------------------------
bool initColorSensor() {
  if (tcs.begin()) {
    Serial.println("✅ TCS34725 initialized successfully.");
    tcs.setInterrupt(false);
    delay(100);
    return true;
  } else {
    Serial.println("❌ ERROR: TCS34725 not found. Check wiring (SCL/SDA/3.3V/GND).");
    return false;
  }
}

// -----------------------------------------------------
// HELPER: Convert RGB → HSV
// -----------------------------------------------------
static void rgbToHSV(float r, float g, float b, float &h, float &s, float &v) {
  float maxVal = max(r, max(g, b));
  float minVal = min(r, min(g, b));
  v = maxVal;
  float delta = maxVal - minVal;

  if (maxVal != 0)
    s = delta / maxVal;
  else {
    s = 0;
    h = 0;
    return;
  }

  if (r == maxVal)
    h = (g - b) / delta;
  else if (g == maxVal)
    h = 2 + (b - r) / delta;
  else
    h = 4 + (r - g) / delta;

  h *= 60;
  if (h < 0) h += 360;
}

// -----------------------------------------------------
// HELPER: Classify RGB → Cube Color (W,R,G,Y,O,B,X)
// -----------------------------------------------------
static char classifyColor(float r, float g, float b) {
  float h, s, v;
  rgbToHSV(r, g, b, h, s, v);

  // Normalize brightness to [0,1]
  v = v / 255.0;

  // Simple threshold rules; tune per lighting setup
  if (v < 0.15 || s < 0.10) return 'W';  // white (low saturation or bright)

  if (h >= 0 && h < 20)   return 'R';    // red
  if (h >= 20 && h < 45)  return 'O';    // orange
  if (h >= 45 && h < 90)  return 'Y';    // yellow
  if (h >= 90 && h < 160) return 'G';    // green
  if (h >= 160 && h < 250)return 'B';    // blue

  return 'X';  // unknown / invalid
}

// -----------------------------------------------------
// READ ONE COLOR SAMPLE
// -----------------------------------------------------
char readColorSensor() {
  uint16_t r_raw, g_raw, b_raw, c_raw;
  tcs.getRawData(&r_raw, &g_raw, &b_raw, &c_raw);

  if (c_raw < MIN_VALID_CLEAR) {
    Serial.println("⚠️  Low brightness — invalid reading");
    return 'X';
  }

  // Normalize to 0–255
  float r = (float)r_raw / c_raw * 255.0;
  float g = (float)g_raw / c_raw * 255.0;
  float b = (float)b_raw / c_raw * 255.0;

  char c = classifyColor(r, g, b);

  Serial.printf("→ Raw R:%d G:%d B:%d C:%d | RGB(%.0f,%.0f,%.0f) → %c\n",
                r_raw, g_raw, b_raw, c_raw, r, g, b, c);

  return c;
}

// -----------------------------------------------------
// SMOOTHED COLOR READ (AVERAGE OF N SAMPLES)
// -----------------------------------------------------
char readColorSmoothed(int samples = 3, int delayMs = 60) {
  float rSum = 0, gSum = 0, bSum = 0;
  int validSamples = 0;

  for (int i = 0; i < samples; i++) {
    uint16_t r_raw, g_raw, b_raw, c_raw;
    tcs.getRawData(&r_raw, &g_raw, &b_raw, &c_raw);

    if (c_raw > MIN_VALID_CLEAR) {
      rSum += (float)r_raw / c_raw * 255.0;
      gSum += (float)g_raw / c_raw * 255.0;
      bSum += (float)b_raw / c_raw * 255.0;
      validSamples++;
    }
    delay(delayMs);
  }

  if (validSamples == 0) {
    Serial.println("⚠️  All smoothed samples invalid");
    return 'X';
  }

  float r = rSum / validSamples;
  float g = gSum / validSamples;
  float b = bSum / validSamples;

  char c = classifyColor(r, g, b);
  Serial.printf("Averaged RGB(%.0f,%.0f,%.0f) → %c\n", r, g, b, c);

  return c;
}

