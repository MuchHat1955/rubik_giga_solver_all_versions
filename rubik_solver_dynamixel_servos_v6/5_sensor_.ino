/********************************************************************
 * COLOR SENSOR HARDWARE
 ********************************************************************/

#include <Adafruit_TCS34725.h>

// -----------------------------------------------------
// CONFIGURATION
// -----------------------------------------------------

// Integration time: longer = more light collection, more delay
#define COLOR_INTEGRATION_TIME  TCS34725_INTEGRATIONTIME_154MS
#define COLOR_GAIN              TCS34725_GAIN_4X

// Thresholds for classification (tune based on your lighting)
#define MIN_VALID_CLEAR  100    // minimum brightness to accept a reading

Adafruit_TCS34725 tcs = Adafruit_TCS34725(COLOR_INTEGRATION_TIME, COLOR_GAIN);

// Optional: store recent readings for averaging
struct RGB {
  float r, g, b;
};

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
    Serial.println("❌ TCS34725 not found! Check wiring (SCL/SDA/3.3V/GND).");
    return false;
  }
}

// -----------------------------------------------------
// HELPER: RGB → HSV
// -----------------------------------------------------
void rgbToHSV(float r, float g, float b, float &h, float &s, float &v) {
  float maxVal = max(r, max(g, b));
  float minVal = min(r, min(g, b));
  v = maxVal;
  float delta = maxVal - minVal;
  if (maxVal != 0) s = delta / maxVal; else s = 0;
  if (s == 0) {
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
// HELPER: classify color into W/R/G/B/O/Y/X
// -----------------------------------------------------
char classifyColor(float r, float g, float b) {
  float h, s, v;
  rgbToHSV(r, g, b, h, s, v);

  // normalize v to 0–1
  v = v / 255.0;

  if (v < 0.15 || s < 0.10) return 'W'; // white (low sat or bright)

  if (h >= 0 && h < 20) return 'R';       // red
  if (h >= 20 && h < 45) return 'O';      // orange
  if (h >= 45 && h < 90) return 'Y';      // yellow
  if (h >= 90 && h < 160) return 'G';     // green
  if (h >= 160 && h < 250) return 'B';    // blue

  return 'X'; // unknown
}

// -----------------------------------------------------
// READ ONE COLOR SAMPLE
// -----------------------------------------------------
char readColorSensor() {
  uint16_t r_raw, g_raw, b_raw, c_raw;
  tcs.getRawData(&r_raw, &g_raw, &b_raw, &c_raw);

  if (c_raw < MIN_VALID_CLEAR) {
    Serial.println("⚠️ Low brightness - invalid reading");
    return 'X';
  }

  // normalize
  float r = (float)r_raw / c_raw * 255.0;
  float g = (float)g_raw / c_raw * 255.0;
  float b = (float)b_raw / c_raw * 255.0;

  char c = classifyColor(r, g, b);

  Serial.printf("Color raw R:%d G:%d B:%d C:%d -> RGB(%.0f,%.0f,%.0f) => %c\n",
                r_raw, g_raw, b_raw, c_raw, r, g, b, c);

  return c;
}

// -----------------------------------------------------
// SMOOTHED COLOR READ (AVERAGE OF N READS)
// -----------------------------------------------------
char readColorSmoothed(int samples = 3, int delayMs = 60) {
  float rSum = 0, gSum = 0, bSum = 0;
  for (int i = 0; i < samples; i++) {
    uint16_t r_raw, g_raw, b_raw, c_raw;
    tcs.getRawData(&r_raw, &g_raw, &b_raw, &c_raw);
    if (c_raw > MIN_VALID_CLEAR) {
      rSum += (float)r_raw / c_raw * 255.0;
      gSum += (float)g_raw / c_raw * 255.0;
      bSum += (float)b_raw / c_raw * 255.0;
    }
    delay(delayMs);
  }

  float r = rSum / samples;
  float g = gSum / samples;
  float b = bSum / samples;

  char c = classifyColor(r, g, b);
  Serial.printf("Averaged color -> RGB(%.0f,%.0f,%.0f) => %c\n", r, g, b, c);
  return c;
}

///////////////////////////////////////////////////////////////
#include <Adafruit_TCS34725.h>

// -----------------------------------------------------
// CONFIGURATION
// -----------------------------------------------------

// Integration time: longer = more light collection, more delay
#define COLOR_INTEGRATION_TIME  TCS34725_INTEGRATIONTIME_154MS
#define COLOR_GAIN              TCS34725_GAIN_4X

// Thresholds for classification (tune based on your lighting)
#define MIN_VALID_CLEAR  100    // minimum brightness to accept a reading

Adafruit_TCS34725 tcs = Adafruit_TCS34725(COLOR_INTEGRATION_TIME, COLOR_GAIN);

// Optional: store recent readings for averaging
struct RGB {
  float r, g, b;
};

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
    Serial.println("❌ TCS34725 not found! Check wiring (SCL/SDA/3.3V/GND).");
    return false;
  }
}

// -----------------------------------------------------
// HELPER: RGB → HSV
// -----------------------------------------------------
void rgbToHSV(float r, float g, float b, float &h, float &s, float &v) {
  float maxVal = max(r, max(g, b));
  float minVal = min(r, min(g, b));
  v = maxVal;
  float delta = maxVal - minVal;
  if (maxVal != 0) s = delta / maxVal; else s = 0;
  if (s == 0) {
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
// HELPER: classify color into W/R/G/B/O/Y/X
// -----------------------------------------------------
char classifyColor(float r, float g, float b) {
  float h, s, v;
  rgbToHSV(r, g, b, h, s, v);

  // normalize v to 0–1
  v = v / 255.0;

  if (v < 0.15 || s < 0.10) return 'W'; // white (low sat or bright)

  if (h >= 0 && h < 20) return 'R';       // red
  if (h >= 20 && h < 45) return 'O';      // orange
  if (h >= 45 && h < 90) return 'Y';      // yellow
  if (h >= 90 && h < 160) return 'G';     // green
  if (h >= 160 && h < 250) return 'B';    // blue

  return 'X'; // unknown
}

// -----------------------------------------------------
// READ ONE COLOR SAMPLE
// -----------------------------------------------------
char readColorSensor() {
  uint16_t r_raw, g_raw, b_raw, c_raw;
  tcs.getRawData(&r_raw, &g_raw, &b_raw, &c_raw);

  if (c_raw < MIN_VALID_CLEAR) {
    Serial.println("⚠️ Low brightness - invalid reading");
    return 'X';
  }

  // normalize
  float r = (float)r_raw / c_raw * 255.0;
  float g = (float)g_raw / c_raw * 255.0;
  float b = (float)b_raw / c_raw * 255.0;

  char c = classifyColor(r, g, b);

  Serial.printf("Color raw R:%d G:%d B:%d C:%d -> RGB(%.0f,%.0f,%.0f) => %c\n",
                r_raw, g_raw, b_raw, c_raw, r, g, b, c);

  return c;
}

// -----------------------------------------------------
// SMOOTHED COLOR READ (AVERAGE OF N READS)
// -----------------------------------------------------
char readColorSmoothed(int samples = 3, int delayMs = 60) {
  float rSum = 0, gSum = 0, bSum = 0;
  for (int i = 0; i < samples; i++) {
    uint16_t r_raw, g_raw, b_raw, c_raw;
    tcs.getRawData(&r_raw, &g_raw, &b_raw, &c_raw);
    if (c_raw > MIN_VALID_CLEAR) {
      rSum += (float)r_raw / c_raw * 255.0;
      gSum += (float)g_raw / c_raw * 255.0;
      bSum += (float)b_raw / c_raw * 255.0;
    }
    delay(delayMs);
  }

  float r = rSum / samples;
  float g = gSum / samples;
  float b = bSum / samples;

  char c = classifyColor(r, g, b);
  Serial.printf("Averaged color -> RGB(%.0f,%.0f,%.0f) => %c\n", r, g, b, c);
  return c;
}

///////////////////////////////////////////////////////////////
