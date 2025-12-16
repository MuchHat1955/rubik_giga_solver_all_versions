#include <Adafruit_TCS34725.h>
#include "color_sensor.h"
#include "utils.h"
#include "log.h"

// Global sensor instance
static Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X);

static bool tcs_initialized = false;

#define TCS_LED_PIN 10  // choose any free digital pin

void init_tcs_led() {
  pinMode(TCS_LED_PIN, OUTPUT);
  analogWrite(TCS_LED_PIN, 0);  // LED OFF (active LOW)
}

void tcs_led_on() {
  analogWrite(TCS_LED_PIN, 128);  // LED ON
}

void tcs_led_off() {
  analogWrite(TCS_LED_PIN, 0);  // LED OFF
}

struct ColorRef {
  const char* name;
  float r, g, b;
};

// NEW calibrated values (your real cube at 2 cm + tube)
ColorRef refs[] = {
  { "R", 0.508, 0.241, 0.241 },
  { "O", 0.510, 0.255, 0.216 },
  { "Y", 0.387, 0.367, 0.206 },
  { "G", 0.250, 0.412, 0.294 },
  { "B", 0.194, 0.290, 0.452 },
  { "W", 0.301, 0.323, 0.339 }
};

String classify_color(float r, float g, float b, float c) {

  // Normalize
  float rn = r / c;
  float gn = g / c;
  float bn = b / c;

  DEBUG_INFO(MOD_COLORSENSOR, "avg_norm");
  DEBUG_KV("r", rn);
  DEBUG_KV("g", gn);
  DEBUG_KV("b", bn);

  float bestDist = 999.0;
  const char* best = "X";

  for (auto& ref : refs) {

    float dr = rn - ref.r;
    float dg = gn - ref.g;
    float db = bn - ref.b;

    float dist = dr * dr + dg * dg + db * db;

    // ---- FIX 2: ORANGE BIAS LOGIC ----
    if (strcmp(ref.name, "R") == 0) {
      float green_blue_ratio = (bn > 0.001) ? (gn / bn) : 1.0;
      if (green_blue_ratio > 1.05) {
        dist += 0.0004;
      }
    }
    // ----------------------------------

    DEBUG_INFO(MOD_COLORSENSOR, "compare_ref");
    DEBUG_KV("ref", ref.name);
    DEBUG_KV("dist", dist);

    if (dist < bestDist) {
      bestDist = dist;
      best = ref.name;
    }
  }

  DEBUG_INFO(MOD_COLORSENSOR, "best_distance");
  DEBUG_KV("dist", bestDist);

  if (bestDist > 0.02)
    return "X";

  return String(best);
}

String read_color() {

  uint8_t samples = 8;

  DEBUG_INFO(MOD_COLORSENSOR, "start_read_color");

  if (!tcs_initialized) {
    DEBUG_INFO(MOD_COLORSENSOR, "tcs_begin");
    if (!tcs.begin()) {
      DEBUG_ERR(MOD_COLORSENSOR, "tcs_not_found");
      return "na";
    }
    tcs_initialized = true;
    DEBUG_INFO(MOD_COLORSENSOR, "tcs_initialized");
  }

  tcs.setInterrupt(false);  // LED ON
  tcs_led_on();
  delay(10);

  float sum_r = 0, sum_g = 0, sum_b = 0, sum_c = 0;

  DEBUG_INFO(MOD_COLORSENSOR, "raw_samples");

  // skip reading 0
  for (uint8_t i = 0; i <= samples; i++) {
    uint16_t r, g, b, c;
    delay(20);
    tcs.getRawData(&r, &g, &b, &c);
    if (i == 0) continue;

    DEBUG_INFO(MOD_COLORSENSOR, "raw_sample");
    DEBUG_KV("i", i);
    DEBUG_KV("r", r);
    DEBUG_KV("g", g);
    DEBUG_KV("b", b);
    DEBUG_KV("c", c);

    sum_r += r;
    sum_g += g;
    sum_b += b;
    sum_c += c;
  }

  tcs.setInterrupt(true);  // LED OFF
  tcs_led_off();

  float avg_r = sum_r / samples;
  float avg_g = sum_g / samples;
  float avg_b = sum_b / samples;
  float avg_c = sum_c / samples;

  DEBUG_INFO(MOD_COLORSENSOR, "avg_raw");
  DEBUG_KV("r", avg_r);
  DEBUG_KV("g", avg_g);
  DEBUG_KV("b", avg_b);
  DEBUG_KV("c", avg_c);

  String ret = classify_color(avg_r, avg_g, avg_b, avg_c);

  DEBUG_INFO(MOD_COLORSENSOR, "final_color");
  DEBUG_KV("color", ret.c_str());

  return ret;
}
