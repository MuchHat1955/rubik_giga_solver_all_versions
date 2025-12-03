#include <Adafruit_TCS34725.h>
#include "utils.h"

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
  { "R", 0.481, 0.240, 0.275 },
  { "O", 0.492, 0.254, 0.212 },
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

  serial_printf_verbose("avg_norm: %.3f,%.3f,%.3f\n", rn, gn, bn);

  float bestDist = 999.0;
  const char* best = "X";

  for (auto& ref : refs) {
    float dr = rn - ref.r;
    float dg = gn - ref.g;
    float db = bn - ref.b;

    float dist = dr*dr + dg*dg + db*db;

    serial_printf_verbose("  compare %s => dist=%.4f\n", ref.name, dist);

    if (dist < bestDist) {
      bestDist = dist;
      best = ref.name;
    }
  }

  serial_printf_verbose("bestDist = %.4f\n", bestDist);

  // Confidence gate — tune if needed, 0.02 works well
  if (bestDist > 0.02)
    return "X";

  return String(best);
}

String read_color(uint8_t samples = 8) {

  serial_printf_verbose("start read color\n");

  if (!tcs_initialized) {
    serial_printf_verbose("start tcs begin");
    if (!tcs.begin()) {
      serial_printf_verbose("ERR TCS34725 not found!");
      return "na";
    }
    tcs_initialized = true;
    serial_printf_verbose("TCS34725 initialized");
  }

  tcs.setInterrupt(false);  // LED ON
  tcs_led_on();
  delay(10);

  float sum_r = 0, sum_g = 0, sum_b = 0, sum_c = 0;

  serial_printf_verbose("raw_samples:\n");

  // skip reading 0
  for (uint8_t i = 0; i <= samples; i++) {
    uint16_t r, g, b, c;
    delay(20);
    tcs.getRawData(&r, &g, &b, &c);
    if (i == 0) continue;

    serial_printf_verbose("%u: %u,%u,%u,%u\n", i, r, g, b, c);

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

  serial_printf_verbose("calling clasify color with: %.1f,%.1f,%.1f,%.1f\n",
                avg_r, avg_g, avg_b, avg_c);

  String ret = classify_color(avg_r, avg_g, avg_b, avg_c);

  serial_printf_verbose("final color: %s\n",
                ret.c_str());
  return ret;
}
