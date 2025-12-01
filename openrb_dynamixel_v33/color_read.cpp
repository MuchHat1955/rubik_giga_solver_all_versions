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

ColorRef refs[] = {
  { "R", 0.514, 0.309, 0.274 },
  { "G", 0.327, 0.403, 0.353 },
  { "B", 0.331, 0.388, 0.540 },
  { "Y", 0.40, 0.40, 0.21 },
  { "W", 0.26, 0.34, 0.36 },
  { "O", 0.456, 0.297, 0.219 }
};

String classify_color(float r, float g, float b, float c) {
  float rn = r / c;
  float gn = g / c;
  float bn = b / c;

  serial_printf("avg_norm: %.3f,%.3f,%.3f\n", rn, gn, bn);

  float bestDist = 999.0;
  const char* best = "X";

  for (auto& ref : refs) {
    float dr = rn - ref.r;
    float dg = gn - ref.g;
    float db = bn - ref.b;

    float dist = dr * dr + dg * dg + db * db;

    if (dist < bestDist) {
      bestDist = dist;
      best = ref.name;
    }
  }

  // Optional: confidence check
  if (bestDist > 0.02)  // tune this threshold
    return "X";

  return String(best);
}


String read_color(uint8_t samples = 8) {

  serial_printf("start read color");

  if (!tcs_initialized) {
    serial_printf("start tcs begin");
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

  serial_printf("raw_samples:\n");

  for (uint8_t i = 0; i < samples; i++) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    serial_printf("%u: %u,%u,%u,%u\n", i, r, g, b, c);

    sum_r += r;
    sum_g += g;
    sum_b += b;
    sum_c += c;

    delay(10);
  }

  tcs.setInterrupt(true);  // LED OFF
  tcs_led_off();

  float avg_r = sum_r / samples;
  float avg_g = sum_g / samples;
  float avg_b = sum_b / samples;
  float avg_c = sum_c / samples;

  serial_printf("calling clasify color with: %.1f,%.1f,%.1f,%.1f\n",
                avg_r, avg_g, avg_b, avg_c);

  String ret = classify_color(avg_r, avg_g, avg_b, avg_c);

  serial_printf("final color: %s\n",
                ret.c_str());
  return ret;
}
