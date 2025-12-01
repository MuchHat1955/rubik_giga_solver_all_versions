#include <Adafruit_TCS34725.h>

// Global sensor instance
static Adafruit_TCS34725 tcs = Adafruit_TCS34725(
    TCS34725_INTEGRATIONTIME_50MS,
    TCS34725_GAIN_4X);

static bool tcs_initialized = false;

String classify_color(float r, float g, float b, float c) {
  // Normalize RGB against clear channel
  float rn = r / c;
  float gn = g / c;
  float bn = b / c;

  serial_printf("avg_norm: %.3f,%.3f,%.3f\n", rn, gn, bn);

  // VERY simple rules — tune as needed for cube
  if (rn > 0.55 && gn < 0.40 && bn < 0.35) return "R";
  if (gn > 0.55 && rn < 0.40 && bn < 0.40) return "G";
  if (bn > 0.55 && rn < 0.40 && gn < 0.40) return "B";

  // Yellow = high R + G
  if (rn > 0.48 && gn > 0.48 && bn < 0.30) return "Y";

  // Orange = red dominant but brighter
  if (rn > 0.60 && gn < 0.40 && bn < 0.30 && c > 150) return "O";

  // White = all high, similar
  if (fabs(rn-gn) < 0.08 && fabs(gn-bn) < 0.08 && rn > 0.40) return "W";

  return "X";
}

String read_color(uint8_t samples = 8) {

  if (!tcs_initialized) {
    if (!tcs.begin()) {
      serial_printf_verbose("ERR TCS34725 not found!");
      return "na";
    }
    tcs_initialized = true;
    LOG_INFO("TCS34725 initialized");
  }

  tcs.setInterrupt(false);   // LED ON
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

  tcs.setInterrupt(true);   // LED OFF

  float avg_r = sum_r / samples;
  float avg_g = sum_g / samples;
  float avg_b = sum_b / samples;
  float avg_c = sum_c / samples;

  serial_printf("avg_raw: %.1f,%.1f,%.1f,%.1f\n",
             avg_r, avg_g, avg_b, avg_c);

  return classify_color(avg_r, avg_g, avg_b, avg_c);
}
