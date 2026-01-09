/*
HARDWARE IS OpenRB-150
to debug recompile with serial instead of serial1

// TODO change below in utils.h for debugging vs normal usage
#define __serial Serial // logs in IDE for debugging
#define __serial Serial2 // normal usage, nothing shows in IDE
*/

#include <Arduino.h>
#include "utils.h"
#include "log.h"
#include "servos.h"
#include "vertical_kinematics.h"
#include "servo_move.h"
#include "cmd_parser.h"
#include "color_sensor.h"
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <algorithm>

bool serial_ok = false;
String version_str = "na";
String protocol_str = "protocol v1";

// -------------------------------------------------------------------
//                           HELPER for version
// -------------------------------------------------------------------

void serial_print_lowercase(const char *s) {
  while (*s) {
    char c = *s++;
    if (c >= 'A' && c <= 'Z') c = c - 'A' + 'a';
    __serial.print(c);
  }
}

const char *extract_version_from_file() {
  static char ver[8] = "v?";
  const char *f = __FILE__;

  for (const char *p = f; *p; ++p) {
    if (*p == 'v' && isdigit(*(p + 1))) {
      ver[0] = 'v';
      ver[1] = *(p + 1);
      int i = 2;
      p += 2;
      while (isdigit(*p) && i < (int)sizeof(ver) - 1) {
        ver[i++] = *p++;
      }
      ver[i] = '\0';
      return ver;
    }
  }
  return ver;
}

void print_build_banner() {
  __serial.print("----- dynamixel servos controller ");
  version_str = String(extract_version_from_file());
  version_str += " | built ";
  version_str += String(__DATE__);
  version_str += " ";
  version_str += String(__TIME__);  // already numeric, no case
  version_str += " | ";
  version_str += protocol_str;
  version_str.toLowerCase();
  __serial.print(version_str.c_str());
  __serial.println(" -----");
}

// -------------------------------------------------------------------
//                           SETUP
// -------------------------------------------------------------------

void setup() {
  Serial.begin(115300);
  delay(555);
  Serial.println("setup started");

  __serial.begin(115200);
  unsigned long end_millis = millis() + 11000;
  while (!__serial && millis() < end_millis) {
    delay(11);
  }
  if (__serial) serial_ok = true;
  delay(666);

  init_color_sensor();

  dxl.begin(57600);
  dxl.setPortProtocolVersion(PROTOCOL);

  // quick test for all servos
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    ServoConfig *s = all_servos[i];
    uint8_t id = s->get_id();

    if (dxl_ping_cached(id)) {
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_POSITION);
      dxl.torqueOn(id);

      // Read current limits from servo
      uint16_t hw_min = dxl.readControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id);
      uint16_t hw_max = dxl.readControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id);

      // Read configured limits from ServoConfig
      uint16_t cfg_min = s->min_ticks();
      uint16_t cfg_max = s->max_ticks();

      DEBUG_INFO(MOD_CMD, "servo_ok");
      DEBUG_VAR("key", s->get_key());
      DEBUG_VAR("id", id);
      DEBUG_VAR("cfg_min", cfg_min);
      DEBUG_VAR("cfg_max", cfg_max);
      DEBUG_VAR("hw_min", hw_min);
      DEBUG_VAR("hw_max", hw_max);

    } else {
      DEBUG_INFO(MOD_CMD, "servo_not_responding");
      DEBUG_VAR("key", s->get_key());
      DEBUG_VAR("id", id);
    }
  }

  __serial.println();
  print_build_banner();
  __serial.println(get_help_text());
  __serial.println("----------------------------------------------------------------------------");

  init_servo_limits();
  __serial.println();

  Serial.println("setup end");
}

// -------------------------------------------------------------------
//                              LOOP
// -------------------------------------------------------------------

void loop() {
  serial_line.poll();

  if (serial_line.count() < 1) return;

  const char *line = serial_line.read(0);
  Serial.print("loop line received=");
  Serial.println(line);
  if (!line) return;

  process_serial_command(line);
}
