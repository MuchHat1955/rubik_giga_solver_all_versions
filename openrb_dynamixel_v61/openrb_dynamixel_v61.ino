#include <Arduino.h>
#include "utils.h"
#include "log.h"
#include "servos.h"
#include "vertical_kinematics.h"
#include "movement.h"
#include "cmd_parser.h"
#include "color_sensor.h"
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <algorithm>

// -------------------------------------------------------------------
//                           SETUP
// -------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  init_tcs_led();

  dxl.begin(57600);
  dxl.setPortProtocolVersion(PROTOCOL);

  Serial.println("---- Dynamixel xl430 Controller v57ok -------------------------------------");
  Serial.println();

  // quick test for all servos
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    ServoConfig *s = all_servos[i];
    uint8_t id = s->get_id();

    if (dxl.ping(id)) {
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
      DEBUG_KV("key", s->get_key());
      DEBUG_KV("id", id);
      DEBUG_KV("cfg_min", cfg_min);
      DEBUG_KV("cfg_max", cfg_max);
      DEBUG_KV("hw_min", hw_min);
      DEBUG_KV("hw_max", hw_max);

    } else {
      DEBUG_INFO(MOD_CMD, "servo_not_responding");
      DEBUG_KV("key", s->get_key());
      DEBUG_KV("id", id);
    }
  }

  Serial.println();
  Serial.println(get_help_text());
  Serial.println();

  init_servo_limits();
  Serial.println();

  Serial.println("------------------ End Setup --------------------------------------------");
  Serial.println();
}

// -------------------------------------------------------------------
//                              LOOP
// -------------------------------------------------------------------

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  process_serial_command(line);
}
