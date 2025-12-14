#include <Arduino.h>
#include "utils.h"
#include "servos.h"
#include "vertical_kinematics.h"
#include "movement.h"
#include "cmd_parser.h"
#include "color_sensor.h"
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <algorithm>

// -------------------------------------------------------------------
//                           GLOBAL FLAGS
// -------------------------------------------------------------------

bool verboseOn = false;  // default at boot = ON

extern uint32_t start_ms;

// -------------------------------------------------------------------
//                           SETUP
// -------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  unsigned long millis_wait = 6666;
  while (!Serial && millis() < millis_wait) {
    delay(2222);
  }
  delay(1111);
  Serial.println("---- Serial Started v59 ----------------------------");
  Serial.println();
    delay(33333);

  init_tcs_led();

  do {
    if (Serial) {
      dxl.begin(57600);
      dxl.setPortProtocolVersion(PROTOCOL);
      break;
    }
    Serial.println("---- Waiting on Serial to settle before starting dxl");
    delay(1111);
  } while (true);

  Serial.println("---- RB Dynamixel xl430 Started --------------------");
  Serial.println();
  delay(111);

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

      serial_printf_verbose("Servo %s (id=%u) OK | cfg[%u-%u] hw[%u-%u]\n",
                            s->get_key(), id, cfg_min, cfg_max, hw_min, hw_max);
    } else {
      serial_printf_verbose("Servo %s (id=%u) NOT RESPONDING\n", s->get_key(), id);
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
  if (!Serial) return;
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  process_serial_command(line);
}
