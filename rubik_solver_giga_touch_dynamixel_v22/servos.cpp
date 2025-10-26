// ----------------------------------------------------------
//                      servos.cpp
// ----------------------------------------------------------
#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "logging.h"
#include "servos.h"

// ---- EDIT TO MATCH YOUR WIRING ----
#define DXL_SERIAL Serial1
#define DXL_DIR_PIN 10
#define DXL_BAUD 57600

// Global Dynamixel bus
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
bool hasDynamixel = false;

// ----------------------------------------------------------
//                  HARDWARE INITIALIZATION
// ----------------------------------------------------------
void dynamixel_begin() {
  LOG_SECTION_START("dynamixel_begin");

  DXL_SERIAL.begin(DXL_BAUD);
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(2.0);

  // Quick scan for any responding servo
  bool ok = false;
  for (int id = 1; id <= 20; id++) {
    if (dxl.ping(id)) {
      ok = true;
      LOG_VAR("first responsive ID", id);
      break;
    }
  }

  hasDynamixel = ok;
  LOG_VAR("status", ok ? "detected" : "none found");
  LOG_SECTION_END();
}
