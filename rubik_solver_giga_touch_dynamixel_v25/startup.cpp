// ------------------ STARTUP DIAGNOSTICS ------------------
#include <Arduino.h>
#include "servos.h"
#include "ui_touch.h"      // for setFooter()
#include "ui_status.h"     // for updateButtonStateByKey()
#include "servo_manager.h"
#include "logging.h"

extern ServoManager servoMgr;

// ----------------------------------------------------------
//                 STARTUP TEST CONFIG
// ----------------------------------------------------------
static bool startupDone = false;
static bool startupOK = false;
static String startupErrors;

// ----------------------------------------------------------
//               RUN STARTUP TESTS
// ----------------------------------------------------------
// This runs once at boot, after dxl.begin() and ui_init()
// It pings all registered servos, updates their cached state,
// and builds a summary string (errors + counts).
bool runStartupTests() {
  if (startupDone) return startupOK;

  LOG_SECTION_START("startup tests");

  startupErrors = "";
  startupOK = true;
  int failCount = 0;
  int total = 0;

  for (auto& kv : servoMgr.getServoStore()) {  // we’ll add getServoMap() helper below
    const String& key = kv.first;
    total++;

    servoMgr.updateServo(key);
    if (servoMgr.hasServoErrors(key)) {
      startupOK = false;
      failCount++;
      String err = servoMgr.getErrorString(key);
      if (err.length()) {
        startupErrors += key + ": " + err + "\n";
      } else {
        startupErrors += key + ": unknown issue\n";
      }

      updateButtonStateByKey(key, true, false);  // show issue on UI
    } else {
      updateButtonStateByKey(key, false, false); // clear any old red overlays
    }
  }

  if (startupOK) {
    setFooter("startup test ok");
    LOG_VAR("startup status", "ok");
  } else {
    char buf[64];
    snprintf(buf, sizeof(buf), "startup: %d/%d failed", failCount, total);
    setFooter(buf);
    LOG_VAR("startup errors", buf);
  }

  startupDone = true;
  LOG_SECTION_END();
  return startupOK;
}

// ----------------------------------------------------------
//           RETRIEVE STARTUP TEST SUMMARY
// ----------------------------------------------------------
String getStartupTestErrorString() {
  if (startupOK) return "All servos responded OK.";
  return startupErrors.length() ? startupErrors : "Unknown startup failure.";
}

// ----------------------------------------------------------
//                HELPER FOR UI MODULES
// ----------------------------------------------------------
// Used by the “system” or “error_status” screen
// to display all servo info plus startup summary.
String getSystemDiagnosticString() {
  String out = "";
  out += "--- STARTUP STATUS ---\n";
  out += startupOK ? "All systems OK.\n\n" : "Errors detected:\n\n";
  out += getStartupTestErrorString();
  out += "\n\n";
  out += servoMgr.getFullDiagnosticString();
  return out;
}
