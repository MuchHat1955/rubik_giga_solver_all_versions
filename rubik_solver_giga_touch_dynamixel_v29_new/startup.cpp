// ------------------ STARTUP DIAGNOSTICS ------------------
#include <Arduino.h>
#include "ui_touch.h"   // for setFooter()
#include "ui_status.h"  // for updateButtonStateByKey()
#include "logging.h"
#include "rb_interface.h"

extern RBInterface rb;

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
  bool startupOK = true;
  int failCount = 0;
  int total = 0;

  startupOk = rb.updateInfo();

  if (startupOK) {
    setFooter("startup test ok");
    LOG_VAR("startup status", "ok");
  } else {
    setFooter(rb.getLastErrorLine());
  }

  startupDone = true;
  LOG_SECTION_END();
  return startupOK;
}

// ----------------------------------------------------------
//           RETRIEVE STARTUP TEST SUMMARY
// ----------------------------------------------------------
String getStartupTestErrorString() {
  if (startupOK) return "all servos responded OK.";
  return startupErrors.length() ? startupErrors : "unknown startup failure.";
}
