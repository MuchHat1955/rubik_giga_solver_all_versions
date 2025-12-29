// ------------------ STARTUP DIAGNOSTICS ------------------
#include <arduino.h>

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

// ----------------------------------------------------------
//               RUN STARTUP TESTS
// ----------------------------------------------------------
// This runs once at boot, after dxl.begin() and ui_init()
// It pings all registered servos, updates their cached state,
// and builds a summary string (errors + counts).
bool runStartupTests() {
  if (startupDone) return startupOK;

  LOG_PRINTF("---- start startup run tests\n");

  int failCount = 0;
  int total = 0;

  int cmd_id=0;
  startupOK = runCommand("READSERVO", "0", &cmd_id);
  startupOK = true;

  if (startupOK) {
    setFooter("startup test ok");
    LOG_PRINTF("startup status ok\n");
  } else {
    setFooter(getLastError(cmd_id).c_str());
  }

  startupDone = true;
  LOG_PRINTF("---- end startup run tests\n");
  return startupOK;
}

// ----------------------------------------------------------
//           RETRIEVE STARTUP TEST SUMMARY
// ----------------------------------------------------------
String getStartupTestErrorString() {
  if (startupOK) return "all servos responded OK.";
  return getAllErrorLines();
}
