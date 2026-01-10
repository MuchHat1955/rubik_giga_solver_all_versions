// ------------------ STARTUP DIAGNOSTICS ------------------
#include <arduino.h>

#include "ui_touch.h"   // for setFooter()
#include "ui_status.h"  // for updateButtonStateByKey()
#include "logging.h"
#include "rb_interface.h"

extern RBInterface rb;

bool runRbStartupTests();
bool runSolverStartupTests();
bool solver_begin();

// ----------------------------------------------------------
//                 STARTUP TEST CONFIG
// ----------------------------------------------------------
static bool rb_begin_ok = false;
static bool solver_begin_ok = false;
static bool rb_startup_tests_ok = false;
static bool solver_startup_tests_ok = false;

// ----------------------------------------------------------
//               RUN STARTUP TESTS
// ----------------------------------------------------------
// This runs once at boot, after dxl.begin() and ui_init()
// It pings all registered servos, updates their cached state,
// and builds a summary string (errors + counts).
bool runStartupTests(bool force_rerun) {

  if (!force_rerun &&                                    //
      rb_startup_tests_ok && solver_startup_tests_ok &&  //
      rb_begin_ok && solver_begin_ok) {
    LOG_PRINTF_RUN("startup tests skipped | already run and ok\n");
    setFooter("startup tests | rb ok | solver ok", _DONE_SUCCESS);
    __delay(666);
    return true;
  }

  if (force_rerun) {
    rb_startup_tests_ok = false;
    solver_startup_tests_ok = false;
    rb_begin_ok = false;
    solver_begin_ok = false;
  }

  // rerun begin if needed
  if (!rb_begin_ok || !solver_begin_ok || force_rerun) {
    // start rb servos serial
    if (!rb_begin_ok || force_rerun) {
      setFooter("starting... rb", _RUNNING_NOSTOP);
      rb_begin_ok = rb.begin();
      if (rb_begin_ok) __delay(666);
    }
    // start teensy cube solver serial
    if (!solver_begin_ok || force_rerun) {
      setFooter("starting... solver", _RUNNING_NOSTOP);
      solver_begin_ok = solver_begin();
      if (solver_begin_ok) __delay(666);
    }

    if (!rb_begin_ok) LOG_ERR("[RB] error=rb_serial_failed\n");
    if (!solver_begin_ok) LOG_ERR("[SOLVER] error=solver_serial_failed\n");

    if (rb_begin_ok) LOG_PRINTF_RB("rb_serial_ok\n");
    if (solver_begin_ok) LOG_PRINTF_SOLVER("solver_serial_ok\n");

    String begin_result = "serials | ";
    if (rb_begin_ok) begin_result += "rb ok";
    else begin_result += "rb failed";
    begin_result += " | ";
    if (solver_begin_ok) begin_result += "solver ok";
    else begin_result += "solver failed";

    if (!rb_begin_ok || !solver_begin_ok) setFooter(begin_result.c_str(), _DONE_ERROR);
    else setFooter(begin_result.c_str(), _DONE_SUCCESS);
    __delay(666);
  }

  // run tests only if both begin are ok
  if (rb_begin_ok && solver_begin_ok) {
    if (!rb_startup_tests_ok || force_rerun) {
      setFooter("running... rb tests", _RUNNING_NOSTOP);
      rb_startup_tests_ok = runRbStartupTests();
    }
    if (!solver_startup_tests_ok || force_rerun) {
      setFooter("running... solver tests", _RUNNING_NOSTOP);
      solver_startup_tests_ok = runSolverStartupTests();
    }

    if (!rb_startup_tests_ok) LOG_ERR("[RB] error=rb_startup_tests_failed\n");
    if (!solver_startup_tests_ok) LOG_ERR("[SOLVER] error=solver_startup_tests_failed\n");

    String startup_tests_result = "startup tests | ";
    if (rb_startup_tests_ok) startup_tests_result += "rb ok";
    else startup_tests_result += "rb failed";
    startup_tests_result += " | ";
    if (solver_startup_tests_ok) startup_tests_result += "solver ok";
    else startup_tests_result += "solver failed";

    if (!rb_startup_tests_ok || !solver_startup_tests_ok) setFooter(startup_tests_result.c_str(), _DONE_ERROR);
    else setFooter(startup_tests_result.c_str(), _DONE_SUCCESS);
    __delay(666);
  }
  return rb_startup_tests_ok && solver_startup_tests_ok;
}


bool runRbStartupTests() {
  LOG_PRINTF("---- start rb startup run tests\n");

  int cmd_id = 0;
  bool tests_ok = runCommand("READSERVO", "0", &cmd_id);

  if (tests_ok) {
    LOG_PRINTF("startup status ok\n");
  } else {
    setFooter(getLastError(cmd_id).c_str(), _DONE_ERROR);
  }
  LOG_PRINTF("---- end rb startup run tests\n");
  return tests_ok;
}

bool runSolverStartupTests() {
  //TODO->TO IMPLEMENT add a startup test for solver, send a test cube

  LOG_PRINTF("---- start solver startup run tests\n");
  bool tests_ok = true;

  /*/
  int cmd_id = 0;
  bool tests_ok = runCommand("INFOSERVOS", "0", &cmd_id);

  if (tests_ok) {
    LOG_PRINTF("startup status ok\n");
  } else {
    setFooter(getLastError(cmd_id).c_str(), _DONE_ERROR);
  }
  */
  LOG_PRINTF("---- end solver startup run tests\n");
  return tests_ok;
}
