// ------------------ STARTUP DIAGNOSTICS ------------------
#include <arduino.h>

#include "ui_touch.h"   // for setFooter()
#include "ui_status.h"  // for updateButtonStateByKey()
#include "logging.h"
#include "rb_interface.h"
#include "rubik_solver.h"

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
      setFooter("start... rb", _RUNNING_NOSTOP);
      rb_begin_ok = rb.begin();
      if (rb_begin_ok) __delay(666);
    }
    // start teensy cube solver serial
    if (!solver_begin_ok || force_rerun) {
      setFooter("start... solver", _RUNNING_NOSTOP);
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
    __delay(1333);
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
    __delay(1333);
  }
  return rb_startup_tests_ok && solver_startup_tests_ok;
}


bool runRbStartupTests() {
  LOG_PRINTF_RUN("start... rb startup run tests\n");

  int cmd_id = 0;
  bool tests_ok = runCommand("READSERVO", "0", &cmd_id);

  if (tests_ok) {
    LOG_PRINTF_RUN("startup status ok\n");
  } else {
    setFooter(getLastError(cmd_id).c_str(), _DONE_ERROR);
  }
  LOG_PRINTF_RUN("end... rb startup run tests\n");
  return tests_ok;
}

String test_colors54 = "DRLUUBFBRBLURRLRUBLRDDFDLFUFUFFDBRDUBRUFLLFDDBFLUBLRBD";

bool runSolverStartupTests() {

  String test_solution_found = "";
  int solution_move_count = 0;
  int solution_time_to_compute = 0;

  LOG_PRINTF_SOLVER("start... solver test solution\n");
  setFooter("run... solver test", _RUNNING_NOSTOP);

  // the actual test
  bool tests_ok = solver_find_solution(test_colors54,
                                       test_solution_found,
                                       solution_move_count,
                                       solution_time_to_compute);

  if (tests_ok) {
    String footer_text = "test solution found " +                        //
                         String(solution_move_count) + " moves " +  //
                         String(solution_time_to_compute) + "ms";
    LOG_PRINTF_SOLVER("solver test ok %s\n", footer_text.c_str());
    setFooter(footer_text.c_str(), _DONE_SUCCESS);
    __delay(1333);
  } else {
    String footer_text = "test solution not found returned " +           //
                         String(solution_move_count) + " moves " +  //
                         String(solution_time_to_compute) + "ms";
    LOG_PRINTF_SOLVER("solver test failed %s\n", footer_text.c_str());
    setFooter(footer_text.c_str(), _DONE_ERROR);
    __delay(1333);
  }

  LOG_PRINTF_RUN("end... solver startup run tests } result {%d}\n", tests_ok);
  return tests_ok;
}
