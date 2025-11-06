#include "rb_interface.h"
#include "logging.h"

// ============================================================
// Constructor
// ============================================================
RBInterface::RBInterface(Stream& port)
  : serial(port) {}

// ============================================================
// BEGIN - initialize link and disable verbose
// ============================================================
bool RBInterface::begin(unsigned long baud, uint32_t timeout_ms) {
  LOG_SECTION_START_PRINTF("begin", "| baud=%lu timeout=%lu", baud, timeout_ms);

  serial.begin(baud);
  serial.setTimeout(timeout_ms);
  clearErrorBuffer();

  LOG_PRINTF("Initializing communication...");
  delay(300);

  // Turn verbose OFF to get clean command responses
  serial.println("VERBOSEOFF");
  delay(100);

  // Flush any startup noise
  while (serial.available()) serial.read();

  // Confirm RB responds
  serial.println("READ 0");
  unsigned long t0 = millis();
  bool got = false;
  while (millis() - t0 < 2000) {
    if (!serial.available()) continue;
    String line = serial.readStringUntil('\n');
    if (line.startsWith("STATUS")) {
      got = true;
      break;
    }
  }

  if (!got) {
    LOG_PRINTF("⚠ No response from RB board!");
    LOG_SECTION_END();
    return false;
  }

  LOG_PRINTF("Communication OK, reading servo INFO...");

  // Query INFO for all servos (11–17)
  uint8_t ids[] = { ID_ARM1, ID_ARM2, ID_WRIST, ID_GRIP1, ID_GRIP2, ID_BASE, ID_XM };
  for (uint8_t i = 0; i < sizeof(ids); i++) {
    requestServoInfo(ids[i]);
    delay(40);
  }

  LOG_PRINTF("Initialization complete, verbose OFF.");
  LOG_SECTION_END();
  return true;
}

// ============================================================
// Generic command
// ============================================================
bool RBInterface::runCommand(const char* name, const float* args, int argCount) {
  LOG_SECTION_START_PRINTF("runCommand", "| cmd=%s argc=%d", name, argCount);

  clearErrorBuffer();

  String cmd(name);
  for (int i = 0; i < argCount; i++) {
    cmd += " ";
    cmd += String(args[i], 3);
  }
  serial.println(cmd);

  LOG_PRINTF("[GIGA→RB] %s", cmd.c_str());
  bool ok = waitForCompletion(name);

  LOG_PRINTF("Command %s result=%s", name, ok ? "OK" : "FAIL");
  LOG_SECTION_END();
  return ok;
}

// ============================================================
// INFO fetch helper
// ============================================================
bool RBInterface::requestServoInfo(uint8_t id) {
  LOG_SECTION_START_PRINTF("requestServoInfo", "| id=%d", id);

  String cmd = "INFO ";
  cmd += String(id);
  serial.println(cmd);

  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    if (!serial.available()) continue;
    String line = serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("INFO id=")) {
      LOG_PRINTF("%s", line.c_str());
      LOG_SECTION_END();
      return true;
    }
  }
  LOG_PRINTF("⚠ No INFO response for ID %d", id);
  LOG_SECTION_END();
  return false;
}

// ============================================================
// Parsing and verification
// ============================================================
void RBInterface::parseStatusLine(const String& line) {
  if (line.startsWith("STATUS SERVO")) {
    if (verboseOn) LOG_PRINTF("%s", line.c_str());
    return;
  }

  int idx = line.indexOf(' ');
  if (idx < 0) return;
  String rest = line.substring(idx + 1);

  double val;
  for (int p = 0; p < rest.length();) {
    int eq = rest.indexOf('=', p);
    if (eq < 0) break;
    int sp = rest.indexOf(' ', eq);
    if (sp < 0) sp = rest.length();
    String k = rest.substring(p, eq);
    String v = rest.substring(eq + 1, sp);
    val = v.toDouble();

    if (k == "x_mm") last.x_mm = val;
    else if (k == "y_mm") last.y_mm = val;
    else if (k == "a1_deg") last.a1_deg = val;
    else if (k == "a2_deg") last.a2_deg = val;
    else if (k == "g_vert_deg") last.g_vert_deg = val;
    else if (k == "g1_per") last.g1_per = val;
    else if (k == "g2_per") last.g2_per = val;
    else if (k == "base_deg") last.base_deg = val;
    else if (k == "completed") last.completed = (int)val;

    p = sp + 1;
  }
}

// ============================================================
// Stub: updateFooter()
// Called whenever a "MOVING ..." line is received from RB.
// You can later update the display, GUI, or telemetry here.
// ============================================================
void RBInterface::updateFooter(const char* text) {
  // TODO: Implement display or UI footer update
  LOG_PRINTF("[FOOTER] %s\n", text);
}


// ============================================================
// Wait for START/END/ERR sequence
// ============================================================
bool RBInterface::waitForCompletion(const char* commandName) {
  LOG_SECTION_START_PRINTF("waitForCompletion", "| cmd=%s", commandName);

  String startMarker = String(commandName) + " START";
  String endMarker   = String(commandName) + " END";
  unsigned long t0   = millis();
  bool success = false;

  while (millis() - t0 < 8000) {
    if (!serial.available()) continue;
    String line = serial.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) continue;

    // --- Handle ERR lines ---
    if (line.startsWith("ERR")) {
      errorLines.push_back(line);
      LOG_PRINTF("%s", line.c_str());
      continue;
    }

    // --- Handle progress lines ---
    // Any line starting with "MOVING" is considered progress feedback
    if (line.startsWith("MOVING")) {
      if (verboseOn) LOG_PRINTF("%s", line.c_str());
      updateFooter(line.c_str());  // NEW HOOK
      continue;
    }

    // --- Handle START marker ---
    if (line.startsWith(startMarker)) {
      parseStatusLine(line);
      LOG_PRINTF("%s started", commandName);
      continue;
    }

    // --- Handle END marker ---
    if (line.startsWith(endMarker)) {
      parseStatusLine(line);
      LOG_PRINTF("%s ended completed=%d", commandName, last.completed);
      success = (last.completed == 1);
      verifyExpected(commandName);
      break;
    }
  }

  if (!success) {
    errorLines.push_back("ERR: Timeout or incomplete command");
    LOG_PRINTF("⚠ Timeout or incomplete command");
  }

  LOG_SECTION_END();
  return success;
}

// ============================================================
// Verify expected final status for MOVE commands
// ============================================================
void RBInterface::verifyExpected(const char* cmd) {
  LOG_SECTION_START_PRINTF("verifyExpected", "| cmd=%s", cmd);

  if (strncmp(cmd, "MOVEYMM", 7) == 0)
    LOG_PRINTF("y_mm=%.2f", last.y_mm);
  if (strncmp(cmd, "MOVEXMM", 7) == 0)
    LOG_PRINTF("x_mm=%.2f", last.x_mm);
  if (strncmp(cmd, "MOVEGRIPPER", 11) == 0)
    LOG_PRINTF("grip1=%.2f grip2=%.2f", last.g1_per, last.g2_per);

  LOG_SECTION_END();
}

// ============================================================
// UpdateInfo - force RB to send current status (READ 0)
// ============================================================
bool RBInterface::updateInfo() {
  LOG_SECTION_START("updateInfo");

  clearErrorBuffer();
  serial.println("READ 0");
  LOG_PRINTF("Requesting READ 0...");

  unsigned long t0 = millis();
  bool gotAny = false;

  while (millis() - t0 < 2000) {
    if (!serial.available()) continue;

    String line = serial.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) continue;

    if (line.startsWith("ERR")) {
      errorLines.push_back(line);
      LOG_PRINTF("%s", line.c_str());
      continue;
    }

    if (line.startsWith("STATUS SERVO")) {
      gotAny = true;
      parseStatusLine(line);
      continue;
    }

    if (line.startsWith("READ END") || line.indexOf("x_mm=") > 0) {
      parseStatusLine(line);
      gotAny = true;
    }
  }

  if (!gotAny) {
    errorLines.push_back("ERR: No READ 0 response received");
    LOG_PRINTF("⚠ No response for READ 0");
    LOG_SECTION_END();
    return false;
  }

  LOG_PRINTF("READ 0 done | X=%.2f Y=%.2f A1=%.2f A2=%.2f G=%.2f",
             last.x_mm, last.y_mm, last.a1_deg, last.a2_deg, last.g_vert_deg);

  LOG_SECTION_END();
  return true;
}

// ============================================================
// Simple wrappers
// ============================================================
bool RBInterface::xyInfoMm(double* x, double* y) {
  *x = last.x_mm;
  *y = last.y_mm;
  return true;
}
bool RBInterface::gripperInfoPer(double* g) {
  *g = (last.g1_per + last.g2_per) / 2.0;
  return true;
}
bool RBInterface::baseInfoDeg(double* b) {
  *b = last.base_deg;
  return true;
}
bool RBInterface::wristVertInfoDeg(double* v) {
  *v = last.g_vert_deg;
  return true;
}

bool RBInterface::moveYmm(double y) {
  LOG_SECTION_START_PRINTF("moveYmm", "| y=%.2f", y);
  float a[] = { (float)y };
  bool ok = runCommand("MOVEYMM", a, 1);
  LOG_SECTION_END();
  return ok;
}
bool RBInterface::moveXmm(double x) {
  LOG_SECTION_START_PRINTF("moveXmm", "| x=%.2f", x);
  float a[] = { (float)x };
  bool ok = runCommand("MOVEXMM", a, 1);
  LOG_SECTION_END();
  return ok;
}
bool RBInterface::moveBaseDeg(double d) {
  LOG_SECTION_START_PRINTF("moveBaseDeg", "| deg=%.2f", d);
  float a[] = { (float)d };
  bool ok = runCommand("MOVEBASE", a, 1);
  LOG_SECTION_END();
  return ok;
}
bool RBInterface::moveWristVertDeg(double d) {
  LOG_SECTION_START_PRINTF("moveWristVertDeg", "| deg=%.2f", d);
  float a[] = { (float)d };
  bool ok = runCommand("MOVEWRISTVERTDEG", a, 1);
  LOG_SECTION_END();
  return ok;
}
bool RBInterface::moveGrippersPer(double p) {
  LOG_SECTION_START_PRINTF("moveGrippersPer", "| per=%.2f", p);
  float a[] = { (float)p };
  bool ok = runCommand("MOVEGRIPPER", a, 1);
  LOG_SECTION_END();
  return ok;
}

// ============================================================
// Error & Status helpers
// ============================================================
const char* RBInterface::getLastErrorLine() {
  return errorLines.empty() ? "" : errorLines.back().c_str();
}
String RBInterface::getAllErrorLines() const {
  String s;
  for (auto& e : errorLines) s += e + "\n";
  return s;
}
RBStatus RBInterface::getLastStatus() const {
  return last;
}

bool RBInterface::readUntilEnd(const char* keyword) {
  LOG_SECTION_START_PRINTF("readUntilEnd", "| key=%s", keyword);
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    if (!serial.available()) continue;
    String line = serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("ERR")) errorLines.push_back(line);
    if (line.endsWith("END")) {
      parseStatusLine(line);
      LOG_SECTION_END();
      return true;
    }
  }
  LOG_SECTION_END();
  return false;
}

void RBInterface::clearErrorBuffer() {
  errorLines.clear();
}

/* --------------------------------------------------------------------------------------------------------------------------------------------------
struct CommandEntry {
  const char* name;
  const char* fmt;
  bool (*handler)(int argc, double* argv);
  const char* desc;
};

static CommandEntry command_table[] = {
  { "VERBOSEON", "", cmd_verbose_on, "VERBOSEON - enable verbose output" },
  { "VERBOSEOFF", "", cmd_verbose_off, "VERBOSEOFF - disable verbose output" },

  { "SETMIN", "%d", cmd_set_min, "SETMIN <id> - set current pos as min" },
  { "SETMAX", "%d", cmd_set_max, "SETMAX <id> - set current pos as max" },
  { "SETZERO", "%d", cmd_set_zero, "SETLIMITMAX <id> - set current pos as zero" },
  { "SETDIRPLUS", "%d", cmd_set_dir_plus, "SETDIRPLUS <id> - set dir using the current pos > zero" },
  { "SETDIRMINUS", "%d", cmd_set_dir_minus, "SETDIRMINUS <id> - set dir using the current pos < zero" },

  { "POSZERO", "", cmd_pos_zero, "POSZERO - move to pos zero" },
  { "MOVETICKS", "%d %d", cmd_move_ticks, "MOVEDEG <id> <ticks goal> - move one servo to ticks (not smooth)" },
  { "MOVEDEG", "%d %f", cmd_move_deg, "MOVEDEG <id> <deg goal> - move one servo to degree (smooth)" },
  { "MOVEPER", "%d %f", cmd_move_per, "MOVEPER <id> <per goal> - move one servo to percent (smooth)" },
  { "MOVEYMM", "%f", cmd_move_y, "MOVEYMM <float mm> - vertical move" },
  { "MOVEXMM", "%f", cmd_move_x, "MOVEXMM <float mm> - lateral move" },
  { "MOVEXYMM", "%f %f", cmd_move_xy, "MOVEXYMM <float mm> <float mm> - lateral then vertical move" },
  { "MOVEGRIPPER", "%f", cmd_move_gripper, "MOVEGRIPPER <percentage> - move both grips to percentage" },
  { "MOVEWRISTVERTDEG", "%f", cmd_move_wrist_vert, "MOVEWRISTVERTDEG <deg> - move wrist relative to vertical" },

  { "READ", "%d", cmd_read, "READ <id> - show servo summary status" },
  { "INFO", "%d", cmd_info, "INFO <id> - show servo full status" },

  { "LEDON", "%d", cmd_ledon, "LEDON <id> - turn servo LED on" },
  { "LEDOFF", "%d", cmd_ledoff, "LEDOFF <id> - turn servo LED off" },
};

--------------------------------------------------------------------------------------------------------------------------------------------------

VERBOSEOFF START x_mm=3.10 y_mm=45.29 a1_deg=68.55 a2_deg=-39.29 g_vert_deg=0.79 g1_per=30.12 g2_per=30.12 base_deg=-39.29
Verbose OFF
VERBOSEOFF END completed=1 x_mm=3.10 y_mm=45.29 a1_deg=68.55 a2_deg=-39.29 g_vert_deg=0.79 g1_per=30.12 g2_per=30.12 base_deg=-39.29

INFO
  serial_printf("INFO id=%d", id);
  serial_printf(" op_mode=%d", op);
  serial_printf(" drive_mode=%d bit0=%s profil=", drv, (drv & 0x01) ? "TIME" : "VELOCITY");
  serial_printf(" profile_vel=%d  rpm=%.3frpm, tps=%.1f ticks_s)", pv, rpm, tps);
  serial_printf(" prifile_accel=%d", pa);
  serial_printf(" pos_min=%d", minL);
  serial_printf(" pos_max=%d", maxL);
  serial_printf(" span_deg=%.1f", spanDeg);
  serial_printf(" pos_present=%d\n", pos);

READ
      serial_printf_verbose("STATUS SERVO name=%s id=%d pos=%d deg=%.2f per=%.2f current_ma=%d temp_deg=%d\n",
                            s->get_key(), sid, pos_ticks, pos_deg, pos_per, curr_mA, temp_C);
*/

/*
USAGE EXAMPLE
  // Example: move to Y = 65 mm
  if (rb.moveYmm(65.0))
    Serial.printf("Move done, y=%.2f\n", rb.getLastStatus().y_mm);
  else
    Serial.println(rb.getAllErrorLines());
}
-------------------------------------------------------------------------------------------------------------------------------------------------*/
