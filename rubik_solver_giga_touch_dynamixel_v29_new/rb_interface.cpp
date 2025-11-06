#include "rb_interface.h"

/*
EXAMPLE
  rb.moveYmm(65.0);
  double x, y;
  rb.xyInfoMm(&x, &y);
  Serial.printf("Final Y=%.2f\n", y);
  */

// ============================================================
// Constructor
// ============================================================
RBInterface::RBInterface(Stream& port)
  : serial(port) {}

// ============================================================
// BEGIN - initialize link and disable verbose
// ============================================================
bool RBInterface::begin(unsigned long baud, uint32_t timeout_ms) {
  serial.begin(baud);
  serial.setTimeout(timeout_ms);
  clearErrorBuffer();

  Serial.println("[RB] Initializing communication...");
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
    if (line.startsWith("STATUS")) { got = true; break; }
  }

  if (!got) {
    Serial.println("[RB] No response from RB board!");
    return false;
  }

  Serial.println("[RB] Communication OK, reading servo INFO...");

  // Query INFO for all servos (11–17)
  uint8_t ids[] = { ID_ARM1, ID_ARM2, ID_WRIST, ID_GRIP1, ID_GRIP2, ID_BASE, ID_XM };
  for (uint8_t i = 0; i < sizeof(ids); i++) {
    requestServoInfo(ids[i]);
    delay(40);
  }

  Serial.println("[RB] Initialization complete, verbose OFF.");
  return true;
}

// ============================================================
// Generic command
// ============================================================
bool RBInterface::runCommand(const char* name, const float* args, int argCount) {
  clearErrorBuffer();

  String cmd(name);
  for (int i = 0; i < argCount; i++) {
    cmd += " ";
    cmd += String(args[i], 3);
  }
  serial.println(cmd);

  Serial.printf("[GIGA→RB] %s\n", cmd.c_str());
  return waitForCompletion(name);
}

// ============================================================
// INFO fetch helper
// ============================================================
bool RBInterface::requestServoInfo(uint8_t id) {
  String cmd = "INFO ";
  cmd += String(id);
  serial.println(cmd);

  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    if (!serial.available()) continue;
    String line = serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("INFO id=")) {
      Serial.println(line);
      return true;
    }
  }
  Serial.printf("[RB] WARN: No INFO response for ID %d\n", id);
  return false;
}

// ============================================================
// Parsing and verification
// ============================================================
void RBInterface::parseStatusLine(const String& line) {
  int idx = line.indexOf(' ');
  if (idx < 0) return;
  String rest = line.substring(idx + 1);

  char key[20];
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
// Wait for START/END/ERR sequence
// ============================================================
bool RBInterface::waitForCompletion(const char* commandName) {
  String startMarker = String(commandName) + " START";
  String endMarker   = String(commandName) + " END";
  unsigned long t0 = millis();
  bool success = false;

  while (millis() - t0 < 8000) {
    if (!serial.available()) continue;
    String line = serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    if (line.startsWith("ERR")) {
      errorLines.push_back(line);
      Serial.println(line);
      continue;
    }
    if (line.startsWith(startMarker)) {
      parseStatusLine(line);
      Serial.printf("[RB] %s started\n", commandName);
    }
    if (line.startsWith(endMarker)) {
      parseStatusLine(line);
      Serial.printf("[RB] %s ended completed=%d\n", commandName, last.completed);
      success = (last.completed == 1);
      verifyExpected(commandName);
      break;
    }
  }

  if (!success) errorLines.push_back("ERR: Timeout or incomplete command");
  return success;
}

// ============================================================
// Verify expected final status for MOVE commands
// ============================================================
void RBInterface::verifyExpected(const char* cmd) {
  double tol = 2.0;
  if (strncmp(cmd, "MOVEYMM", 7) == 0)
    Serial.printf("[VERIFY] y_mm=%.2f\n", last.y_mm);
  if (strncmp(cmd, "MOVEXMM", 7) == 0)
    Serial.printf("[VERIFY] x_mm=%.2f\n", last.x_mm);
  if (strncmp(cmd, "MOVEGRIPPER", 11) == 0)
    Serial.printf("[VERIFY] grip1=%.2f grip2=%.2f\n", last.g1_per, last.g2_per);
}

// ============================================================
// Simple wrappers
// ============================================================
bool RBInterface::updateInfo() { return readUntilEnd(nullptr); }
bool RBInterface::xyInfoMm(double* x, double* y) { *x=last.x_mm; *y=last.y_mm; return true; }
bool RBInterface::gripperInfoPer(double* g) { *g=(last.g1_per+last.g2_per)/2.0; return true; }
bool RBInterface::baseInfoDeg(double* b) { *b=last.base_deg; return true; }
bool RBInterface::wristVertInfoDeg(double* v) { *v=last.g_vert_deg; return true; }

bool RBInterface::moveYmm(double y) { float a[]={ (float)y }; return runCommand("MOVEYMM", a, 1); }
bool RBInterface::moveXmm(double x) { float a[]={ (float)x }; return runCommand("MOVEXMM", a, 1); }
bool RBInterface::moveBaseDeg(double d) { float a[]={ (float)d }; return runCommand("MOVEBASE", a, 1); }
bool RBInterface::moveWristVertDeg(double d){ float a[]={ (float)d }; return runCommand("MOVEWRISTVERTDEG", a, 1); }
bool RBInterface::moveGrippersPer(double p){ float a[]={ (float)p }; return runCommand("MOVEGRIPPER", a, 1); }

const char* RBInterface::getLastErrorLine(){ return errorLines.empty()?"":errorLines.back().c_str(); }
String RBInterface::getAllErrorLines() const{ String s; for(auto&e:errorLines)s+=e+"\n"; return s; }
RBStatus RBInterface::getLastStatus() const{ return last; }

bool RBInterface::readUntilEnd(const char* keyword){ unsigned long t0=millis(); while(millis()-t0<2000){ if(!serial.available())continue; String line=serial.readStringUntil('\n'); line.trim(); if(line.startsWith("ERR"))errorLines.push_back(line); if(line.endsWith("END")){ parseStatusLine(line); return true; } } return false; }

void RBInterface::clearErrorBuffer(){ errorLines.clear(); }


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
