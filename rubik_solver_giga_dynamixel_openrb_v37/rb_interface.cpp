#include "rb_interface.h"
#include "logging.h"
#include "ui_touch.h"
#include "pose_store.h"

extern PoseStore pose_store;

// ============================================================
// Constructor
// ============================================================
RBInterface::RBInterface() {}

// ============================================================
// BEGIN - initialize link and disable verbose
// ============================================================
bool RBInterface::begin(unsigned long baud, uint32_t timeout_ms) {
  LOG_SECTION_START("begin", "| baud=%lu timeout=%lu", baud, timeout_ms);

  Serial2.begin(baud);
  Serial2.setTimeout(timeout_ms);
  clearErrorBuffer();

  LOG_PRINTF("Initializing communication...");
  delay(300);

  // Turn verbose OFF to get clean command responses
  Serial2.println("VERBOSEOFF");
  delay(100);

  // Flush any startup noise
  while (Serial2.available()) Serial2.read();

  // Confirm RB responds
  Serial2.println("READ 0");
  unsigned long t0 = millis();
  bool got = false;
  while (millis() - t0 < 2000) {
    if (!Serial2.available()) continue;
    String line = Serial2.readStringUntil('\n');
    if (line.startsWith("STATUS")) {
      got = true;
      break;
    }
  }

  if (!got) {
    pose_store.set_all_poses_last_run(false);
    LOG_ERROR("no response from rb");
    LOG_SECTION_END();
    return false;
  }

  LOG_PRINTF("Communication OK, reading servo INFO...\n");

  // Query INFO for all servos (11–16)
  uint8_t ids[] = { ID_ARM1, ID_ARM2, ID_WRIST, ID_GRIPPER1, ID_GRIPPER2, ID_BASE };
  for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
    requestServoInfo(ids[i]);
    delay(40);
  }

  LOG_PRINTF("Initialization complete, verbose OFF.\n");
  LOG_SECTION_END();
  return true;
}

// ============================================================
// Generic command
// ============================================================
bool RBInterface::runCommand(const char* name, const float* args, int argCount) {
  LOG_SECTION_START("runCommand", "| cmd{%s} argc{%d}", name, argCount);

  clearErrorBuffer();

  String cmd(name);
  for (int i = 0; i < argCount; i++) {
    cmd += " ";
    if (strcmp(name, "MOVEPER") == 0 && i == 0)
      cmd += String((int)args[i]);  // integer formatting for servo ID
    else
      cmd += String(args[i], 3);  // 3-decimal float
  }

  setFooter(cmd.c_str());
  Serial2.println(cmd);

  LOG_PRINTF("[GIGA→RB] {%s}", cmd.c_str());
  bool ok = waitForCompletion(name);

  LOG_PRINTF("Command {%s} result{%s}\n", name, ok ? "OK" : "FAIL");
  String txt = cmd + " " + (ok ? "OK" : "FAIL");
  setFooter(txt.c_str());
  LOG_SECTION_END();
  return ok;
}

// ============================================================
// INFO fetch helper
// ============================================================
bool RBInterface::requestServoInfo(uint8_t id) {
  LOG_SECTION_START("requestServoInfo", "| id{%d}", id);

  String cmd = "INFO " + String(id);
  Serial2.println(cmd);

  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    if (!Serial2.available()) continue;
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (!line.startsWith("INFO id=")) continue;

    LOG_PRINTF("{%s}", line.c_str());

    ServoInfo info;
    info.id = id;
    sscanf(line.c_str(),
           "INFO id=%hhu op_mode=%hhu drive_mode=%hhu profile_vel=%d rpm=%lf tps=%lf profile_accel=%d pos_min=%d pos_max=%d span_deg=%lf pos_present=%d",
           &info.id, &info.op_mode, &info.drive_mode,
           &info.profile_vel, &info.rpm, &info.ticks_per_sec,
           &info.profile_accel, &info.pos_min, &info.pos_max,
           &info.span_deg, &info.pos_present);
    info.time_based = (info.drive_mode & 0x01);

    if (id < MAX_SERVOS) servo_infos[id] = info;
    info.log();

    LOG_SECTION_END();
    return true;
  }

  LOG_ERROR("no INFO for servo ID {%d}\n", id);
  LOG_SECTION_END();
  return false;
}

// ============================================================
// Request all servos and cache in array
// ============================================================
bool RBInterface::requestAllServoInfo() {
  LOG_SECTION_START("requestAllServoInfo","");

  bool all_ok = true;

  for (auto id : { ID_ARM1, ID_ARM2, ID_WRIST, ID_GRIPPER1, ID_GRIPPER2, ID_BASE }) {
    String cmd = "INFO " + String(id);
    Serial2.println(cmd);

    unsigned long t0 = millis();
    bool got = false;

    while (millis() - t0 < 1000) {
      if (!Serial2.available()) continue;
      String line = Serial2.readStringUntil('\n');
      line.trim();
      if (!line.startsWith("INFO id=")) continue;

      got = true;
      LOG_PRINTF("{%s}", line.c_str());

      ServoInfo info;
      info.id = id;

      sscanf(line.c_str(),
             "INFO id=%hhu op_mode=%hhu drive_mode=%hhu profile_vel=%d rpm=%lf tps=%lf profile_accel=%d pos_min=%d pos_max=%d span_deg=%lf pos_present=%d",
             &info.id, &info.op_mode, &info.drive_mode,
             &info.profile_vel, &info.rpm, &info.ticks_per_sec,
             &info.profile_accel, &info.pos_min, &info.pos_max,
             &info.span_deg, &info.pos_present);

      info.time_based = (info.drive_mode & 0x01);

      if (id < MAX_SERVOS) servo_infos[id] = info;
      break;
    }

    if (!got) {
      all_ok = false;
      LOG_ERROR("no INFO for servo ID {%d}\n", id);
      if (id < MAX_SERVOS) servo_infos[id].clear();
    }
  }

  LOG_SECTION_END();
  return all_ok;
}

// ============================================================
// Return all cached servo info as text lines
// ============================================================
String RBInterface::getAllServoInfoLines() const {
  String out;
  out.reserve(1024);

  for (auto id : { ID_ARM1, ID_ARM2, ID_WRIST, ID_GRIPPER1, ID_GRIPPER2, ID_BASE }) {
    if (id >= MAX_SERVOS) continue;
    const ServoInfo& s = servo_infos[id];
    if (s.id == 0) continue;  // skip empty slots

    out += String(s.id) + ". info" +                                    //
           " | op_mode " + String(s.op_mode) +                          //
           " | drive_mode " + String(s.drive_mode) +                    //
           " | time_based " + String(s.time_based ? "time" : "velo") +  //
           " | profile_vel " + String(s.profile_vel) +                  //
           " | rpm " + String(s.rpm, 3) +                               //
           " | tps " + String(s.ticks_per_sec, 1) +                     //
           " | profile_accel " + String(s.profile_accel) +              //
           " | pos_min " + String(s.pos_min) +                          //
           " | pos_max " + String(s.pos_max) +                          //
           " | span_deg " + String(s.span_deg, 1) +                     //
           " | pos_present " + String(s.pos_present) + "\n";            //
  }

  return out;
}

// ============================================================
// Parsing and verification (rest of your existing code)
// ============================================================
void RBInterface::parseStatusLine(const String& line) {
  if (line.startsWith("STATUS SERVO")) {
    if (verboseOn) LOG_PRINTF("{%s}", line.c_str());
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
// Wait for START/END/ERR sequence
// ============================================================
bool RBInterface::waitForCompletion(const char* commandName) {
  LOG_SECTION_START("waitForCompletion", "| cmd{%s}", commandName);

  String startMarker = String(commandName) + " START";
  String endMarker = String(commandName) + " END";
  unsigned long t0 = millis();
  bool success = false;

  while (millis() - t0 < 8000) {
    if (!Serial2.available()) continue;
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) continue;

    // --- Handle ERR lines ---
    if (line.startsWith("ERR")) {
      setFooter(line.c_str());
      addErrorLine(line);
      LOG_PRINTF("{%s}", line.c_str());
      continue;
    }

    // --- Handle progress lines ---
    // Any line starting with "MOVING" is considered progress feedback
    if (line.startsWith("MOVING")) {
      if (verboseOn) LOG_PRINTF("{%s}", line.c_str());
      setFooter(line.c_str());  // NEW HOOK
      continue;
    }

    // --- Handle START marker ---
    if (line.startsWith(startMarker)) {
      parseStatusLine(line);
      LOG_PRINTF("{%s} started", commandName);
      setFooter(line.c_str());
      continue;
    }

    // --- Handle END marker ---
    if (line.startsWith(endMarker)) {
      parseStatusLine(line);
      setFooter(line.c_str());
      LOG_PRINTF("{%s} ended completed{%d}", commandName, last.completed);
      success = (last.completed == 1);

      break;
    }
  }

  if (!success) {
    setFooter("[!] timeout or incomplete command");
    LOG_ERROR("timeout or incomplete command {%s}", commandName);
  }

  LOG_SECTION_END();
  return success;
}

// ============================================================
// Verify expected final status for MOVE commands
// ============================================================
bool RBInterface::verifyExpected(const char* cmd_name, double val, int servo_id, double tol) {
  LOG_SECTION_START("verifyExpected", "| cmd{%s}", cmd_name);

  double actual = 0.0, err = 0.0;
  char buf[200];

  // Handle MOVEGRIPPER separately (two servos)
  if (strcmp(cmd_name, "MOVEGRIPPER") == 0) {
    double err1 = fabs(last.g1_per - val);
    double err2 = fabs(last.g2_per - val);

    LOG_PRINTF("verify expected move{%s} g1_per expected{%.2f} actual{%.2f} err{%.2f}", cmd_name, val, last.g1_per, err1);
    LOG_PRINTF("verify expected move{%s} g2_per expected{%.2f} actual{%.2f} err{%.2f}", cmd_name, val, last.g2_per, err2);

    if (err1 <= tol && err2 <= tol) return true;

    if (err1 > tol) {
      snprintf(buf, sizeof(buf), "verify expected move{%s} g1_per expected{%.2f} actual{%.2f} err{%.2f}", cmd_name, val, last.g1_per, err1);
      addErrorLine(buf);
    }
    if (err2 > tol) {
      snprintf(buf, sizeof(buf), "verify expected move{%s} g2_per expected{%.2f} actual{%.2f} err{%.2f}", cmd_name, val, last.g2_per, err2);
      addErrorLine(buf);
    }
    return false;
  }

  // For all other commands: pick the actual value
  if (strcmp(cmd_name, "MOVEYMM") == 0) actual = last.y_mm;
  else if (strcmp(cmd_name, "MOVEXMM") == 0) actual = last.x_mm;
  else if (strcmp(cmd_name, "MOVEWRISTVERT") == 0) actual = last.g_vert_deg;
  else if (strcmp(cmd_name, "MOVEPER") == 0) {
    if (servo_id == ID_BASE) actual = last.base_deg;
    else if (servo_id == ID_GRIPPER1) actual = last.g1_per;
    else if (servo_id == ID_GRIPPER2) actual = last.g2_per;
    else actual = 0;
  } else {
    LOG_PRINTF("[!] unknown cmd_name: %s", cmd_name);
    return true;
  }

  // Common computation and logging
  err = fabs(actual - val);
  LOG_PRINTF("verify expected move{%s} servo{%d} expected{%.2f} actual{%.2f} err{%.2f}", cmd_name, servo_id, val, actual, err);

  if (err <= tol) return true;

  LOG_ERROR("verify expected move{%s} servo{%d} expected{%.2f} actual{%.2f} err{%.2f}",
               cmd_name, servo_id, val, actual, err);

  LOG_SECTION_END();
  return false;
}

// ============================================================
// UpdateInfo - force RB to send current status (READ 0)
// ============================================================
bool RBInterface::updateInfo() {
  LOG_SECTION_START("updateInfo","");

  clearErrorBuffer();
  Serial2.println("READ 0");
  LOG_PRINTF("requesting READ 0...\n");

  unsigned long t0 = millis();
  bool gotAny = false;

  while (millis() - t0 < 2000) {
    if (!Serial2.available()) continue;

    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) continue;

    if (line.startsWith("ERR")) {
      addErrorLine(line);
      LOG_PRINTF("{%s}", line.c_str());
      continue;
    }

    if (line.startsWith("STATUS SERVO")) {
      gotAny = true;
      parseStatusLine(line);
      continue;
    }

    if (line.startsWith("READ END") || line.indexOf("x_mm=") >= 0) {
      parseStatusLine(line);
      gotAny = true;
    }
  }

  if (!gotAny) {
    addErrorLine("no read 0 received");
    LOG_PRINTF("[!] no response for read 0\n");
    pose_store.set_all_poses_last_run(false);
    LOG_SECTION_END();
    return false;
  }

  LOG_PRINTF("READ 0 done | X{%.2f} Y{%.2f} A1{%.2f} A2{%.2f} G{%.2f}\n",
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
bool RBInterface::grippersInfoPer(double* g) {
  *g = (last.g1_per + last.g2_per) / 2.0;
  return true;
}
bool RBInterface::gripper1InfoPer(double* g) {
  *g = last.g1_per;
  return true;
}
bool RBInterface::gripper2InfoPer(double* g) {
  *g = last.g2_per;
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
  LOG_SECTION_START("moveYmm", "| y{%.2f}", y);
  float a[] = { (float)y };
  char cmd[] = "MOVEYMM";
  bool ok = runCommand(cmd, a, 1);
  if (ok) ok = verifyExpected(cmd, y, -1, 1.0);
  LOG_SECTION_END();
  return ok;
}
bool RBInterface::moveXmm(double x) {
  LOG_SECTION_START("moveXmm", "| x{%.2f}", x);
  float a[] = { (float)x };
  char cmd[] = "MOVEXMM";
  bool ok = runCommand(cmd, a, 1);
  if (ok) ok = verifyExpected(cmd, x, -1, 1.0);
  LOG_SECTION_END();
  return ok;
}
bool RBInterface::moveBaseDeg(double d) {
  LOG_SECTION_START("moveBaseDeg", "| deg{%.2f}", d);
  float a[] = { (float)ID_BASE, (float)d };
  char cmd[] = "MOVEPER";
  bool ok = runCommand(cmd, a, 2);
  if (ok) ok = verifyExpected(cmd, d, ID_BASE, 1.0);
  LOG_SECTION_END();
  return ok;
}
bool RBInterface::moveWristVertDeg(double d) {
  LOG_SECTION_START("moveWristVertDeg", "| deg{%.2f}", d);
  float a[] = { (float)d };
  char cmd[] = "MOVEWRISTVERTDEG";
  bool ok = runCommand(cmd, a, 1);
  if (ok) ok = verifyExpected(cmd, d, -1, 1.0);
  LOG_SECTION_END();
  return ok;
}
bool RBInterface::moveGrippersPer(double p) {
  LOG_SECTION_START("moveGrippersPer", "| per{%.2f}", p);
  float a[] = { (float)p };
  char cmd[] = "MOVEGRIPPERPER";
  bool ok = runCommand(cmd, a, 1);
  if (ok) ok = verifyExpected(cmd, p, -1, 1.0);
  LOG_SECTION_END();
  return ok;
}
bool RBInterface::moveGripper1Per(double p) {
  LOG_SECTION_START("moveGripper1Per", "| per{%.2f}", p);
  float a[] = { (float)ID_GRIPPER1, (float)p };
  char cmd[] = "MOVEPER";
  bool ok = runCommand(cmd, a, 2);
  if (ok) ok = verifyExpected(cmd, p, ID_GRIPPER1, 1.0);
  LOG_SECTION_END();
  return ok;
}
bool RBInterface::moveGripper2Per(double p) {
  LOG_SECTION_START("moveGripper2Per", "| per{%.2f}", p);
  float a[] = { (float)ID_GRIPPER2, (float)p };
  char cmd[] = "MOVEPER";
  bool ok = runCommand(cmd, a, 2);
  if (ok) ok = verifyExpected(cmd, p, ID_GRIPPER2, 1.0);
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
  for (auto& e : errorLines) {
    String line = e;
    line.trim();  // removes \n, \r, and spaces at ends
    s += line + "\n";
  }
  return s;
}
RBStatus RBInterface::getLastStatus() const {
  return last;
}

bool RBInterface::readUntilEnd(const char* keyword) {
  LOG_SECTION_START("readUntilEnd", "| key{%s}", keyword);
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    if (!Serial2.available()) continue;
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.startsWith("ERR")) addErrorLine(line);
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
  // errorLines.clear(); //TODO should see if this needs to be ever cleared
}

RBInterface rb;

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

  { "SETMIN", "{%d}", cmd_set_min, "SETMIN <id> - set current pos as min" },
  { "SETMAX", "{%d}", cmd_set_max, "SETMAX <id> - set current pos as max" },
  { "SETZERO", "{%d}", cmd_set_zero, "SETLIMITMAX <id> - set current pos as zero" },
  { "SETDIRPLUS", "{%d}", cmd_set_dir_plus, "SETDIRPLUS <id> - set dir using the current pos > zero" },
  { "SETDIRMINUS", "{%d}", cmd_set_dir_minus, "SETDIRMINUS <id> - set dir using the current pos < zero" },

  { "POSZERO", "", cmd_pos_zero, "POSZERO - move to pos zero" },
  { "MOVETICKS", "{%d} {%d}", cmd_move_ticks, "MOVEDEG <id> <ticks goal> - move one servo to ticks (not smooth)" },
  { "MOVEDEG", "{%d} %f", cmd_move_deg, "MOVEDEG <id> <deg goal> - move one servo to degree (smooth)" },
  { "MOVEPER", "{%d} %f", cmd_move_per, "MOVEPER <id> <per goal> - move one servo to percent (smooth)" },
  { "MOVEYMM", "%f", cmd_move_y, "MOVEYMM <float mm> - vertical move" },
  { "MOVEXMM", "%f", cmd_move_x, "MOVEXMM <float mm> - lateral move" },
  { "MOVEXYMM", "%f %f", cmd_move_xy, "MOVEXYMM <float mm> <float mm> - lateral then vertical move" },
  { "MOVEgripperPER", "%f", cmd_move_gripper, "MOVEgripperPER <percentage> - move both grippers to percentage" },
  { "MOVEWRISTVERTDEG", "%f", cmd_move_wrist_vert, "MOVEWRISTVERTDEG <deg> - move wrist relative to vertical" },

  { "READ", "{%d}", cmd_read, "READ <id> - show servo summary status" },
  { "INFO", "{%d}", cmd_info, "INFO <id> - show servo full status" },

  { "LEDON", "{%d}", cmd_ledon, "LEDON <id> - turn servo LED on" },
  { "LEDOFF", "{%d}", cmd_ledoff, "LEDOFF <id> - turn servo LED off" },
};

--------------------------------------------------------------------------------------------------------------------------------------------------

VERBOSEOFF START x_mm=3.10 y_mm=45.29 a1_deg=68.55 a2_deg=-39.29 g_vert_deg=0.79 g1_per=30.12 g2_per=30.12 base_deg=-39.29
Verbose OFF
VERBOSEOFF END completed=1 x_mm=3.10 y_mm=45.29 a1_deg=68.55 a2_deg=-39.29 g_vert_deg=0.79 g1_per=30.12 g2_per=30.12 base_deg=-39.29

INFO
  serial_printf("INFO id{%d}", id);
  serial_printf(" op_mode{%d}", op);
  serial_printf(" drive_mode{%d} bit0{%s} profil=", drv, (drv & 0x01) ? "TIME" : "VELOCITY");
  serial_printf(" profile_vel{%d}  rpm=%.3frpm, tps=%.1f ticks_s)", pv, rpm, tps);
  serial_printf(" prifile_accel{%d}", pa);
  serial_printf(" pos_min{%d}", minL);
  serial_printf(" pos_max{%d}", maxL);
  serial_printf(" span_deg=%.1f", spanDeg);
  serial_printf(" pos_present{%d}\n", pos);

READ
      serial_printf_verbose("STATUS SERVO name{%s} id{%d} pos{%d} deg{%.2f} per{%.2f} current_ma{%d} temp_deg{%d}\n",
                            s->get_key(), sid, pos_ticks, pos_deg, pos_per, curr_mA, temp_C);
*/

/*
USAGE EXAMPLE
  // Example: move to Y = 65 mm
  if (rb.moveYmm(65.0))
    Serial.printf("Move done, y{%.2f}\n", rb.getLastStatus().y_mm);
  else
    Serial.println(rb.getAllErrorLines());
}
-------------------------------------------------------------------------------------------------------------------------------------------------*/
