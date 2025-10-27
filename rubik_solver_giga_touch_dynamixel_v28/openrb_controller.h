#pragma once
#include <Arduino.h>

/**
 * OpenRBController
 * ----------------
 * GIGA-side wrapper for communicating with the OpenRB controller (Serial2).
 * Automatically ensures VERBOSEOFF before sending commands to reduce traffic.
 */

struct XYStatus {
  float x_mm = 0, y_mm = 0;
  float a1_deg = 0, a2_deg = 0, grip_deg = 0;
  int a1_ticks = 0, a2_ticks = 0, grip_ticks = 0;
  bool valid = false;
};

struct ServoStatus {
  uint8_t id = 0;
  int pos = 0;
  int current_mA = 0;
  int temp_C = 0;
  bool valid = false;
};

class OpenRBController {
public:
  explicit OpenRBController(HardwareSerial &serial = Serial2, unsigned long baud = 115200)
    : port(serial), baudrate(baud), verboseState(false) {}

  void begin() {
    port.begin(baudrate);
    delay(100);
    flushInput();
    ensureVerboseOff(); // default to OFF on startup
  }

  // --------------------------------------------------------
  //  CORE LOW-LEVEL UTILITIES
  // --------------------------------------------------------
  void sendCommand(const String &cmd) {
    ensureVerboseOff();
    port.print(cmd);
    port.print("\n");
    port.flush();
  }

  String readLine(uint32_t timeout_ms = 1000) {
    String line = "";
    uint32_t t0 = millis();
    while (millis() - t0 < timeout_ms) {
      while (port.available()) {
        char c = port.read();
        if (c == '\r') continue;
        if (c == '\n') {
          if (line.length()) return line;
          else continue;
        }
        line += c;
      }
    }
    return line;
  }

  void flushInput() { while (port.available()) port.read(); }

  // --------------------------------------------------------
  //  VERBOSE CONTROL
  // --------------------------------------------------------
  void ensureVerboseOff() {
    if (!verboseState) return;
    port.print("VERBOSEOFF\n");
    port.flush();
    delay(50);
    verboseState = false;
    flushInput();
  }

  void ensureVerboseOn() {
    if (verboseState) return;
    port.print("VERBOSEON\n");
    port.flush();
    delay(50);
    verboseState = true;
    flushInput();
  }

  bool isVerboseOn() const { return verboseState; }

  // --------------------------------------------------------
  //  BASIC COMMANDS
  // --------------------------------------------------------
  void moveServo(uint8_t id, int ticks) {
    sendCommand("MOVE " + String(id) + " " + String(ticks));
  }
  void moveY(float y_mm) { sendCommand("MOVEY " + String(y_mm, 2)); }
  void moveX(float x_mm) { sendCommand("MOVEX " + String(x_mm, 2)); }
  void readXY() { sendCommand("READXY"); }
  void readServos() { sendCommand("READ"); }

  void setZero(uint8_t id, int ticks, int dir = 1) {
    sendCommand("SETZERO " + String(id) + " " + String(ticks) + " " + String(dir));
  }
  void setArmLength(float len) { sendCommand("SETLEN " + String(len, 2)); }
  void ledOn(uint8_t id) { sendCommand("LEDON " + String(id)); }
  void ledOff(uint8_t id) { sendCommand("LEDOFF " + String(id)); }

  // --------------------------------------------------------
  //  WAIT / PARSE HELPERS
  // --------------------------------------------------------
  bool waitForLine(const char *prefix, String &out, uint32_t timeout_ms = 3000) {
    uint32_t t0 = millis();
    while (millis() - t0 < timeout_ms) {
      String line = readLine(200);
      if (line.startsWith(prefix)) { out = line; return true; }
    }
    return false;
  }

  bool waitForXY(XYStatus &out, uint32_t timeout_ms = 4000) {
    String line;
    if (!waitForLine("READXY", line, timeout_ms)) return false;
    parseXYLine(line, out);
    out.valid = true;
    return true;
  }

  bool waitForServoTable(ServoStatus *arr, size_t maxN, uint32_t timeout_ms = 4000) {
    uint32_t t0 = millis();
    size_t count = 0;
    while (millis() - t0 < timeout_ms && count < maxN) {
      String line = readLine(200);
      if (line.startsWith("READ ") && line.indexOf('|') == -1) {
        ServoStatus s{};
        if (parseServoLine(line, s)) arr[count++] = s;
      }
    }
    return count > 0;
  }

  // --------------------------------------------------------
  //  SYNCHRONOUS HIGH-LEVEL ACTIONS
  // --------------------------------------------------------
  bool moveServoSync(uint8_t id, int ticks, ServoStatus *status = nullptr) {
    flushInput();
    moveServo(id, ticks);
    ServoStatus s{};
    if (readServoSync(id, s)) {
      if (status) *status = s;
      return true;
    }
    return false;
  }

  bool moveToYSync(float y_mm, XYStatus *status = nullptr) {
    flushInput();
    moveY(y_mm);
    XYStatus xy;
    if (waitForXY(xy)) {
      if (status) *status = xy;
      return true;
    }
    return false;
  }

  bool moveToXSync(float x_mm, XYStatus *status = nullptr) {
    flushInput();
    moveX(x_mm);
    XYStatus xy;
    if (waitForXY(xy)) {
      if (status) *status = xy;
      return true;
    }
    return false;
  }

  bool readXYSync(XYStatus &xy) {
    flushInput();
    readXY();
    return waitForXY(xy);
  }

  bool readServosSync(ServoStatus *arr, size_t maxN) {
    flushInput();
    readServos();
    return waitForServoTable(arr, maxN);
  }

  bool readServoSync(uint8_t id, ServoStatus &out) {
    ServoStatus tmp[10];
    if (!readServosSync(tmp, 10)) return false;
    for (auto &s : tmp) if (s.valid && s.id == id) { out = s; return true; }
    return false;
  }

  // --------------------------------------------------------
  //  PARSERS
  // --------------------------------------------------------
  static void parseXYLine(const String &line, XYStatus &st) {
    // Expect lines like:
    //   READXY MM 12.34mm 56.78mm
    //   READXY TICKS 2048 3072 1980
    const char *c = line.c_str();
    if (sscanf(c, "READXY MM %fmm %fmm", &st.x_mm, &st.y_mm) == 2) st.valid = true;
    else if (sscanf(c, "READXY TICKS %d %d %d", &st.a1_ticks, &st.a2_ticks, &st.grip_ticks) == 3)
      st.valid = true;
  }

  static bool parseServoLine(const String &line, ServoStatus &s) {
    // Expect: READ <id> <pos>
    int id, pos;
    if (sscanf(line.c_str(), "READ %d %d", &id, &pos) == 2) {
      s.id = id;
      s.pos = pos;
      s.valid = true;
      return true;
    }
    return false;
  }

private:
  HardwareSerial &port;
  unsigned long baudrate;
  bool verboseState; // false = VERBOSEOFF
};
