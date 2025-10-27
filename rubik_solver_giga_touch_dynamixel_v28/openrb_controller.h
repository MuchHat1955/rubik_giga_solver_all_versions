#pragma once
#include <Arduino.h>

/**
 * OpenRBController (GIGA side)
 * -----------------------------
 * Communicates with the OpenRB controller via Serial2.
 * - Ensures VERBOSEOFF for compact output
 * - Provides synchronous MOVE/READ commands
 * - Caches last-known servo + XY states
 * - Supports mm-based vertical/lateral moves
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

  // ----------------------------------------------------
  // Initialization
  // ----------------------------------------------------
  void begin() {
    port.begin(baudrate);
    delay(100);
    flushInput();
    ensureVerboseOff();
  }

  // ----------------------------------------------------
  // Low-level I/O
  // ----------------------------------------------------
  void sendCommand(const String &cmd) {
    ensureVerboseOff();
    port.print(cmd);
    port.print("\n");
    port.flush();
  }

  String readLine(uint32_t timeout_ms = 1000) {
    String line;
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

  // ----------------------------------------------------
  // Verbose control
  // ----------------------------------------------------
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

  // ----------------------------------------------------
  // Core motion commands
  // ----------------------------------------------------
  void moveServo(uint8_t id, int ticks) { sendCommand("MOVE " + String(id) + " " + String(ticks)); }
  void moveY(float y_mm) { sendCommand("MOVEY " + String(y_mm, 2)); }
  void moveX(float x_mm) { sendCommand("MOVEX " + String(x_mm, 2)); }
  void readXY() { sendCommand("READXY"); }
  void readServos() { sendCommand("READ"); }

  void setZero(uint8_t id, int ticks, int dir = 1) {
    sendCommand("SETZERO " + String(id) + " " + String(ticks) + " " + String(dir));
  }
  void setArmLength(float len) { sendCommand("SETLEN " + String(len, 2)); }

  // ----------------------------------------------------
  // Wait + Parse helpers
  // ----------------------------------------------------
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
    if (out.valid) lastXY = out;
    return out.valid;
  }

  bool waitForServoTable(ServoStatus *arr, size_t maxN, uint32_t timeout_ms = 4000) {
    uint32_t t0 = millis();
    size_t count = 0;
    while (millis() - t0 < timeout_ms && count < maxN) {
      String line = readLine(200);
      if (line.startsWith("READ ") && line.indexOf('|') == -1) {
        ServoStatus s{};
        if (parseServoLine(line, s)) {
          arr[count++] = s;
          cacheServo(s);
        }
      }
    }
    return count > 0;
  }

  // ----------------------------------------------------
  // Synchronous moves
  // ----------------------------------------------------
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

  // ----------------------------------------------------
  // Parsing utilities
  // ----------------------------------------------------
  static void parseXYLine(const String &line, XYStatus &st) {
    const char *c = line.c_str();
    if (sscanf(c, "READXY MM %fmm %fmm", &st.x_mm, &st.y_mm) == 2)
      st.valid = true;
    if (sscanf(c, "READXY TICKS %d %d %d", &st.a1_ticks, &st.a2_ticks, &st.grip_ticks) == 3)
      st.valid = true;
  }

  static bool parseServoLine(const String &line, ServoStatus &s) {
    int id, pos;
    if (sscanf(line.c_str(), "READ %d %d", &id, &pos) == 2) {
      s.id = id;
      s.pos = pos;
      s.valid = true;
      return true;
    }
    return false;
  }

  // ----------------------------------------------------
  // Cached state
  // ----------------------------------------------------
  const XYStatus &getLastXY() const { return lastXY; }

  bool getLastServo(uint8_t id, ServoStatus &out) const {
    for (int i = 0; i < cacheCount; i++) {
      if (servoCache[i].valid && servoCache[i].id == id) {
        out = servoCache[i];
        return true;
      }
    }
    return false;
  }

  void clearCache() {
    cacheCount = 0;
    lastXY.valid = false;
  }

  // ----------------------------------------------------
  // High-level millimeter wrappers
  // ----------------------------------------------------
  bool moveToVertical() {
    ensureVerboseOff();
    flushInput();
    sendCommand("MOVEX 0");
    XYStatus xy;
    return waitForXY(xy, 3000);
  }

  bool moveToYmm(float targetY_mm, XYStatus *status = nullptr) {
    ensureVerboseOff();
    moveToVertical();  // align before going up/down
    flushInput();
    sendCommand("MOVEY " + String(targetY_mm, 2));
    XYStatus xy;
    if (waitForXY(xy, 5000)) {
      lastXY = xy;
      if (status) *status = xy;
      return true;
    }
    return false;
  }

  bool moveToXmm(float xOffset_mm, XYStatus *status = nullptr) {
    ensureVerboseOff();
    flushInput();
    sendCommand("MOVEX " + String(xOffset_mm, 2));
    XYStatus xy;
    if (waitForXY(xy, 5000)) {
      lastXY = xy;
      if (status) *status = xy;
      return true;
    }
    return false;
  }

private:
  void cacheServo(const ServoStatus &s) {
    if (!s.valid) return;
    for (int i = 0; i < cacheCount; i++) {
      if (servoCache[i].id == s.id) { servoCache[i] = s; return; }
    }
    if (cacheCount < MAX_CACHE) servoCache[cacheCount++] = s;
  }

  HardwareSerial &port;
  unsigned long baudrate;
  bool verboseState; // false = VERBOSEOFF
  static constexpr int MAX_CACHE = 10;
  ServoStatus servoCache[MAX_CACHE];
  int cacheCount = 0;
  XYStatus lastXY;
};

