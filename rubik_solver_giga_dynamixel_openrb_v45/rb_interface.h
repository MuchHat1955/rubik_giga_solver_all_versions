#pragma once
#include "utils.h"
#include "logging.h"
#include <Arduino.h>
#include <vector>
#include <functional>

class RBInterface {
public:
  using command_end_cb_t =
    std::function<void(const String& result, const String& duration)>;

  using info_cb_t =
    std::function<void(const String& module,
                       uint32_t id,
                       const String& payload)>;

  using error_cb_t =
    std::function<void(const String& module,
                       uint32_t id,
                       const String& payload)>;

  RBInterface();

  bool begin(unsigned long baud, uint32_t timeout_ms);

  // Sends command and waits until command_start is received
  uint32_t send_command(const String& command, const String& params);

  // User callbacks
  void on_command_end(command_end_cb_t cb);
  void on_info(info_cb_t cb);
  void on_error(error_cb_t cb);

  // Must be called from loop()
  void poll();

  uint32_t get_current_cmd_id() const {
    return current_cmd_id_;
  }

private:
  HardwareSerial* serial_ = &Serial2;

  uint32_t current_cmd_id_ = 0;
  bool waiting_for_start_ = false;

  command_end_cb_t command_end_cb_;
  info_cb_t info_cb_;
  error_cb_t error_cb_;

  void handle_line(const String& line);

  static uint32_t parse_id(const String& line);
};

void init_rb_wrappers();
bool runCommand(const String& command,
                const String& params,
                int* cmdId);
bool checkServosStatus();
String getLastError(int cmdId);
bool getLastResult(int cmdId);
String getLastCommandName(int cmdId);
String getLastCommandParams(int cmdId);

// TODO use all of below
String getLastColorString();
String getLastOrientation();
String getColorStringForCmd(uint32_t cmd_id);
String getOrientationForCmd(uint32_t cmd_id);
String getRbInterfaceVersion();

String getLastColorStringCurr(uint32_t* cmd_id = nullptr);
int getLastColorReadStep(uint32_t* cmd_id = nullptr);
int getLastCubeMoveStep(uint32_t* cmd_id = nullptr);
int getLastRobotMoveStep(uint32_t* cmd_id = nullptr);
String getLastColorReadMap(uint32_t* cmd_id = nullptr);
bool getLastServoStatus(int servo_id, rb_servo_status_t& out);
String getLastServoStatusStr(int servo_id);
