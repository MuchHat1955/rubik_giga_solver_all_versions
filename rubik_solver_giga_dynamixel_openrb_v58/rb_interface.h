#pragma once
#include "utils.h"
#include "logging.h"
#include <Arduino.h>
#include <vector>
#include <functional>

class RBInterface {
public:
  using command_end_cb_t =
    void (*)(uint32_t id, const String& result, const String& duration);

  using info_cb_t =
    std::function<void(const String& module,
                       uint32_t id,
                       const String& payload)>;

  using error_cb_t =
    std::function<void(const String& module,
                       uint32_t id,
                       const String& payload)>;

  RBInterface();

  bool begin();

  // Sends command and waits until command_start is received
  uint32_t send_command(const String& command, const String& params);
  bool send_stop_command();

  // User callbacks
  void on_command_end(command_end_cb_t cb);
  void on_info(info_cb_t cb);
  void on_error(error_cb_t cb);

  // Must be called from loop()
  void poll();

  uint32_t get_current_cmd_id() const {
    return current_cmd_id_;
  }
  void force_command_end() {
    waiting_for_end_ = false;
  }

private:
  uint32_t current_cmd_id_ = 0;
  bool waiting_for_start_ = false;
  bool waiting_for_end_ = false;

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
bool runStopCommand();
bool checkServosStatus();
String getLastError(int cmdId);
bool getLastResult(int cmdId);
String getLastCommandName(int cmdId);
String getLastCommandParams(int cmdId);

String getOrientationForCmd(int cmd_id);
String getRbInterfaceVersion();

int getLastColorReadStep(int* cmd_id = nullptr);
int getLastCubeMoveStep(int* cmd_id = nullptr);
int getLastRobotMoveStep(int* cmd_id = nullptr);
String getLastColorReadMap(int* cmd_id = nullptr);
String getLastServoStatusStr(int servo_id);
char getLastColorOneColor(int* cmd_id);

String get_last_orientation();
String get_last_cube_solution();
String get_last_color_string54();
