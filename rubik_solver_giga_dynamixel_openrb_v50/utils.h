#pragma once

#include <arduino.h>
#include <Arduino_H7_Video.h>
#include <lvgl.h>
#include <Arduino_GigaDisplayTouch.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include <kvstore.h>
#include <kvstore_global_api.h>

#define _SERIAL_SOLVER Serial3 // on the board is marked as Serial2
#define _SERIAL_RB Serial4 // on the board is marked as Serial2
#define SERIAL_BAUD 115200
#define SERIAL_TIMEOUT 500
#define SERIAL_CMD_TIMEOUT 6000