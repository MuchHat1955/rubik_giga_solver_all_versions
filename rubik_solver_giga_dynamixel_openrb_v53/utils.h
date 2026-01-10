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

// on the board is marked as Serial1
#define _SERIAL_SOLVER Serial2

// on the board is marked as Serial3
#define _SERIAL_RB Serial3

#define SERIAL_BAUD 115200
#define SERIAL_TIMEOUT 500
#define SERIAL_CMD_TIMEOUT 6000