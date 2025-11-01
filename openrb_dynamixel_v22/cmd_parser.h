#pragma once
#include <Arduino.h>
#include "servos.h"
#include "movement.h"
#include "vertical_kinematics.h"
#include "utils.h"

// -------------------------------------------------------------------
//                      COMMAND PARSER - DECLARATION
// -------------------------------------------------------------------

// Lightweight Arduino scanf alternative
int arduino_sscanf(const String &line, const char *fmt, void *out);

// Command dispatcher
void process_serial_command(String &line);

// Helpers for showing servo info
void cmdInfo(uint8_t id);
void cmdSetLimit(uint8_t id, int minL, int maxL);
void print_pos(uint8_t id);
String get_help_text();
