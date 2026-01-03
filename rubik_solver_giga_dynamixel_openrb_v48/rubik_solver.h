#pragma once
#include <Arduino.h>

bool is_solved_top_two_layers(const String &cube);
String find_solution_for_bottom_layer(const String &cube);
String compress_moves(const String &moves);
String getColorsForSolution(const String &solution);
bool is_valid_color_string(const String &str54);
