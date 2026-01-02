#pragma once
#include <Arduino.h>

bool top_two_layers_solved_bool(const String &cube);
String find_solution_solve_bottom_layer(const String &cube);
String compress_moves(const String &moves);
String getColorsForSolution(const String &solution);
