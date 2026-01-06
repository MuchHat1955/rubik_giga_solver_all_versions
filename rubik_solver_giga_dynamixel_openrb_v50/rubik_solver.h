#pragma once
#include <Arduino.h>

#define SOLVER_SERIAL Serial3

bool is_solved_top_two_layers(const String &cube);
String find_solution_for_bottom_layer(const String &cube);
String compress_moves(const String &moves);
String getColorsForSolution(const String &solution);
bool is_valid_color_string(const String &str54);

// ============================================================
// Teensy Solver API
// ============================================================

bool solver_begin();

String solver_get_version();

bool solver_find_solution(const String& cube54,
                          String& out_solution,
                          int& out_move_count,
                          int* out_time_ms = nullptr);

String solver_get_last_error();

