#pragma once
#include <Arduino.h>
#include "utils.h"

class ColorAnalyzer {
public:
  ColorAnalyzer();

  // ----------------------------------------------------------
  // Color string access (54 chars, URFDLB, row-major per face)
  // ----------------------------------------------------------
  void set_colors(const String &colors);
  const String &get_colors() const { return colors_; }

  // ----------------------------------------------------------
  // Validation / fixing
  // ----------------------------------------------------------
  // Fully legal Rubik cube state? (centers, counts, edges, corners)
  bool is_color_string_valid_bool() const;

  // True if current string is already valid OR can be fixed by changing
  // exactly one sticker to one of the 6 center colors.
  bool is_string_fixable_bool() const;

  // If already valid -> fixed_out = original, return true.
  // If can be fixed by changing exactly 1 sticker -> fixed_out = fixed string, return true.
  // Else return false.
  bool try_fix_color_string(String &fixed_out) const;

  // Human-readable diagnostic: "OK", or detailed explanation why invalid
  String get_string_check_log() const;

  // ----------------------------------------------------------
  // Stages (0..6)
  // ----------------------------------------------------------
  // 0: top face
  // 1: top layer
  // 2: middle layer
  // 3: bottom cross
  // 4: bottom layer
  // 5: bottom face
  // 6: cube solved
  int get_stage_count() const { return 7; }
  const char *get_stage_name(int id) const;

  bool is_stage_done_bool(int id) const;
  bool is_stage_partial_bool(int id) const;

private:
  String colors_;      // length 54, URFDLB, row-major per face
  mutable String last_error_;  // for detailed diagnostics

  // ----------------------------------------------------------
  // Index helpers
  // ----------------------------------------------------------
  int base_index(char face) const;
  char face_center_color(char face) const { return colors_[base_index(face) + 4]; }

  char face_center_color_from(const String &s, char face) const {
    return s[base_index(face) + 4];
  }

  // ----------------------------------------------------------
  // Stages implementation
  // ----------------------------------------------------------
  bool face_solved_bool(char face) const;

  bool top_layer_solved_bool() const;
  bool middle_layer_solved_bool() const;
  bool bottom_cross_solved_bool() const;
  bool bottom_layer_solved_bool() const;

  // ----------------------------------------------------------
  // Validation implementation
  // ----------------------------------------------------------
  bool is_color_string_valid_impl(const String &s) const;

  void compute_color_counts_from(const String &s, int out[256]) const;
  bool centers_correct_from(const String &s) const;
  bool valid_color_counts_from(const String &s) const;
  bool edges_corners_color_consistent_from(const String &s) const;

  // ----------------------------------------------------------
  // Small helpers
  // ----------------------------------------------------------
  void sort_pair(char &a, char &b) const;
  void sort_triple(char &a, char &b, char &c) const;
};
