#pragma once
#include <Arduino.h>
#include "utils.h"

class ColorAnalyzer {
public:
  ColorAnalyzer();

  // ----------------------------------------------------------
  // Color string access
  // ----------------------------------------------------------
  void set_colors(const String &colors);
  const String& get_colors() const { return colors_; }

  // ----------------------------------------------------------
  // Validation / fixing
  // ----------------------------------------------------------
  bool is_color_string_valid_bool() const;
  bool is_string_fixable_bool() const;                    // true if already valid OR 1-sticker-fixable

  // If valid -> returns same string in fixed_out.
  // If 1-change fixable -> returns corrected string in fixed_out.
  // Else -> returns false.
  bool try_fix_color_string(String &fixed_out) const;

  String get_string_check_log() const;                    // human readable explanation

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
  const char* get_stage_name(int id) const;

  bool is_stage_done_bool(int id) const;
  bool is_stage_partial_bool(int id) const;

private:
  String colors_;   // 54 chars

  // ----------------------------------------------------------
  // Index helpers
  // ----------------------------------------------------------
  int base_index(char face) const;
  char face_center_color(char face) const { return colors_[base_index(face) + 4]; }

  char face_center_color_from(const String &s, char face) const {
    return s[base_index(face) + 4];
  }

  // For a global index 0..53, which face is it on?
  char face_of_index(int idx) const;

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
