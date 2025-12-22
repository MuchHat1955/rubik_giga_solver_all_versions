#ifndef COLOR_ANALYZER_H
#define COLOR_ANALYZER_H

#include <Arduino.h>
#include "utils.h"
#include "log.h"

class ColorAnalyzer {
public:
  ColorAnalyzer();

  // Public wrappers
  void set_colors(const String &colors);
  void clear_color_analyzer();
  bool apply_cube_move(String m);
  bool is_color_string_valid_bool() const;
  String get_standard_color_string_54();
  bool is_string_fixable_bool() const;
  bool try_fix_color_string(String &fixed_out) const;
  String get_string_check_log() const;

  // Stage detection
  const char *get_stage_name(int id) const;
  int get_stage_count() const {
    return 7;
  }
  bool is_stage_done_bool(int id) const;
  bool is_stage_partial_bool(int id) const;

  // Diagnostic
  String get_last_error() const {
    return last_error_;
  }

  // ---- Smart fix entrypoint ----
  String fix_string_smart() const;

  // ---- Fix unknown '.' stickers ----
  String fix_string_unknown_1() const;
  String fix_string_unknown_2() const;
  String fix_string_unknown_3() const;

  // ---- Fix count errors ----
  String fix_string_count_1() const;
  String fix_string_count_2() const;

  // ---- Fix r<->o confusion ----
  String fix_string_ro_1() const;
  String fix_string_ro_2() const;

  // ---- General single-sticker try ----
  String fix_string_general_1() const;

private:
  String colors_standard_orientation_54_;
  mutable String last_error_;

  // --- Index / helper methods ---
  void compute_color_counts_from(const String &s, int out[256]) const;

  // --- Validation core ---
  bool is_color_string_valid_impl(const String &s) const;
  bool centers_correct_from(const String &s) const;
  bool valid_color_counts_from(const String &s) const;
  bool edges_corners_color_consistent_from(const String &s) const;

  // --- Solvability ---
  char get_edge_reference_color(const String &s, int piece_id) const;
  bool check_edge_flip_parity_simplified(const String &s) const;

  // --- Solved state checks ---
  bool face_solved_bool(char face) const;
  bool top_layer_solved_bool() const;
  bool middle_layer_solved_bool() const;
  bool bottom_cross_solved_bool() const;
  bool bottom_layer_solved_bool() const;

  // ---- Count helpers ----
  static void count_colors_and_unknowns(const String &s, int counts[256], int &unknown_count);
  static void count_colors(const String &s, int counts[256]);
};

char oposite_color(char color);
char get_stickercolor_from_color_string_54(String color_54, char face, int slot);
bool is_valid_move(String &token);
bool is_valid_color(char color);
int face_base_index(char face);
void sort_pair(char &a, char &b);
void sort_triple(char &a, char &b, char &c);
char face_center_color_from(const String &s, char face);

// Global instance
extern ColorAnalyzer color_analyzer;

#endif
