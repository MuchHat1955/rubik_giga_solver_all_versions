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
  bool is_color_string_valid_bool() const;  // Includes necessary checks
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

  // ---- Public "smart fix" entrypoint ----
  String fix_string_smart() const;

  // ---- Fix unknown '.' stickers (exactly 1/2/3 unknowns) ----
  String fix_string_unknown_1() const;
  String fix_string_unknown_2() const;
  String fix_string_unknown_3() const;

  // ---- Fix count errors (one color too many / one too few) ----
  String fix_string_count_1() const;
  String fix_string_count_2() const;

  // ---- Fix common confusion r<->o (one or two swaps) ----
  String fix_string_ro_1() const;
  String fix_string_ro_2() const;

  // ---- General single-sticker try among centers ----
  String fix_string_general_1() const;

  // ---- Center inference / scheme rotation / orientation helpers ----
  String infer_centers_from_partial(const String &s) const;
  String rotate_to_standard_scheme(const String &s) const;

  bool is_orientation_string_valid_bool(const String &s) const;
  String get_orientation_string(const String &s) const;

private:
  String colors_;
  mutable String last_error_;

  // --- Index/Helper Methods ---
  int base_index(char face) const;
  void sort_pair(char &a, char &b) const;
  void sort_triple(char &a, char &b, char &c) const;
  char face_center_color_from(const String &s, char face) const;
  void compute_color_counts_from(const String &s, int out[256]) const;

  // --- Validation Core ---
  bool is_color_string_valid_impl(const String &s) const;
  bool centers_correct_from(const String &s) const;
  bool valid_color_counts_from(const String &s) const;
  bool edges_corners_color_consistent_from(const String &s) const;

  // --- Solvability: NECESSARY Condition Checks ---
  char get_edge_reference_color(const String &s, int piece_id) const;
  bool check_edge_flip_parity_simplified(const String &s) const;

  // --- Solved State Checks ---
  bool face_solved_bool(char face) const;
  bool top_layer_solved_bool() const;
  bool middle_layer_solved_bool() const;
  bool bottom_cross_solved_bool() const;
  bool bottom_layer_solved_bool() const;

  // ---- Helpers that must exist elsewhere (declared here for compilation) ----
  bool is_color_string_valid_bool() const;
  bool are_centers_valid_scheme_bool() const;
  bool is_color_string_valid_impl(const String &s) const;

  // If you already have a different signature / location, adjust accordingly.
  static char face_center_color_from(const String &s, char face_char);

  // Rotation / orientation helpers referenced by your .cpp
  static void apply_face_rotation(const String &in, String &out, const int face_map[6]);
  static int opposite_face(int f);
  static int face_of_center_index(int idx);

  static bool extract_centers(const String &s, char centers[6]);
  static void apply_face_rotation_centers(const char in[6], char out[6], const int face_map[6]);

  // Count helpers used by fixers
  static void count_colors(const String &s, int counts[256]);
  static void count_colors_and_unknowns(const String &s, int counts[256], int &unknown_count);

  // Indices/maps used by center inference / rotations.
  // These are referenced in the code you pasted; if they already exist elsewhere,
  // you can delete them here and keep only the declarations you need.
  static const int center_idx[6];  // {4,13,22,31,40,49}
  static const int opp_face[3];    // {3,4,5} meaning pairs: (0,3)(1,4)(2,5)
  static const int face_rotations[24][6];
};

// Global instance (defined in color_analyzer.cpp)
extern ColorAnalyzer color_analyzer;

#endif  // COLOR_ANALYZER_H
