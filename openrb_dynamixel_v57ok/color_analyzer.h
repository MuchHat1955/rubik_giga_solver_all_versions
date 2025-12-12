#ifndef COLOR_ANALYZER_H
#define COLOR_ANALYZER_H

#include <Arduino.h>
#include "utils.h"

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
    int get_stage_count() const { return 7; }
    bool is_stage_done_bool(int id) const;
    bool is_stage_partial_bool(int id) const;
    
    // Diagnostic
    String get_last_error() const { return last_error_; }

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
};

// Global instance (defined in color_analyzer.cpp)
extern ColorAnalyzer color_analyzer;

#endif // COLOR_ANALYZER_H
