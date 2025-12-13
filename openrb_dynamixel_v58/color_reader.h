#pragma once

#include <Arduino.h>
#include "ori.h"

class CubeOri;

struct color_map_step_t {
  const char *robot_move;  // movement: "y_plus", "y_minus", "z_minus", "y_180", …
  const char *face;        // face to read: "f","r","u","", …
  bool mirrored;           // true = bottom band (mirror), false = normal
  const char *order;       // slot order: "236541" or "231"
};

class CubeColorReader {
public:
  using read_color_cb_t = char (*)(int slot_index);  // sensor returns 1 color per slot

  CubeColorReader(CubeOri &ori, read_color_cb_t cb);

  void clear();
  bool read_cube_full();
  bool read_cube_bottom();
  void fill_solved_cube();

  String get_cube_colors_string() const;
  void apply_moves(const String &moves);
  void print_cube_colors_string();
  void print_face_compact(char face) const;
  void update_color_string(char face, int offset, char color);


private:
  CubeOri &ori_;
  read_color_cb_t cb_;
  char colors_[54];  // u r f d l b, 9 stickers each

  void fill_unknown_();
  void fill_solved_cube_top2layers_();
  void apply_slot_to_face_(char face, int slot, char color, bool mirrored);
  int face_base_index_(char face) const;
  bool read_cube(bool all_cube_vs_just_bottom_layer);

  bool process_step_(int step_index,
                     const char *robot_move,
                     const char *face,
                     bool mirrored,
                     const char *order);

  void rotate_face(char face, char dir);
};