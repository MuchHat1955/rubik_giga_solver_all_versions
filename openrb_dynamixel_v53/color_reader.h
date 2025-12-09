#pragma once

#include <Arduino.h>
#include "ori.h"

class CubeOri;

class CubeColorReader {
public:
  using read_color_cb_t = char (*)(int slot_index);  // sensor returns 1 color per slot

  CubeColorReader(CubeOri &ori, read_color_cb_t cb);

  void clear();
  bool read_full_cube();
  String get_cube_colors_string() const;
  void apply_moves(const String &moves);
  void print_cube_colors_string();

private:
  CubeOri &ori_;
  read_color_cb_t cb_;
  char colors_[54];  // u r f d l b, 9 stickers each

  void fill_unknown_();
  void apply_slot_to_face_(char face, int slot, char color, bool mirrored);
  int face_base_index_(char face) const;

  bool process_step_(int step_index,
                     const char *robot_move,
                     const char *face,
                     bool mirrored,
                     const char *order);

  void rotate_face(char face, char dir);
  void print_face_compact_(char face) const;
};