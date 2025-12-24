#pragma once

#include <Arduino.h>
#include "ori.h"

class CubeOri;


#define SCAN_MODE_FULL 54    // 54 stickers
#define SCAN_MODE_BOTTOM 15  // 15 stickers
#define SCAN_MODE_CENTERS 1  // 4 stickers
#define SCAN_MODE_SOLVED 55  // 54 stickers

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

  void clear_color_reader();
  bool set_colors(String colors);
  bool read_cube_full();
  bool read_cube_bottom();
  bool read_cube_f_and_r_centers();
  bool read_cube_solved();

  String get_justread_color_string_face(char face) const;
  String get_justread_color_string_faces() const;
  String get_justread_color_string_54() const;

private:
  CubeOri &ori_;
  read_color_cb_t color_sensor_cb_;
  char colors_justread_54[54];  // u r f d l b, 9 stickers each

  void fill_unknown_();
  void update_color_string(char face, int offset, char color);
  void apply_slot_to_face_(char face, int slot, char color, bool mirrored);
  bool read_cube(int scan_mode);
  bool process_color_scan_step_(int step_index,
                                const char *robot_move,
                                const char *face,
                                bool mirrored,
                                const char *order);

  void rotate_face(char face, char dir);
};

char color_to_face(char color);
char face_to_color(char face);
String fill_colors_if_top_two_layers_solved(const String &in54);
String fill_solved_cube();