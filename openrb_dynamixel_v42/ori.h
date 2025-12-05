#pragma once
#include <Arduino.h>
#include "servos.h"
#include "utils.h"
#include "vertical_kinematics.h"

#pragma once
#include <Arduino.h>
#include <map>

class RubikOrientation {
public:
  RubikOrientation();

  // Reset orientation and color data
  void reset();

  // Orientation rotation (angles must be 0, 90, -90, 180)
  void rotate(int angle_x, int angle_y, int angle_z);

  // Convert logical move "F+" into physical "R+"
  String mapMove(const String &move);

  // Log orientation mapping
  void logOrientation() const;

  // Update the top two rows (6 stickers) of the *current physical front*
  void updateFrontColors(const char colors[6]);

  // Update any sticker explicitly: logical face & sticker index 0‒8
  void updateSticker(char face, int index, char color);

  // Return full 54-character cube string (U R F D L B order)
  String getCubeColors() const;

private:
  // Face order: F R U B L D (internal storage)
  String face_str[6];

  // Orientation mapping: logical → physical
  std::map<char, char> face_map;

  // Helpers
  int faceIndex(char f) const;
  int normalize(int a);

  // Rotation application
  void applyRotationX(int a);
  void applyRotationY(int a);
  void applyRotationZ(int a);

  // Remap helper
  void remap(const std::map<char,char> &m);
};
