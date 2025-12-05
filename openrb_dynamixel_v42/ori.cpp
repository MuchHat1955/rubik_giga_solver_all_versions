#include "ori.h"

#include "RubikOrientation.h"

// ------------------------------------------------------------
// Constructor
// ------------------------------------------------------------
RubikOrientation::RubikOrientation() {
  reset();
}

// ------------------------------------------------------------
// Reset orientation + color strings
// ------------------------------------------------------------
void RubikOrientation::reset() {
  face_map['F'] = 'F';
  face_map['B'] = 'B';
  face_map['R'] = 'R';
  face_map['L'] = 'L';
  face_map['U'] = 'U';
  face_map['D'] = 'D';

  // Initialize all faces to "XXXXXXXXX"
  for (int i = 0; i < 6; i++)
    face_str[i] = "XXXXXXXXX";
}

// ------------------------------------------------------------
// Log orientation mapping
// ------------------------------------------------------------
void RubikOrientation::logOrientation() const {
  Serial.print("[ORI] ");
  Serial.print("F->"); Serial.print(face_map.at('F')); Serial.print("  ");
  Serial.print("R->"); Serial.print(face_map.at('R')); Serial.print("  ");
  Serial.print("U->"); Serial.print(face_map.at('U')); Serial.print("  ");
  Serial.print("B->"); Serial.print(face_map.at('B')); Serial.print("  ");
  Serial.print("L->"); Serial.print(face_map.at('L')); Serial.print("  ");
  Serial.print("D->"); Serial.print(face_map.at('D'));
  Serial.println();
}

// ------------------------------------------------------------
// Map a logical move "F+" into physical "R+"
// ------------------------------------------------------------
String RubikOrientation::mapMove(const String &move) {
  if (move.length() < 2) return move;

  char face   = move.charAt(0);
  char suffix = move.charAt(1);

  char phys = face_map[face];

  String out;
  out += phys;
  out += suffix;
  return out;
}

// ------------------------------------------------------------
// Rotation dispatcher
// ------------------------------------------------------------
void RubikOrientation::rotate(int angle_x, int angle_y, int angle_z) {
  applyRotationX(angle_x);
  applyRotationY(angle_y);
  applyRotationZ(angle_z);
}

// ------------------------------------------------------------
// Normalize angle: ensure only 0, 90, -90, 180
// ------------------------------------------------------------
int RubikOrientation::normalize(int a) {
  a %= 360;

  if (a == 270)  return -90;
  if (a == -270) return 90;
  if (a == -180) return 180;

  return a;  // 0, 90, -90, 180 remain valid
}

// ------------------------------------------------------------
// Apply X rotation
// ------------------------------------------------------------
void RubikOrientation::applyRotationX(int a) {
  a = normalize(a);
  if (a == 0) return;

  if (a == 90) {
    remap({ {'U','F'}, {'F','D'}, {'D','B'}, {'B','U'} });
  } else if (a == -90) {
    remap({ {'U','B'}, {'B','D'}, {'D','F'}, {'F','U'} });
  } else if (a == 180) {
    remap({ {'U','D'}, {'D','U'}, {'F','B'}, {'B','F'} });
  }
}

// ------------------------------------------------------------
// Apply Y rotation
// ------------------------------------------------------------
void RubikOrientation::applyRotationY(int a) {
  a = normalize(a);
  if (a == 0) return;

  if (a == 90) {
    remap({ {'F','L'}, {'L','B'}, {'B','R'}, {'R','F'} });
  } else if (a == -90) {
    remap({ {'F','R'}, {'R','B'}, {'B','L'}, {'L','F'} });
  } else if (a == 180) {
    remap({ {'F','B'}, {'B','F'}, {'R','L'}, {'L','R'} });
  }
}

// ------------------------------------------------------------
// Apply Z rotation
// ------------------------------------------------------------
void RubikOrientation::applyRotationZ(int a) {
  a = normalize(a);
  if (a == 0) return;

  if (a == 90) {
    remap({ {'U','R'}, {'R','D'}, {'D','L'}, {'L','U'} });
  } else if (a == -90) {
    remap({ {'U','L'}, {'L','D'}, {'D','R'}, {'R','U'} });
  } else if (a == 180) {
    remap({ {'U','D'}, {'D','U'}, {'R','L'}, {'L','R'} });
  }
}

// ------------------------------------------------------------
// Remap helper
// ------------------------------------------------------------
void RubikOrientation::remap(const std::map<char,char> &m) {
  std::map<char,char> new_map = face_map;
  for (auto &kv : m)
    new_map[kv.first] = face_map[kv.second];
  face_map = new_map;
}

// ------------------------------------------------------------
// Convert face char → internal index
// ------------------------------------------------------------
int RubikOrientation::faceIndex(char f) const {
  switch (f) {
    case 'F': return 0;
    case 'R': return 1;
    case 'U': return 2;
    case 'B': return 3;
    case 'L': return 4;
    case 'D': return 5;
  }
  return -1;
}

// ------------------------------------------------------------
// Update front face (top 2 rows only)
// ------------------------------------------------------------
void RubikOrientation::updateFrontColors(const char colors[6]) {
  char logical_face = face_map.at('F');
  int idx = faceIndex(logical_face);
  if (idx < 0) return;

  String &f = face_str[idx];

  f.setCharAt(0, colors[0]);
  f.setCharAt(1, colors[1]);
  f.setCharAt(2, colors[2]);
  f.setCharAt(3, colors[3]);
  f.setCharAt(4, colors[4]);
  f.setCharAt(5, colors[5]);

  Serial.print("[ORI] Updated colors for logical face ");
  Serial.println(logical_face);
}

// ------------------------------------------------------------
// Manually update any single sticker
// ------------------------------------------------------------
void RubikOrientation::updateSticker(char face, int index, char color) {
  int idx = faceIndex(face);
  if (idx < 0 || index < 0 || index > 8) return;

  face_str[idx].setCharAt(index, color);
}

// ------------------------------------------------------------
// Produce Kociemba format: U R F D L B
// ------------------------------------------------------------
String RubikOrientation::getCubeColors() const {
  const String &F = face_str[0];
  const String &R = face_str[1];
  const String &U = face_str[2];
  const String &B = face_str[3];
  const String &L = face_str[4];
  const String &D = face_str[5];

  String out;
  out.reserve(54);

  out += U;
  out += R;
  out += F;
  out += D;
  out += L;
  out += B;

  return out;
}
