// cube_colors.h
#pragma once
#include <string>
class RubikCubeColors {
public:
  char face[6][9];
  char faceToColor[6];
  RubikCubeColors();
  void resetColors(char fill='X');
  void setDefaultCenters();
  void drawToTFT(const std::string& highlight="") const;
  void setColorURFDLB(int f,int idx,char v);
  char getColorURFDLB(int f,int idx) const;
  void applyMove(const std::string& move);
};
extern RubikCubeColors cubeColors;
