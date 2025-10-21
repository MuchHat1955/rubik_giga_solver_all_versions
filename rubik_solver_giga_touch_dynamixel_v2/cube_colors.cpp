// cube_colors.cpp
#include "cube_colors.h"
#include "ui_touch.h"
#include <cctype>
#include <algorithm>


RubikCubeColors::RubikCubeColors(){ resetColors('X'); setDefaultCenters(); }
void RubikCubeColors::resetColors(char fill){ for(int f=0;f<6;f++)for(int i=0;i<9;i++) face[f][i]=fill; }
void RubikCubeColors::setDefaultCenters(){
  faceToColor[0]='W'; faceToColor[1]='R'; faceToColor[2]='G'; faceToColor[3]='Y'; faceToColor[4]='O'; faceToColor[5]='B';
  face[0][4]='U'; face[1][4]='R'; face[2][4]='F'; face[3][4]='D'; face[4][4]='L'; face[5][4]='B';
}
void RubikCubeColors::drawToTFT(const std::string& hi) const { cube_draw_to_lvgl(face, faceToColor, hi); }
void RubikCubeColors::setColorURFDLB(int f,int idx,char v){ if(f>=0&&f<6&&idx>=0&&idx<9) face[f][idx]=v; }
char RubikCubeColors::getColorURFDLB(int f,int idx) const { if(f>=0&&f<6&&idx>=0&&idx<9) return face[f][idx]; return 'X'; }
void RubikCubeColors::applyMove(const std::string& move){ (void)move; /* TODO: real cube rotation */ }
RubikCubeColors cubeColors;
