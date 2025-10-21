// solve.cpp
#include <Arduino.h>
#include "ui_touch.h"
#include "cube_colors.h"
#include <vector>
#include <string>

extern bool solvingActive;
void runCubeStandardMove(const std::string &moveName);

std::vector<std::string> solveMoves={
  "F","R","U","R'","U'","F'","L","D","L'","U2","B","R2","F2","L'","D'","U","R","F","L2","B2","U2"
};
std::vector<uint8_t> moveStates(solveMoves.size(),0);
int currentMove=0;
const unsigned long MOVE_PAUSE_MS=400;

void runSolveCube(){
  solvingActive=true;
  std::fill(moveStates.begin(),moveStates.end(),0);
  if(!solveMoves.empty()) moveStates[0]=1;
  currentMove=0;
  unsigned long t0=millis();
  drawProgressBarGeneric(260,solveMoves,currentMove,moveStates,true);
  drawSolveStats(230,0,(int)solveMoves.size(),0);
  // tft_present_lvgl();

  while(solvingActive && currentMove<(int)solveMoves.size()){
    const std::string &mv=solveMoves[currentMove];
    runCubeStandardMove(mv);
    cubeColors.applyMove(mv);
    cubeColors.drawToTFT(std::string(1,(char)toupper(mv[0]))+"4");
    moveStates[currentMove]=2;
    unsigned long ws=millis();
    while(millis()-ws<MOVE_PAUSE_MS){ lv_timer_handler(); delay(10); }
    if(!solvingActive) break;
    currentMove++; if(currentMove<(int)moveStates.size()) moveStates[currentMove]=1;
    drawProgressBarGeneric(260,solveMoves,currentMove,moveStates,true);
    drawSolveStats(230,currentMove,(int)solveMoves.size(), millis()-t0);
    // tft_present_lvgl();
  }
  solvingActive=false;
}

void runCubeStandardMove(const std::string &moveName){
  // TODO
}
