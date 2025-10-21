// scramble.cpp
#include <Arduino.h>
#ifdef abs
#undef abs
#endif
#include "ui_touch.h"
#include "cube_colors.h"
#include <vector>
#include <string>
#include <random>
#include <cctype>

void runCubeStandardMove(const std::string& move);
static const char FACES[6] = {'U','R','F','D','L','B'};
static const char* MODS[3] = {"", "'", "2"};
static inline int axisOf(char f){ return (f=='U'||f=='D')?0:(f=='L'||f=='R')?1:2; }
static std::mt19937& rng(){ static std::mt19937 g(0xA5A5BEEF ^ (uint32_t)millis()); return g; }

static std::string genOneMove(char lastFace, int lastAxis){
  for(;;){
    char f = FACES[std::uniform_int_distribution<int>(0,5)(rng())];
    if(f==lastFace) continue;
    if(axisOf(f)==lastAxis) continue;
    const char* m = MODS[std::uniform_int_distribution<int>(0,2)(rng())];
    return std::string(1,f)+m;
  }
}

static std::vector<std::string> generateScramble(int len){
  std::vector<std::string> mv; mv.reserve(len); char lf=0; int la=-1;
  for(int i=0;i<len;i++){ std::string s=genOneMove(lf,la); lf=s[0]; la=axisOf(lf); mv.push_back(s); }
  return mv;
}

void runScramble(int len){
  auto moves = generateScramble(len);
  cubeColors.drawToTFT("");
  unsigned long t0=millis();
  for(int i=0;i<(int)moves.size();++i){
    runCubeStandardMove(moves[i]);
    cubeColors.drawToTFT("");
    lv_timer_handler();
    unsigned long ws=millis();
    while(millis()-ws<350){ lv_timer_handler(); delay(10); }
  }
}
