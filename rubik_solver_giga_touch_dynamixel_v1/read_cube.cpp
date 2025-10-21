// read_cube.cpp
#include <Arduino.h>
#include "ui_touch.h"
#include "cube_colors.h"
#include "color_sensor.h"
#include "servos.h"
#include <vector>
#include <string>
#include <map>
#include <cctype>

extern bool readingActive;

static inline int faceIndex(char c) {
  switch (toupper(c)) {
    case 'U': return 0;
    case 'R': return 1;
    case 'F': return 2;
    case 'D': return 3;
    case 'L': return 4;
    case 'B': return 5;
    default: return -1;
  }
}
static bool parseFaceIndex(const std::string& s, int& f, int& idx) {
  if (s.size() < 2) return false;
  f = faceIndex(s[0]);
  if (f < 0) return false;
  try {
    idx = std::stoi(s.substr(1));
  } catch (...) { idx = 4; }
  idx = std::max(0, std::min(8, idx));
  return true;
}
static void moveSensorToLevel(int level) {
  float mm = (level == 1 ? 0.f : (level == 2 ? 10.f : 20.f));
  moveArmsVertically(mm);
  delay(200);
}
static char readCellAtAngle(int angle) {
  std::vector<char> r;
  r.reserve(9);
  for (int s : { 0, -3, 3 })
    for (int i = 0; i < 3; i++) {
      rotateBaseToAngle(angle + s);
      delay(30);
      r.push_back(readColorSmoothed(5, 40));
    }
  std::map<char, int> cnt;
  for (char c : r)
    if (c != 'X') cnt[c]++;
  int best = 0;
  char v = 'X';
  for (auto& kv : cnt)
    if (kv.second > best) {
      best = kv.second;
      v = kv.first;
    }
  return v;
}
static std::string remapFace(const std::string& t) {
  char f = 'F';
  for (char c : t) {
    char u = toupper(c);
    if (u == 'U' || u == 'R' || u == 'F' || u == 'D' || u == 'L' || u == 'B') {
      f = u;
      break;
    }
  }
  return std::string(1, f) + "4";
}

void runReadCube() {
  readingActive = true;
  cubeColors.resetColors('X');
  cubeColors.drawToTFT("");

  std::vector<std::string> bands = { "L-Top", "F-Top", "R-Top", "L-Mid", "F-Mid", "R-Mid", "L-Bot", "F-Bot", "R-Bot" };
  std::vector<uint8_t> states(bands.size(), 0);
  int cur = 0;
  if (!bands.empty()) states[0] = 1;
  unsigned long t0 = millis();
  drawProgressBarGeneric(260, bands, cur, states, true);
  drawSolveStats(230, cur, (int)bands.size(), 0);

  const std::string orientations[] = { "lfu", "rfd", "dbr", "ubl" };
  const int levels[] = { 2, 3 };

  for (int o = 0; o < 4 && readingActive; o++) {
    for (int l = 0; l < 2 && readingActive; l++) {
      moveSensorToLevel(levels[l]);
      struct Cell {
        const char* name;
        int ang;
      } cells[9] = { { "lub", -90 }, { "lu", -60 }, { "luf", -30 }, { "ful", -10 }, { "fu", 0 }, { "fur", 10 }, { "ruf", 30 }, { "ru", 60 }, { "rub", 90 } };
      for (int i = 0; i < 9 && readingActive; i++) {
        std::string m = remapFace(cells[i].name);
        int f = -1, idx = -1;
        parseFaceIndex(m, f, idx);
        char col = readCellAtAngle(cells[i].ang);
        if (f >= 0) {
          char ex = cubeColors.getColorURFDLB(f, idx);
          cubeColors.setColorURFDLB(f, idx, ex == 'X' ? col : (ex != col && col != 'X' ? 'E' : ex));
        }
        cubeColors.drawToTFT(std::string(1, m.empty() ? 'F' : m[0]) + "4");
      }
      cur++;
      if (cur < (int)states.size()) states[cur] = 1;
      drawProgressBarGeneric(260, bands, cur, states, true);
      drawSolveStats(230, cur, (int)bands.size(), millis() - t0);
      delay(200);
    }
  }
  readingActive = false;
}
