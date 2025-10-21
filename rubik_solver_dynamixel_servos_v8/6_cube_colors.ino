/********************************************************************
 * RUN READ SEQUENCE WITH PROGRESS BAR + COLOR DETECTION
 ********************************************************************/

// -----------------------------------------------------
// RubikCubeColors CLASS
// -----------------------------------------------------

class RubikCubeColors {
public:
  char face[6][9];
  char faceToColor[6];  // WRGYOB mapping

  RubikCubeColors() { resetColors('X'); setDefaultCenters(); }

  void resetColors(char fill = 'X') {
    for (int f = 0; f < 6; ++f)
      for (int i = 0; i < 9; ++i)
        face[f][i] = fill;
  }

  void setDefaultCenters() {
    faceToColor[U_] = 'W';
    faceToColor[R_] = 'R';
    faceToColor[F_] = 'G';
    faceToColor[D_] = 'Y';
    faceToColor[L_] = 'O';
    faceToColor[B_] = 'B';
    face[U_][4] = 'U'; face[R_][4] = 'R'; face[F_][4] = 'F';
    face[D_][4] = 'D'; face[L_][4] = 'L'; face[B_][4] = 'B';
  }

  void setColorURFDLB(int f, int idx, char val) {
    if (f < 0 || f > 5 || idx < 0 || idx > 8) return;
    face[f][idx] = val;
  }

  char getColorURFDLB(int f, int idx) const {
    if (f < 0 || f > 5 || idx < 0 || idx > 8) return 'X';
    return face[f][idx];
  }

  void applyMove(const String &move) {
    Serial.printf("Applying move: %s\n", move.c_str());
    // TODO: actual rotation logic
  }

  void drawToTFT(const String &highlightCell = "") const {
    const int faceSize = 3;
    const int cellSize = 22;
    const int pad = 3;
    const int gap = 6;
    const int cubeWidth = (faceSize * cellSize) + (faceSize - 1) * pad;
    const int cubeHeight = (faceSize * cellSize) + (faceSize - 1) * pad;

    int totalWidth = cubeWidth * 4 + gap * 3;
    int startX = (tft.width() - totalWidth) / 2;
    int startY = 70;

    auto faceOrigin = [&](char face) -> std::pair<int, int> {
      switch (face) {
        case 'U': return {startX + cubeWidth + gap, startY};
        case 'L': return {startX, startY + cubeHeight + gap};
        case 'F': return {startX + cubeWidth + gap, startY + cubeHeight + gap};
        case 'R': return {startX + 2 * (cubeWidth + gap), startY + cubeHeight + gap};
        case 'B': return {startX + 3 * (cubeWidth + gap), startY + cubeHeight + gap};
        case 'D': return {startX + cubeWidth + gap, startY + 2 * (cubeHeight + gap)};
        default:  return {startX, startY};
      }
    };

    auto colorForChar = [&](char c) -> uint16_t {
      switch (toupper(c)) {
        case 'W': return ST7796S_WHITE;
        case 'R': return ST7796S_RED;
        case 'G': return ST7796S_GREEN;
        case 'Y': return ST7796S_YELLOW;
        case 'O': return ST7796S_ORANGE;
        case 'B': return ST7796S_BLUE;
        case 'E': return ST7796S_MAGENTA;
        default:  return ST7796S_DARKGREY;
      }
    };

    const char faces[] = {'U','L','F','R','B','D'};
    for (char faceChar : faces) {
      int fIdx = faceIndex(faceChar);
      auto [fx, fy] = faceOrigin(faceChar);

      for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
          int idx = row * 3 + col;
          char c = face[fIdx][idx];
          int x = fx + col * (cellSize + pad);
          int y = fy + row * (cellSize + pad);

          tft.fillRect(x, y, cellSize, cellSize, colorForChar(c));
          tft.drawRect(x, y, cellSize, cellSize, ST7796S_BLACK);
        }
      }
    }

    if (highlightCell.length() >= 2) {
      char face = highlightCell[0];
      auto [fx, fy] = faceOrigin(face);
      int idx = highlightCell.substring(1).toInt();
      int row = idx / 3;
      int col = idx % 3;
      int x = fx + col * (cellSize + pad);
      int y = fy + row * (cellSize + pad);
      tft.drawRect(x - 2, y - 2, cellSize + 4, cellSize + 4, ST7796S_WHITE);
    }
  }

private:
  enum { U_ = 0, R_ = 1, F_ = 2, D_ = 3, L_ = 4, B_ = 5 };
  static int faceIndex(char c) {
    switch (toupper(c)) {
      case 'U': return U_; case 'R': return R_; case 'F': return F_;
      case 'D': return D_; case 'L': return L_; case 'B': return B_;
      default: return -1;
    }
  }
};

// =============================================================
// ---------------- PROGRESS BAR (Generic) ---------------------

enum ProgressState : uint8_t {
  STATE_PENDING = 0,
  STATE_ACTIVE  = 1,
  STATE_DONE_OK = 2,
  STATE_WARNING = 3,
  STATE_ERROR   = 4
};

void drawProgressBarGeneric(Adafruit_ST7796S_kbv &tft,
                            int y_offset,
                            const std::vector<String> &items,
                            int currentIndex,
                            const std::vector<uint8_t> &states,
                            bool showCounter = true)
{
  const int btnW = 42, btnH = 26, spacing = 6, radius = 5;
  const int maxVisible = 10;
  int total = items.size();
  if (total == 0) return;

  int startIndex = 0;
  if (currentIndex >= maxVisible - 2)
    startIndex = currentIndex - (maxVisible - 3);
  if (startIndex < 0) startIndex = 0;
  if (startIndex > total - maxVisible)
    startIndex = max(0, total - maxVisible);

  int visibleCount = min(maxVisible, total - startIndex);
  int totalWidth = visibleCount * (btnW + spacing);
  int startX = (tft.width() - totalWidth) / 2;
  int y = y_offset;

  tft.fillRect(0, y, tft.width(), btnH + 24, ST7796S_BLACK);

  if (showCounter) {
    String counter = String(currentIndex + 1) + " / " + String(total);
    tft.setTextSize(1);
    tft.setTextColor(ST7796S_LIGHTGREY);
    int cw = counter.length() * 6;
    tft.setCursor((tft.width() - cw) / 2, y - 12);
    tft.print(counter);
  }

  if (startIndex > 0) {
    tft.setCursor(4, y + btnH / 2 - 4);
    tft.setTextColor(ST7796S_DARKGREY);
    tft.print("...");
  }
  if (startIndex + visibleCount < total) {
    tft.setCursor(tft.width() - 20, y + btnH / 2 - 4);
    tft.setTextColor(ST7796S_DARKGREY);
    tft.print("...");
  }

  for (int i = 0; i < visibleCount; i++) {
    int idx = startIndex + i;
    String label = items[idx];
    int x = startX + i * (btnW + spacing);

    uint16_t color = ST7796S_LIGHTGREY;
    if (states.size() > idx) {
      switch (states[idx]) {
        case STATE_ACTIVE:  color = ST7796S_BLUE; break;
        case STATE_DONE_OK: color = ST7796S_GREEN; break;
        case STATE_WARNING: color = ST7796S_YELLOW; break;
        case STATE_ERROR:   color = ST7796S_RED; break;
      }
    }

    tft.fillRoundRect(x, y, btnW, btnH, radius, color);
    if (idx == currentIndex)
      tft.drawRoundRect(x - 1, y - 1, btnW + 2, btnH + 2, radius + 2, ST7796S_WHITE);

    tft.setTextColor(ST7796S_BLACK);
    int tw = label.length() * 6;
    tft.setCursor(x + (btnW - tw) / 2, y + (btnH / 2) - 4);
    tft.print(label);
  }
}

// =============================================================
// ---------------- SOLVER CONTROL LOGIC -----------------------

std::vector<String> solveMoves = {
  "F", "R", "U", "R'", "U'", "F'", "L", "D", "L'", "U2",
  "B", "R2", "F2", "L'", "D'", "U", "R", "F", "L2", "B2", "U2"
};

std::vector<uint8_t> moveStates(solveMoves.size(), 0);
int currentMove = 0;
const unsigned long MOVE_PAUSE_MS = 500;
bool solvingActive = false;
RubikCubeColors cubeColors;

// =============================================================
// Helpers
String convertStandardMove(const String &s) {
  if (s.endsWith("'")) return s.substring(0,1) + "-";
  if (s.endsWith("2")) return s.substring(0,1) + "++";
  return s.substring(0,1) + "+";
}

bool runCubeStandardMove(const String &move) {
  Serial.printf("Executing hardware move: %s\n", move.c_str());
  delay(150);
  return true;
}

void drawSolveStats(Adafruit_ST7796S_kbv &tft, int y, int current, int total, unsigned long elapsed_ms) {
  tft.setTextSize(2);
  tft.setTextColor(ST7796S_WHITE, ST7796S_BLACK);
  tft.fillRect(0, y, tft.width(), 24, ST7796S_BLACK);

  float pct = (total > 0) ? (100.0f * (float)(current + 1) / total) : 0;
  float elapsed_s = elapsed_ms / 1000.0f;

  char buf[64];
  snprintf(buf, sizeof(buf), "Move %2d/%d  (%.0f%%)  %.1fs",
           current + 1, total, pct, elapsed_s);

  int textW = strlen(buf) * 12;
  int x = (tft.width() - textW) / 2;
  tft.setCursor(x, y + 4);
  tft.print(buf);
}


