/********************************************************************
 * DRAW CUBE COLORS AND PROGRESS BAR DURING READING AND SOLVING
 ********************************************************************/

void drawCubeOnTFT(Adafruit_ST7796S_kbv &tft, const String &highlightCell = "") {
    const int faceSize = 3;
    const int cellSize = 22;     // each color square in pixels
    const int pad = 3;           // spacing between squares
    const int gap = 6;           // spacing between faces
    const int cubeWidth = (faceSize * cellSize) + (faceSize - 1) * pad;
    const int cubeHeight = (faceSize * cellSize) + (faceSize - 1) * pad;

    // Layout:
    //        [U]
    // [L] [F] [R] [B]
    //        [D]

    int totalWidth = cubeWidth * 4 + gap * 3;
    int totalHeight = cubeHeight * 3 + gap * 2;

    int startX = (tft.width() - totalWidth) / 2;
    int startY = 70;  // leave space for title/subtitle

    // --- Helper: map face name to on-screen coordinates ---
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

    // --- Helper: map color char to display color ---
    auto colorForChar = [&](char c) -> uint16_t {
        switch (toupper(c)) {
            case 'W': return ST7796S_WHITE;
            case 'R': return ST7796S_RED;
            case 'G': return ST7796S_GREEN;
            case 'Y': return ST7796S_YELLOW;
            case 'O': return ST7796S_ORANGE;
            case 'B': return ST7796S_BLUE;
            case 'X': return ST7796S_DARKGREY;
            case 'E': return ST7796S_MAGENTA;  // error
            default:  return ST7796S_BLACK;
        }
    };

    // --- Draw all faces ---
    char faces[] = {'U','L','F','R','B','D'};
    for (char face : faces) {
        auto [fx, fy] = faceOrigin(face);

        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                String cellKey = String(face) + String(row*3+col);
                char c = colors.count(cellKey) ? colors[cellKey] : 'X';
                int x = fx + col * (cellSize + pad);
                int y = fy + row * (cellSize + pad);

                tft.fillRect(x, y, cellSize, cellSize, colorForChar(c));
                tft.drawRect(x, y, cellSize, cellSize, ST7796S_BLACK);
            }
        }
    }

    // --- Optional highlight for last updated cell ---
    if (highlightCell != "") {
        char face = highlightCell[0];
        auto [fx, fy] = faceOrigin(face);

        // assume highlightCell is formatted like "F4" meaning row=1,col=1
        int idx = highlightCell.substring(1).toInt();
        int row = idx / 3;
        int col = idx % 3;
        int x = fx + col * (cellSize + pad);
        int y = fy + row * (cellSize + pad);

        tft.drawRect(x - 2, y - 2, cellSize + 4, cellSize + 4, ST7796S_WHITE);
        delay(60);
        tft.drawRect(x - 2, y - 2, cellSize + 4, cellSize + 4, ST7796S_BLACK);
    }
}


// display in real time as you read

void RubikColor::setColor(const String &cell, char color, Adafruit_ST7796S_kbv &tft) {
    char upColor = toupper(color);
    if (upColor != 'W' && upColor != 'R' && upColor != 'G' &&
        upColor != 'Y' && upColor != 'O' && upColor != 'B' &&
        upColor != 'X' && upColor != 'E') {
        upColor = 'X';
    }

    // if re-read differs from previous → mark as error
    if (colors.count(cell) && colors[cell] != 'X' && colors[cell] != upColor)
        upColor = 'E';

    colors[cell] = upColor;

    drawCubeOnTFT(tft, cell);
}

example

RubikColor cube;
...
cube.setColor("F4", 'G', tft);   // reads one color and updates display

/********************************************************************
 * BAND FOR READ PROGRESS
 ********************************************************************/

void drawColorReadProgress(Adafruit_ST7796S_kbv &tft, int y_offset,
                           const std::vector<String> &bands,
                           const std::map<String, String> &bandState,
                           const std::set<String> &readTwice)
{
  // --- Layout ---
  int btnW = 44;          // each button 40 + 4px spacing
  int btnH = 24;
  int spacing = 4;
  int totalW = bands.size() * (btnW + spacing);
  int startX = (tft.width() - totalW) / 2;
  int y = y_offset;

  tft.setTextSize(1);
  tft.setTextColor(ST7796S_BLACK);

  for (int i = 0; i < (int)bands.size(); i++) {
    String key = bands[i];
    int x = startX + i * (btnW + spacing);

    // Determine color
    uint16_t fillColor = ST7796S_DARKGREY;  // default: not done
    String state = bandState.count(key) ? bandState.at(key) : "pending";

    if (state == "reading")      fillColor = ST7796S_BLUE;
    else if (state == "done")    fillColor = ST7796S_GREEN;
    else if (state == "warning") fillColor = ST7796S_YELLOW;

    // Draw rounded box
    tft.fillRoundRect(x, y, btnW, btnH, 5, fillColor);
    if (readTwice.count(key)) {
      // draw white outline if band will be read twice
      tft.drawRoundRect(x, y, btnW, btnH, 5, ST7796S_WHITE);
    }

    // Draw text
    int tw = key.length() * 6;
    int tx = x + (btnW - tw) / 2;
    int ty = y + (btnH / 2) - 4;
    tft.setCursor(tx, ty);
    tft.print(key);
  }
}

//usage

std::vector<String> bands = {
  "LT", "FT", "RT", "LM", "FM", "RM", "LB", "FB", "RB"
};

std::map<String, String> bandState;   // pending / reading / done / warning
std::set<String> readTwice = { "FM", "RM" };  // example: these will be read twice

void exampleDraw() {
  tft.fillRect(0, 220, 480, 40, ST7796S_BLACK); // clear bottom area
  drawColorReadProgress(tft, 230, bands, bandState, readTwice);
}

bandState["FT"] = "reading"; exampleDraw();
bandState["FT"] = "done"; exampleDraw();
bandState["FT"] = "warning"; exampleDraw();

/********************************************************************
 * BAND FOR SOLVE PROGRESS
 ********************************************************************/
void drawProgressBar(Adafruit_ST7796S_kbv &tft,
                     int y_offset,
                     const std::vector<String> &items,
                     int currentIndex,
                     const std::vector<uint8_t> &states)
{
  // --- Layout constants ---
  const int btnW = 42;
  const int btnH = 26;
  const int spacing = 6;
  const int maxVisible = 10;   // fits ~10 buttons on 480px TFT
  const int radius = 5;
  int total = items.size();

  if (total == 0) return;

  // --- Determine scrolling window ---
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

  // --- Clear the band area ---
  tft.fillRect(0, y, tft.width(), btnH + 20, ST7796S_BLACK);

  // --- Draw counter (centered above bar) ---
  String counter = String(currentIndex + 1) + " / " + String(total);
  tft.setTextSize(1);
  tft.setTextColor(ST7796S_LIGHTGREY);
  int cw = counter.length() * 6;
  tft.setCursor((tft.width() - cw) / 2, y - 12);
  tft.print(counter);

  // --- “…” markers if scrolling ---
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

  // --- Draw each button ---
  for (int i = 0; i < visibleCount; i++) {
    int idx = startIndex + i;
    String label = items[idx];
    int x = startX + i * (btnW + spacing);

    // --- Determine color based on state ---
    uint16_t color = ST7796S_LIGHTGREY; // pending
    if (states.size() > idx) {
      switch (states[idx]) {
        case 1: color = ST7796S_BLUE; break;    // active
        case 2: color = ST7796S_GREEN; break;   // done ok
        case 3: color = ST7796S_YELLOW; break;  // warning / double-read
        default: break;
      }
    }

    tft.fillRoundRect(x, y, btnW, btnH, radius, color);

    // --- Highlight current move ---
    if (idx == currentIndex)
      tft.drawRoundRect(x - 1, y - 1, btnW + 2, btnH + 2, radius + 2, ST7796S_WHITE);

    // --- Label text ---
    tft.setTextColor(ST7796S_BLACK);
    int tw = label.length() * 6;
    tft.setCursor(x + (btnW - tw) / 2, y + (btnH / 2) - 4);
    tft.print(label);
  }
}


/********************************************************************
 * EXAMPLE FOR SOLVING
 ********************************************************************/

std::vector<String> solveMoves = {
  "F", "R", "U", "R'", "U'", "F'", "L", "D", "L'", "U2",
  "B", "R2", "F2", "L'", "D'", "U", "R", "F", "L2", "B2", "U2"
};
std::vector<uint8_t> moveStates(solveMoves.size(), 0);  // 0=pending

int currentMove = 0;

void updateSolveUI() {
  drawProgressBar(tft, 260, solveMoves, currentMove, moveStates);
}

// Call this after each move:
void advanceSolveProgress(bool ok) {
  moveStates[currentMove] = ok ? 2 : 3;
  currentMove++;
  if (currentMove < (int)moveStates.size())
    moveStates[currentMove] = 1;  // mark as active
  updateSolveUI();
}


/********************************************************************
 * EXAMPLE FOR COLOR READ
 ********************************************************************/

std::vector<String> readBands = {
  "L-Top", "F-Top", "R-Top", "L-Mid", "F-Mid", "R-Mid",
  "L-Bot", "F-Bot", "R-Bot"
};
std::vector<uint8_t> readStates(readBands.size(), 0);

int currentBand = 0;

void updateReadUI() {
  drawProgressBar(tft, 240, readBands, currentBand, readStates);
}

void markBandDone(bool ok, bool warning = false) {
  readStates[currentBand] = warning ? 3 : (ok ? 2 : 3);
  currentBand++;
  if (currentBand < (int)readStates.size())
    readStates[currentBand] = 1;
  updateReadUI();
}

// integration
markBandDone(true);    // during read
advanceSolveProgress(true);  // during solve






