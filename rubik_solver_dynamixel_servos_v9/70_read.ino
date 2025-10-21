/********************************************************************
 * RUN READ CUBE — full visual version
 * Reads all faces via multiple orientations with live TFT progress
 ********************************************************************/

// ----- CONFIG -----
const unsigned long READ_PAUSE_MS = 400;   // pause between bands
bool readingActive = false;

// ----- Global state -----
std::vector<String> readBands = {
  "L-Top", "F-Top", "R-Top",
  "L-Mid", "F-Mid", "R-Mid",
  "L-Bot", "F-Bot", "R-Bot"
};
std::vector<uint8_t> readStates(readBands.size(), 0); // 0=pending, 1=active, 2=done, 3=warning
int currentBand = 0;


// ====================== UI HELPERS ======================
void drawReadStats(Adafruit_ST7796S_kbv &tft, int y, int current, int total, unsigned long elapsed_ms) {
  tft.setTextSize(2);
  tft.setTextColor(ST7796S_WHITE, ST7796S_BLACK);
  tft.fillRect(0, y, tft.width(), 24, ST7796S_BLACK);

  float pct = (total > 0) ? (100.0f * (float)(current + 1) / total) : 0;
  float elapsed_s = elapsed_ms / 1000.0f;

  char buf[64];
  snprintf(buf, sizeof(buf), "Band %2d/%d  (%.0f%%)  %.1fs",
           current + 1, total, pct, elapsed_s);

  int textW = strlen(buf) * 12;
  int x = (tft.width() - textW) / 2;
  tft.setCursor(x, y + 4);
  tft.print(buf);
}


void drawReadProgressBar(Adafruit_ST7796S_kbv &tft,
                         int y_offset,
                         const std::vector<String> &items,
                         int currentIndex,
                         const std::vector<uint8_t> &states)
{
  const int btnW = 42, btnH = 26, spacing = 6, maxVisible = 10, radius = 5;
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

  tft.fillRect(0, y, tft.width(), btnH + 20, ST7796S_BLACK);

  // Counter text
  String counter = String(currentIndex + 1) + " / " + String(total);
  tft.setTextSize(1);
  tft.setTextColor(ST7796S_LIGHTGREY);
  int cw = counter.length() * 6;
  tft.setCursor((tft.width() - cw) / 2, y - 12);
  tft.print(counter);

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
        case 1: color = ST7796S_BLUE; break;
        case 2: color = ST7796S_GREEN; break;
        case 3: color = ST7796S_YELLOW; break;
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


// ====================== CORE READING LOGIC ======================

// Stub hardware helpers
void moveSensorToLevel(int level) {
  Serial.printf("Moving sensor to level %d\n", level);
  delay(200);
}
void rotateBaseToAngle(int angle) {
  Serial.printf("Rotating base to %d°\n", angle);
  delay(80);
}

/*
TODO use here from the sensor.ini either
//
char color = readColorSensor();           // single fast read
// or
char color = readColorSmoothed(5, 40);    // stable average
*/

char readColorSensorEx() {
  return readColorSmoothed(5, 40);
}

char majorityColor(const std::vector<char> &reads) {
  std::map<char, int> count;
  for (char c : reads)
    if (c != 'X') count[c]++;
  int best = 0; char winner = 'X';
  for (auto &kv : count)
    if (kv.second > best) { best = kv.second; winner = kv.first; }

  int validCount = 0;
  for (char c : reads) if (c != 'X') validCount++;
  if (validCount >= 6 && best >= 4) return winner;
  return 'X';
}


// ======= Read one 9-cell band =======
void readColorRow(int level, const String &lfu) {
  moveSensorToLevel(level);
  String ori = expandOrientation(lfu);
  Serial.printf("Reading row level %d, orientation %s\n", level, ori.c_str());

  struct CellPos { const char *name; int angle; };
  const CellPos cells[9] = {
    { "lub", -90 }, { "lu", -60 }, { "luf", -30 },
    { "ful", -10 }, { "fu", 0 }, { "fur", 10 },
    { "ruf", 30 },  { "ru", 60 }, { "rub", 90 }
  };

  for (int i = 0; i < 9; i++) {
    String cell = cells[i].name;
    String mapped = remapFace(cell, ori);
    int angle = cells[i].angle;

    std::vector<char> reads;
    for (int shift : {0, -3, 3}) {
      for (int rep = 0; rep < 3; rep++) {
        rotateBaseToAngle(angle + shift);
        delay(30);
        reads.push_back(readColorSensorEx());
      }
    }

    char voted = majorityColor(reads);
    char existing = cubeColors.getColor(mapped.c_str());
    if (existing == 'X') {
      cubeColors.setColor(mapped.c_str(), voted);
    } else if (existing != voted && voted != 'X') {
      cubeColors.setColor(mapped.c_str(), 'E');
    }

    Serial.printf("[%s→%s] = %c\n", cell.c_str(), mapped.c_str(), voted);

    // Live display
    cubeColors.drawToTFT(String(mapped[0]) + "4");
  }
}


// ====================== MAIN READ ======================
void runReadCube() {
  readingActive = true;
  Serial.println("\n=== Starting cube color read sequence ===");

  cubeColors.resetColors('X');
  cubeOri.reset();
  std::fill(readStates.begin(), readStates.end(), 0);
  if (!readBands.empty()) readStates[0] = 1;
  currentBand = 0;

  unsigned long startTime = millis();
  cubeColors.drawToTFT(tft);
  drawReadProgressBar(tft, 260, readBands, currentBand, readStates);
  drawReadStats(tft, 230, currentBand, readBands.size(), 0);

  // Sequence of orientations to cover all faces
  const String orientations[] = {"lfu", "rfd", "dbr", "ubl"};
  const int levels[] = {2, 3};

  for (int o = 0; o < 4 && readingActive; o++) {
    cubeOri.set(orientations[o]);
    for (int l = 0; l < 2 && readingActive; l++) {
      readColorRow(levels[l], orientations[o]);
      cubeColors.drawToTFT(tft);
      markBandDone(true);
      unsigned long now = millis();
      drawReadProgressBar(tft, 260, readBands, currentBand, readStates);
      drawReadStats(tft, 230, currentBand, readBands.size(), now - startTime);

      unsigned long waitStart = millis();
      while (millis() - waitStart < READ_PAUSE_MS) {
        if (digitalRead(ENC_BTN) == LOW) {
          Serial.println("❌ Read cancelled by user");
          readingActive = false;
          break;
        }
        delay(10);
      }
    }
  }

  Serial.println("✅ Cube read complete.");
  cubeColors.drawToTFT(tft);
  drawReadProgressBar(tft, 260, readBands, currentBand, readStates);
  unsigned long total = millis() - startTime;
  drawReadStats(tft, 230, currentBand, readBands.size(), total);
}
