/********************************************************************
 * READ COLORS AND STORE COLORS
 ********************************************************************/

// ====================== RubikCubeColors ======================
class RubikCubeColors {
public:
  // Internal storage: 6 faces (U,R,F,D,L,B), each 9 stickers (row-major 0..8)
  char face[6][9];

  // Face->Color (WRGYOB) mapping defined by centers (defaults W,R,G,Y,O,B)
  char faceToColor[6];

  RubikCubeColors() { resetColors('X'); setDefaultCenters(); }

  // --------- Reset / Centers ----------
  void resetColors(char fill = 'X') {
    for (int f = 0; f < 6; ++f)
      for (int i = 0; i < 9; ++i)
        face[f][i] = fill;
  }
  void setDefaultCenters() {
    // Default: U=W, R=R, F=G, D=Y, L=O, B=B
    faceToColor[U_] = 'W';
    faceToColor[R_] = 'R';
    faceToColor[F_] = 'G';
    faceToColor[D_] = 'Y';
    faceToColor[L_] = 'O';
    faceToColor[B_] = 'B';
    // Also set centers to their face letters internally (URFDLB)
    face[U_][4] = 'U'; face[R_][4] = 'R'; face[F_][4] = 'F';
    face[D_][4] = 'D'; face[L_][4] = 'L'; face[B_][4] = 'B';
  }
  // Optionally set/override a single center mapping (e.g., U face center color detected as 'W')
  void setCenterColorURFDLB(char faceLetterURFDLB, char colorWRGYOB) {
    int f = faceIndex(faceLetterURFDLB);
    if (f >= 0 && isWRGYOB(colorWRGYOB)) faceToColor[f] = toupper(colorWRGYOB);
  }

  // --------- Setters: URFDLB vs WRGYOB ----------
  // cell: e.g. "ful", "ul", "f", "r", order-insensitive; value must be URFDLB (U,R,F,D,L,B)
  bool setColorURFDLB(const String& cell, char faceLetterURFDLB) {
    int f, idx;
    if (!parseCell(cell, f, idx)) return false;
    if (!isURFDLB(faceLetterURFDLB)) faceLetterURFDLB = 'X';
    face[f][idx] = toupper(faceLetterURFDLB);
    return true;
  }

  // cell: same as above; value must be WRGYOB (W,R,G,Y,O,B)
  // Will convert WRGYOB -> URFDLB via inverse of centers before storing
  bool setColorWRGYOB(const String& cell, char colorWRGYOB) {
    int f, idx;
    if (!parseCell(cell, f, idx)) return false;
    char c = toupper(colorWRGYOB);
    if (!isWRGYOB(c)) c = 'X';
    char faceLetter = colorToFace(c);  // convert via center mapping
    face[f][idx] = faceLetter ? faceLetter : 'X';
    return true;
  }

  // --------- Getters ----------
  // Returns stored URFDLB letter for that sticker (or 'X')
  char getColorURFDLB(const String& cell) const {
    int f, idx;
    if (!parseCell(cell, f, idx)) return 'X';
    return face[f][idx];
  }

  // Returns color (WRGYOB) by mapping the stored face letter through centers.
  // If stored value is 'X' (unknown), returns 'X'.
  char getColorWRGYOB(const String& cell) const {
    int f, idx;
    if (!parseCell(cell, f, idx)) return 'X';
    char urfdlb = face[f][idx];
    if (!isURFDLB(urfdlb)) return 'X';
    int ff = faceIndex(urfdlb);
    return ff >= 0 ? faceToColor[ff] : 'X';
  }

  // Flattened strings (54 chars), face order: U R F D L B (URFDLB)
  String fullStringURFDLB() const {
    String s; s.reserve(54);
    appendFaceString(s, U_); appendFaceString(s, R_); appendFaceString(s, F_);
    appendFaceString(s, D_); appendFaceString(s, L_); appendFaceString(s, B_);
    return s;
  }
  // Same order, but converted to WRGYOB using centers
  String fullStringWRGYOB() const {
    String s; s.reserve(54);
    appendFaceStringColor(s, U_); appendFaceStringColor(s, R_);
    appendFaceStringColor(s, F_); appendFaceStringColor(s, D_);
    appendFaceStringColor(s, L_); appendFaceStringColor(s, B_);
    return s;
  }

  // --------- Drawing on Adafruit_ST7796S_kbv -----------
  // Draw net as:
  //         [U]
  // [L] [F] [R] [B]
  //         [D]
  // Centered horizontally; thin black grid lines between tiles.
  void drawTFT(Adafruit_ST7796S_kbv& tft, int yTop = 64, int tile = 24, int gap = 2) const {
    // Compute full net width: L F R B faces in a row, with gaps between faces
    int facesRow = 4;
    int faceSize = 3 * tile + 4 * gap;     // 3 tiles + inner lines (gap used as line)
    int netWidth = facesRow * faceSize + (facesRow - 1) * gap;
    int xLeft = (tft.width() - netWidth) / 2;
    if (xLeft < 4) xLeft = 4;              // small margin

    // Face positions
    // U centered above F
    int xU = xLeft + faceSize + gap;  // second face in row position
    int yU = yTop;

    int yRow = yTop + faceSize + gap; // row for L F R B
    int xL = xLeft + 0 * (faceSize + gap);
    int xF = xLeft + 1 * (faceSize + gap);
    int xR = xLeft + 2 * (faceSize + gap);
    int xB = xLeft + 3 * (faceSize + gap);

    int xD = xU;
    int yD = yRow + faceSize + gap;

    // Draw all faces
    drawFace(tft, U_, xU, yU, tile, gap);
    drawFace(tft, L_, xL, yRow, tile, gap);
    drawFace(tft, F_, xF, yRow, tile, gap);
    drawFace(tft, R_, xR, yRow, tile, gap);
    drawFace(tft, B_, xB, yRow, tile, gap);
    drawFace(tft, D_, xD, yD, tile, gap);
  }

private:
  // ------- Face enum / helpers -------
  enum { U_ = 0, R_ = 1, F_ = 2, D_ = 3, L_ = 4, B_ = 5 };

  static bool isURFDLB(char c) {
    c = toupper(c);
    return (c=='U'||c=='R'||c=='F'||c=='D'||c=='L'||c=='B');
  }
  static bool isWRGYOB(char c) {
    c = toupper(c);
    return (c=='W'||c=='R'||c=='G'||c=='Y'||c=='O'||c=='B');
  }
  static int faceIndex(char c) {
    c = toupper(c);
    switch (c) {
      case 'U': return U_;
      case 'R': return R_;
      case 'F': return F_;
      case 'D': return D_;
      case 'L': return L_;
      case 'B': return B_;
      default:  return -1;
    }
  }

  // Convert actual color letter (WRGYOB) to face letter (URFDLB) via centers
  char colorToFace(char colorWRGYOB) const {
    char c = toupper(colorWRGYOB);
    for (int f = 0; f < 6; ++f) {
      if (faceToColor[f] == c) {
        return faceLetterFor(f);
      }
    }
    return 0;
  }
  static char faceLetterFor(int f) {
    switch (f) {
      case U_: return 'U'; case R_: return 'R'; case F_: return 'F';
      case D_: return 'D'; case L_: return 'L'; case B_: return 'B';
      default: return 0;
    }
  }

  void appendFaceString(String& s, int f) const {
    for (int i = 0; i < 9; ++i) s += face[f][i];
  }
  void appendFaceStringColor(String& s, int f) const {
    for (int i = 0; i < 9; ++i) {
      char urfdlb = face[f][i];
      if (!isURFDLB(urfdlb)) { s += 'X'; continue; }
      int ff = faceIndex(urfdlb);
      s += (ff >= 0 ? faceToColor[ff] : 'X');
    }
  }

  // ------- Parsing "cell" like "ful","ul","f" -> (face,index) -------
  // Rules:
  //  - 1 letter (center): index 4
  //  - 2 letters (edge): primary face is the first face letter encountered in the string;
  //    index determined by neighbor letter.
  //  - 3 letters (corner): same; index determined by the other two.
  bool parseCell(String cell, int& outFace, int& outIndex) const {
    cell.toLowerCase();
    // collect unique face letters present
    bool has[6] = {false,false,false,false,false,false};
    String filtered;
    for (size_t i = 0; i < cell.length(); ++i) {
      char ch = cell[i];
      int f = -1;
      switch (ch) {
        case 'u': f = U_; break; case 'r': f = R_; break; case 'f': f = F_; break;
        case 'd': f = D_; break; case 'l': f = L_; break; case 'b': f = B_; break;
      }
      if (f >= 0 && !has[f]) { has[f] = true; filtered += ch; }
    }
    int n = filtered.length();
    if (n < 1 || n > 3) return false;

    // choose primary face = the first letter in user's order
    int pf = faceIndex(toupper(filtered[0]));
    if (pf < 0) return false;

    outFace = pf;

    if (n == 1) { outIndex = 4; return true; }    // center

    if (n == 2) {
      // edge on primary face with neighbor letter
      int nf = faceIndex(toupper(filtered[1]));
      if (nf < 0) return false;
      int idx = edgeIndexOnFace(pf, nf);
      if (idx < 0) return false;
      outIndex = idx;
      return true;
    }

    // n == 3 => corner on primary face with two neighbors
    int nf1 = faceIndex(toupper(filtered[1]));
    int nf2 = faceIndex(toupper(filtered[2]));
    if (nf1 < 0 || nf2 < 0) return false;
    int idx = cornerIndexOnFace(pf, nf1, nf2);
    if (idx < 0) return false;
    outIndex = idx;
    return true;
  }

  // ---- Per-face edge index mapping (primary face pf, neighbor nf) ----
  // Indices are row-major (0..8)
  static int edgeIndexOnFace(int pf, int nf) {
    switch (pf) {
      case F_: // neighbors: U(1), D(7), L(3), R(5)
        if (nf==U_) return 1; if (nf==D_) return 7; if (nf==L_) return 3; if (nf==R_) return 5; break;
      case B_: // mirrored left-right: U(1), D(7), L(5), R(3)
        if (nf==U_) return 1; if (nf==D_) return 7; if (nf==L_) return 5; if (nf==R_) return 3; break;
      case U_: // neighbors around: B(top=1), R(5), F(7), L(3)
        if (nf==F_) return 7; if (nf==B_) return 1; if (nf==L_) return 3; if (nf==R_) return 5; break;
      case D_: // neighbors around: F(1), R(5), B(7), L(3)
        if (nf==F_) return 1; if (nf==B_) return 7; if (nf==L_) return 3; if (nf==R_) return 5; break;
      case R_: // neighbors: U(1), D(7), F(3), B(5)
        if (nf==U_) return 1; if (nf==D_) return 7; if (nf==F_) return 3; if (nf==B_) return 5; break;
      case L_: // neighbors: U(1), D(7), F(5), B(3)
        if (nf==U_) return 1; if (nf==D_) return 7; if (nf==F_) return 5; if (nf==B_) return 3; break;
    }
    return -1;
  }

  // ---- Per-face corner index mapping (primary face pf, neighbors n1,n2 in any order) ----
  static int cornerIndexOnFace(int pf, int n1, int n2) {
    auto has2 = [&](int a, int b){ return ( (n1==a && n2==b) || (n1==b && n2==a) ); };
    switch (pf) {
      case F_:
        if (has2(U_,L_)) return 0;
        if (has2(U_,R_)) return 2;
        if (has2(D_,L_)) return 6;
        if (has2(D_,R_)) return 8;
        break;
      case B_:
        if (has2(U_,L_)) return 2;
        if (has2(U_,R_)) return 0;
        if (has2(D_,L_)) return 8;
        if (has2(D_,R_)) return 6;
        break;
      case U_:
        if (has2(F_,L_)) return 6;
        if (has2(F_,R_)) return 8;
        if (has2(B_,L_)) return 0;
        if (has2(B_,R_)) return 2;
        break;
      case D_:
        if (has2(F_,L_)) return 0;
        if (has2(F_,R_)) return 2;
        if (has2(B_,L_)) return 6;
        if (has2(B_,R_)) return 8;
        break;
      case R_:
        if (has2(U_,F_)) return 0;
        if (has2(U_,B_)) return 2;
        if (has2(D_,F_)) return 6;
        if (has2(D_,B_)) return 8;
        break;
      case L_:
        if (has2(U_,F_)) return 2;
        if (has2(U_,B_)) return 0;
        if (has2(D_,F_)) return 8;
        if (has2(D_,B_)) return 6;
        break;
    }
    return -1;
  }

  // --------- Drawing helpers ----------
  static uint16_t colorForURFDLB(char faceLetter) {
    switch (toupper(faceLetter)) {
      case 'U': return ST7796S_WHITE;
      case 'R': return ST7796S_RED;
      case 'F': return ST7796S_GREEN;
      case 'D': return ST7796S_YELLOW;
      case 'L': return ST7796S_ORANGE;
      case 'B': return ST7796S_BLUE;
      default:  return ST7796S_DARKGREY; // unknown 'X'
    }
  }

  void drawFace(Adafruit_ST7796S_kbv& tft, int f, int x0, int y0, int tile, int gap) const {
    // face background (optional): draw a black frame
    tft.drawRect(x0-1, y0-1, 3*tile + 4*gap + 2, 3*tile + 4*gap + 2, ST7796S_BLACK);

    // 3x3 tiles with thin black grid lines (gap acts as line)
    int idx = 0;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        int x = x0 + c*(tile + gap) + gap;
        int y = y0 + r*(tile + gap) + gap;
        uint16_t col = colorForURFDLB(face[f][idx]);
        tft.fillRect(x, y, tile, tile, col);
        tft.drawRect(x, y, tile, tile, ST7796S_BLACK); // thin border
        ++idx;
      }
    }
  }
};
// =================== end RubikCubeColors ===================

// ---------- Example usage ----------
// RubikCubeColors cube;
// cube.resetColors('X');
// cube.setCenterColorURFDLB('U','W'); // etc if your centers differ
// cube.setColorWRGYOB("ful", 'G');    // from camera: green on the F-U-L corner
// cube.setColorURFDLB("fu", 'F');     // explicitly set face letter
// cube.drawTFT(tft, /*yTop=*/64, /*tile=*/24, /*gap=*/2);




void drawCubeLayout(TFT_eSPI &tft, RubikCube &cube) {
  int cell = 18;   // each small square size
  int pad  = 2;    // padding between squares
  int faceSize = 3 * (cell + pad);

  // Compute total width for [L F R B]
  int totalWidth = 4 * faceSize + 3 * pad;
  int startX = (tft.width() - totalWidth) / 2;
  int startY = 80; // leave space for title/subtitle

  auto drawFace = [&](int x0, int y0, char face) {
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        char col = cube.getFaceColor(face, r, c);
        uint16_t fill = colorForChar(col);
        int x = x0 + c * (cell + pad);
        int y = y0 + r * (cell + pad);
        tft.fillRect(x, y, cell, cell, fill);
        tft.drawRect(x, y, cell, cell, ST7796S_BLACK);
      }
    }
  };

  // Center line (L F R B)
  drawFace(startX,               startY + faceSize, 'L');
  drawFace(startX + faceSize+pad, startY + faceSize, 'F');
  drawFace(startX + 2*(faceSize+pad), startY + faceSize, 'R');
  drawFace(startX + 3*(faceSize+pad), startY + faceSize, 'B');

  // Top (U) and bottom (D)
  int centerX = startX + faceSize + pad;  // aligned with F
  drawFace(centerX, startY, 'U');
  drawFace(centerX, startY + 2*(faceSize+pad), 'D');
}

// ==================== RUBIK CUBE COLOR READER ====================
// Reads 18 edge+center cells per layer using an Adafruit color sensor
// and populates a RubikColors instance.
//
// Assumes base rotates between -90° and +90°
// and the color sensor is 25mm in front of the cube centerline.

// ---- STUBS (replace with actual hardware control later) ----

// Rotates base servo or motor to a given angle (in degrees)
void rotateBaseToAngle(float angle) {
  Serial.printf("Rotating base to %.1f°...\n", angle);
  delay(300); // simulate movement delay
}

// Reads one color sample from sensor, returns 'W','R','G','Y','O','B' or 'X' if invalid
char readColorSensor() {
  // TODO: replace this stub with actual color reading & classification
  char possible[7] = {'W','R','G','Y','O','B','X'};
  return possible[random(0,7)];
}

// Helper: check if char is valid color
bool isValidColor(char c) {
  c = toupper(c);
  return (c=='W'||c=='R'||c=='G'||c=='Y'||c=='O'||c=='B');
}

// Helper: compute mode color with validation rules
char computeMajorityColor(const std::vector<char>& readings) {
  // Count valid colors
  std::map<char,int> count;
  int validCount = 0;
  for (char c : readings) {
    if (isValidColor(c)) {
      count[c]++;
      validCount++;
    }
  }

  if (validCount < 6) return 'X'; // not enough valid readings

  // Find the most frequent color
  char best = 'X';
  int bestCount = 0;
  for (auto &kv : count) {
    if (kv.second > bestCount) {
      bestCount = kv.second;
      best = kv.first;
    }
  }

  if (bestCount >= 4) return best;
  return 'X';
}

// ==================== READ ONE CELL ====================
// Reads a single cell at given baseAngle
char readCellWithRetries(float baseAngleDeg) {
  std::vector<char> readings;

  // First 3 at base angle
  rotateBaseToAngle(baseAngleDeg);
  for (int i=0; i<3; ++i) {
    readings.push_back(readColorSensor());
  }

  // If 3 match and valid → done
  if (isValidColor(readings[0]) &&
      readings[0]==readings[1] &&
      readings[1]==readings[2]) {
    return readings[0];
  }

  // Otherwise take ±3° and read 3 each
  std::vector<char> plus3, minus3;

  rotateBaseToAngle(baseAngleDeg + 3);
  for (int i=0; i<3; ++i) plus3.push_back(readColorSensor());
  rotateBaseToAngle(baseAngleDeg - 3);
  for (int i=0; i<3; ++i) minus3.push_back(readColorSensor());

  // check if any triplet is stable
  if (isValidColor(plus3[0]) && plus3[0]==plus3[1] && plus3[1]==plus3[2]) return plus3[0];
  if (isValidColor(minus3[0]) && minus3[0]==minus3[1] && minus3[1]==minus3[2]) return minus3[0];

  // Otherwise combine all 9
  readings.insert(readings.end(), plus3.begin(), plus3.end());
  readings.insert(readings.end(), minus3.begin(), minus3.end());

  return computeMajorityColor(readings);
}

// ==================== READ FULL ROW OF 18 CELLS ====================
// Reads 3x3 visible cells per layer (left, center, right) * 6 cube faces
// Only reads the 18 edge/center cells in current layer orientation

void readLayerColors(RubikColors &cube) {
  // 18 cell positions around cube (same height)
  // Order: left → center → right for F, L, R, B faces (example mapping)
  struct CellPos { String name; float angle; };

  CellPos cells[] = {
    {"fl", -60}, {"fc", 0}, {"fr", 60},    // front row
    {"ll", -90}, {"lc", -60}, {"lr", -30}, // left row
    {"rl", 30},  {"rc", 60}, {"rr", 90}   // right row
  };

  int numCells = sizeof(cells)/sizeof(cells[0]);

  for (int i=0; i<numCells; ++i) {
    char color = readCellWithRetries(cells[i].angle);
    cube.setColor(cells[i].name.c_str(), &color);
    Serial.printf("Cell %-4s → %c\n", cells[i].name.c_str(), color);
  }
}


TODO use below and -90,0,90 for the actual reads
  setParamValue("base_minus60", angle_to_ticks(base_0, base_90, -60));
  setParamValue("base_minus30", angle_to_ticks(base_0, base_90, -60));
  setParamValue("base_30", angle_to_ticks(base_0, base_90, 30));
  setParamValue("base_60", angle_to_ticks(base_0, base_90, 60));

