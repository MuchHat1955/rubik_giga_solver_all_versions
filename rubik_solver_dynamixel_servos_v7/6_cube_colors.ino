/********************************************************************
 * DRAW CUBE COLORS AND PROGRESS BAR DURING READING AND SOLVING
 ********************************************************************/

 /////////////////////////////////////////////////////////////////

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

 /////////////////////////////////////////////////////////////////

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







