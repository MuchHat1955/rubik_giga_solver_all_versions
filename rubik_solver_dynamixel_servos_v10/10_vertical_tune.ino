/*******************************************************
 * Vertical Tuning UI + Vertical Motion Engine
 * - Two columns: Height (mm) and Offset buttons
 * - Highlight / select / edit (+/-0.2 mm)
 * - Stores per-height ticks: arm1_midNNN / arm2_midNNN
 * - Stores per-height offset mm*10: mid_offNNN
 * - moveArmsVertically(mm): uses saved points if present;
 *   interpolates between; extrapolates beyond; falls
 *   back to 2-link IK (L1=L2=80 mm) if needed.
 *******************************************************/

#include <Arduino.h>
#include <math.h>
#include <vector>
#include <map>

// ---------- Expect these globals to already exist ----------
extern Adafruit_ST7796S_kbv tft;
extern Encoder rotary;
extern Preferences prefs;
extern Dynamixel2Arduino dxl;

// Your color theme (override if you already defined)
#ifndef ST7796S_LIGHTGREY
#define ST7796S_LIGHTGREY 0xC618
#endif

#ifndef TITLE_COLOR
#define TITLE_COLOR       ST7796S_WHITE
#endif
#ifndef SUBTITLE_COLOR
#define SUBTITLE_COLOR    ST7796S_YELLOW
#endif
#ifndef ROW_TEXT_COLOR
#define ROW_TEXT_COLOR    ST7796S_WHITE
#endif
#ifndef HIGHLIGHT_COLOR
#define HIGHLIGHT_COLOR   ST7796S_CYAN
#endif
#ifndef SELECT_COLOR
#define SELECT_COLOR      ST7796S_CYAN
#endif
#ifndef BACKGROUND_COLOR
#define BACKGROUND_COLOR  ST7796S_BLACK
#endif

#ifndef TITLE_FONT_SIZE
#define TITLE_FONT_SIZE 3
#endif
#ifndef SUBTITLE_FONT_SIZE
#define SUBTITLE_FONT_SIZE 2
#endif
#ifndef ROW_FONT_SIZE
#define ROW_FONT_SIZE 2
#endif

#ifndef ENC_BTN
#define ENC_BTN 21
#endif

// ---------- External helpers expected ----------
int  getParamValue(const String &k);
void setParamValue(const String &k, int v);
bool runGroupPoseServos(const std::vector<String> &servos, const std::vector<int> &targets);

// ---------- Geometry & Calibration ----------
static const float IK_L1 = 80.0f;  // mm
static const float IK_L2 = 80.0f;  // mm

// Angle ↔ ticks (linear) by two-point calibration
static inline int angleToTicks(const String &servo, float deg) {
  String k0 = servo + "_0deg";
  String k90 = servo + "_90deg";
  int t0  = getParamValue(k0);   // default can be 1024
  int t90 = getParamValue(k90);  // default can be 1536, etc.

  if (t0 == 0 && t90 == 0) {
    // Fallback: neutral mapping
    // XL430 typical center ~2048; 0..4095 for 360°
    // But user constrained 0..3072 => ~0.0879 deg/tick
    // We'll approximate 0°=1024, 90°=1536 if not set
    t0  = 1024;
    t90 = 1536;
  }
  float slope = (float)(t90 - t0) / 90.0f;
  return (int)roundf(t0 + slope * deg);
}
static inline float ticksToAngle(const String &servo, int ticks) {
  String k0 = servo + "_0deg";
  String k90 = servo + "_90deg";
  int t0  = getParamValue(k0);
  int t90 = getParamValue(k90);
  if (t0 == 0 && t90 == 0) {
    t0  = 1024;
    t90 = 1536;
  }
  float slope = 90.0f / (float)(t90 - t0);
  return (ticks - t0) * slope;
}

// ---------- Per-height key helpers ----------
static inline String fmtNNN(float mm) {
  // 0.0..20.0 step 0.5 -> tenths 0..200
  int tenths = (int)roundf(mm * 10.0f);
  if (tenths < 0) tenths = 0;
  if (tenths > 200) tenths = 200;
  char buf[8];
  snprintf(buf, sizeof(buf), "%03d", tenths);
  return String(buf);
}
static inline String kArm1Ticks(float mm) { return "arm1_mid" + fmtNNN(mm); }
static inline String kArm2Ticks(float mm) { return "arm2_mid" + fmtNNN(mm); }
static inline String kOffsetMM10(float mm) { return "mid_off" + fmtNNN(mm); }

// ---------- Preferences helpers (direct) ----------
static inline bool readTicksIfPresent(float mm, int &t1, int &t2) {
  prefs.begin("rubik", true);
  String k1 = kArm1Ticks(mm);
  String k2 = kArm2Ticks(mm);
  bool ok1=false, ok2=false;
  int v1 = prefs.getInt(k1.c_str(), INT32_MIN);
  int v2 = prefs.getInt(k2.c_str(), INT32_MIN);
  prefs.end();
  if (v1 != INT32_MIN) { t1=v1; ok1=true; }
  if (v2 != INT32_MIN) { t2=v2; ok2=true; }
  return ok1 && ok2;
}
static inline void writeTicks(float mm, int t1, int t2) {
  prefs.begin("rubik", false);
  prefs.putInt(kArm1Ticks(mm).c_str(), t1);
  prefs.putInt(kArm2Ticks(mm).c_str(), t2);
  prefs.end();
}
static inline bool readOffsetIfPresent(float mm, int &mm10) {
  prefs.begin("rubik", true);
  int v = prefs.getInt(kOffsetMM10(mm).c_str(), INT32_MIN);
  prefs.end();
  if (v == INT32_MIN) return false;
  mm10 = v;
  return true;
}
static inline void writeOffset(float mm, int mm10) {
  prefs.begin("rubik", false);
  prefs.putInt(kOffsetMM10(mm).c_str(), mm10);
  prefs.end();
}

// ---------- Forward/Inverse Kinematics ----------
static inline void FK_2R(float th1_deg, float th2_deg, float &x, float &y) {
  float t1 = th1_deg * DEG_TO_RAD;
  float t2 = th2_deg * DEG_TO_RAD;
  x = IK_L1 * cosf(t1) + IK_L2 * cosf(t1 + t2);
  y = IK_L1 * sinf(t1) + IK_L2 * sinf(t1 + t2);
}

static bool IK_2R(float x, float y, float &th1_deg, float &th2_deg) {
  float R2 = x*x + y*y;
  float denom = 2.0f * IK_L1 * IK_L2;
  float c2 = (R2 - IK_L1*IK_L1 - IK_L2*IK_L2) / denom;
  if (c2 < -1.0f || c2 > 1.0f) return false;
  // elbow-up (positive s2)
  float s2 = sqrtf(max(0.0f, 1.0f - c2*c2));
  float t2 = atan2f(s2, c2);
  float k1 = IK_L1 + IK_L2 * cosf(t2);
  float k2 = IK_L2 * sinf(t2);
  float t1 = atan2f(y, x) - atan2f(k2, k1);
  th1_deg = t1 * RAD_TO_DEG;
  th2_deg = t2 * RAD_TO_DEG;
  return true;
}

// ---------- Derive vertical line (x0) from mid reference ----------
static bool deriveVerticalLine(float &x0, float &y_mid) {
  int t1_mid = getParamValue("arm1_1");
  int t2_mid = getParamValue("arm2_1");
  if (t1_mid == 0 && t2_mid == 0) {
    // not set; try a safe default
    x0 = 120.0f; y_mid = 100.0f;
    return false;
  }
  float th1 = ticksToAngle("arm1", t1_mid);
  float th2 = ticksToAngle("arm2", t2_mid);
  FK_2R(th1, th2, x0, y_mid);
  return true;
}

// ---------- Solve target ticks at height mm (with optional lateral offset) ----------
static bool solveTicksForHeight(float mm, float offset_mm, int &out_t1, int &out_t2) {
  // 1) If exact saved ticks exist, use them (ignore offset if you prefer exact reproduction)
  int t1s=0, t2s=0;
  if (readTicksIfPresent(mm, t1s, t2s)) {
    out_t1 = t1s; out_t2 = t2s;
    return true;
  }

  // 2) Try interpolate (or extrapolate) from nearest saved points
  //    Build a list of available saved samples (mm, t1, t2)
  std::vector<float> hs;
  std::vector<int>   t1v, t2v;
  for (int tenths=0; tenths<=200; tenths+=5) {
    float h = tenths / 10.0f;
    int a=0,b=0;
    if (readTicksIfPresent(h,a,b)) {
      hs.push_back(h); t1v.push_back(a); t2v.push_back(b);
    }
  }

  auto lerp = [](float a, float b, float t)->float { return a + (b - a)*t; };

  if (!hs.empty()) {
    // find bracketing
    int n = hs.size();
    int lo = -1, hi = -1;
    for (int i=0;i<n;i++){
      if (hs[i] <= mm) { if (lo==-1 || hs[i]>hs[lo]) lo=i; }
      if (hs[i] >= mm) { if (hi==-1 || hs[i]<hs[hi]) hi=i; }
    }
    if (lo>=0 && hi>=0 && lo!=hi) {
      float t = (mm - hs[lo]) / (hs[hi] - hs[lo]);
      float t1f = lerp((float)t1v[lo], (float)t1v[hi], t);
      float t2f = lerp((float)t2v[lo], (float)t2v[hi], t);
      out_t1 = (int)roundf(t1f);
      out_t2 = (int)roundf(t2f);
      return true;
    } else if (lo>=0 && hi==-1 && n>=2) {
      // extrapolate above top using last two
      int i2 = lo;
      int i1 = max(0, lo-1);
      float hspan = hs[i2]-hs[i1]; if (hspan==0) hspan=1;
      float t = (mm - hs[i2]) / hspan;
      float t1f = (float)t1v[i2] + ((float)t1v[i2] - (float)t1v[i1]) * t;
      float t2f = (float)t2v[i2] + ((float)t2v[i2] - (float)t2v[i1]) * t;
      out_t1 = (int)roundf(t1f);
      out_t2 = (int)roundf(t2f);
      return true;
    } else if (hi>=0 && lo==-1 && n>=2) {
      // extrapolate below bottom if ever needed
      int i1 = hi;
      int i2 = min(n-1, hi+1);
      float hspan = hs[i2]-hs[i1]; if (hspan==0) hspan=1;
      float t = (mm - hs[i1]) / hspan;
      float t1f = (float)t1v[i1] + ((float)t1v[i1] - (float)t1v[i2]) * t;
      float t2f = (float)t2v[i1] + ((float)t2v[i1] - (float)t2v[i2]) * t;
      out_t1 = (int)roundf(t1f);
      out_t2 = (int)roundf(t2f);
      return true;
    }
  }

  // 3) IK fallback on vertical line derived from mid
  float x0=120.0f, y_mid=100.0f;
  deriveVerticalLine(x0, y_mid); // best-effort
  float x = x0 + offset_mm;
  float y = y_mid + mm;
  float th1=0, th2=0;
  if (!IK_2R(x,y,th1,th2)) {
    // Clamp y to reachable circle
    float r = min(x, x); (void)r;
    // crude fallback: just clamp y within reach
    float Rmax = IK_L1 + IK_L2 - 1;
    float a = atan2f(y, x);
    x = Rmax * cosf(a);
    y = Rmax * sinf(a);
    if (!IK_2R(x,y,th1,th2)) return false;
  }
  out_t1 = angleToTicks("arm1", th1);
  out_t2 = angleToTicks("arm2", th2);
  return true;
}

// =====================================================
// Public: Move arms vertically to mm (uses stored offset)
// =====================================================
bool moveArmsVertically(float mm) {
  if (mm < 0.0f) mm = 0.0f;
  if (mm > 20.0f) mm = 20.0f;

  int mm10 = 0;
  float offset = 0.0f;
  if (readOffsetIfPresent(mm, mm10)) offset = mm10 / 10.0f;

  int t1=0, t2=0;
  if (!solveTicksForHeight(mm, offset, t1, t2)) {
    Serial.printf("[moveArmsVertically] IK/solve failed at %.1fmm\n", mm);
    return false;
  }

  bool ok = runGroupPoseServos({ "arm1","arm2" }, { t1, t2 });
  if (!ok) Serial.println("[moveArmsVertically] Motion stalled/failed.");
  return ok;
}

// =====================================================
// UI: Run Vertical Tuning
// =====================================================

struct VPoint {
  float height_mm;   // 0.0 .. 20.0
  float offset_mm;   // loaded from prefs (mm10/10)
  bool  hasTicks;    // whether ticks saved for this height
  bool  editable;    // all except 0.0 and 20.0 are editable
};

static std::vector<VPoint> vt_points;
static int vt_highlight = 0;     // index into vt_points
static bool vt_editMode = false; // editing offset?
static long vt_lastEnc = 0;
static unsigned long vt_btnDownAt = 0;

// text width helper
static inline int textWidthPx(const String &s, int sz) {
  return s.length() * 6 * sz;
}

// Build the 0..20.0 @ 0.5mm list, load saved offsets/ticks
static void vt_buildModel() {
  vt_points.clear();
  for (int tenths=0; tenths<=200; tenths+=5) {
    float mm = tenths / 10.0f;
    int mm10 = 0;
    bool hasOff = readOffsetIfPresent(mm, mm10);
    int t1=0,t2=0; bool ht = readTicksIfPresent(mm, t1, t2);
    VPoint p;
    p.height_mm = mm;
    p.offset_mm = hasOff ? (mm10/10.0f) : 0.0f;
    p.hasTicks  = ht;
    p.editable  = (tenths != 0) && (tenths != 200); // 0.0 and 20.0 are non-editable
    vt_points.push_back(p);
  }
  // Start highlighted at 0.0mm (bottom)
  vt_highlight = 0;
  vt_editMode = false;
}

// Draw one button (offset) and the label column
static void vt_drawScreen() {
  tft.fillScreen(BACKGROUND_COLOR);

  // Title
  String title = "Vertical tuning";
  tft.setTextSize(TITLE_FONT_SIZE);
  tft.setTextColor(TITLE_COLOR, BACKGROUND_COLOR);
  int tw = textWidthPx(title, TITLE_FONT_SIZE);
  tft.setCursor((tft.width()-tw)/2, 4);
  tft.print(title);

  // Subtitle (Back button)
  String back = "back";
  int subH=28, subY=42, subX=10, subW=tft.width()-20;
  bool subSelected = (vt_highlight == -1); // we map "Back" as -1 when jumped
  if (subSelected) {
    tft.fillRoundRect(subX, subY, subW, subH, 6, SELECT_COLOR);
    tft.setTextColor(BACKGROUND_COLOR);
  } else {
    tft.drawRoundRect(subX, subY, subW, subH, 6, HIGHLIGHT_COLOR);
    tft.setTextColor(SUBTITLE_COLOR);
  }
  tft.setTextSize(SUBTITLE_FONT_SIZE);
  int bw = textWidthPx(back, SUBTITLE_FONT_SIZE);
  tft.setCursor((tft.width()-bw)/2, subY + (subH-8));
  tft.print(back);

  // List panel
  const int listY = subY + subH + 8;
  const int btnH = 30;
  const int rowGap = 6;
  const int visibleRows = 8; // fits on 480x320 under title/subtitle
  const int col1W = 120;     // "height" column
  const int col2W = 150;     // "offset" button col
  const int panelX = (tft.width() - (col1W+col2W+20)) / 2;
  const int col1X = panelX;
  const int col2X = panelX + col1W + 20;

  // Determine window so that 0.0 at bottom and 20.0 at top are always visible.
  // We'll pin index0 and indexLast and scroll the middle band.
  int n = vt_points.size();           // 41
  int idxBottom = 0;                  // 0.0 mm
  int idxTop    = n-1;                // 20.0 mm

  // The middle window excludes top/bottom
  int middleCount = visibleRows - 2;  // keep two slots reserved (top & bottom)
  if (middleCount < 0) middleCount = 0;

  // Compute a middle-window start around highlight (clamped to 1..n-2)
  int h = vt_highlight;
  if (h <= 0) h = 1;
  if (h >= n-1) h = n-2;

  int startMid = h - middleCount/2;
  if (startMid < 1) startMid = 1;
  if (startMid + middleCount > n-1) startMid = n-1 - middleCount;

  // Draw TOP (20.0mm) pinned
  int y = listY;
  {
    const VPoint &p = vt_points[idxTop];
    // label
    tft.setTextSize(ROW_FONT_SIZE);
    tft.setTextColor(ROW_TEXT_COLOR, BACKGROUND_COLOR);
    char lab[16]; snprintf(lab, sizeof(lab), "%.1f mm", p.height_mm);
    tft.setCursor(col1X, y+8);
    tft.print(lab);

    // button (non-editable, gray)
    uint16_t fill = ST7796S_DARKGREY;
    tft.fillRoundRect(col2X, y, col2W, btnH, 6, fill);
    tft.setTextColor(ST7796S_BLACK);
    char off[16]; snprintf(off, sizeof(off), "%+.1f mm", p.offset_mm);
    int lpx = col2X + (col2W - (int)strlen(off)*6)/2;
    tft.setCursor(lpx, y + (btnH/2)-4);
    tft.print(off);
  }

  // Draw MIDDLE window
  y += btnH + rowGap;
  int drawn = 0;
  for (int i = startMid; i < startMid + middleCount; i++) {
    const VPoint &p = vt_points[i];
    bool highlighted = (i == vt_highlight);
    bool selected    = highlighted && vt_editMode;

    // label
    tft.setTextSize(ROW_FONT_SIZE);
    tft.setTextColor(ROW_TEXT_COLOR, BACKGROUND_COLOR);
    char lab[16]; snprintf(lab, sizeof(lab), "%.1f mm", p.height_mm);
    tft.setCursor(col1X, y+8);
    tft.print(lab);

    // button color:
    // - hasTicks -> cyan-ish (configured as HIGHLIGHT_COLOR fill look),
    // - no ticks -> light grey
    // - selected (editing) -> SELECT_COLOR
    uint16_t fill = p.hasTicks ? ST7796S_LIGHTGREY : ST7796S_DARKGREY;
    if (selected) fill = SELECT_COLOR;

    tft.fillRoundRect(col2X, y, col2W, btnH, 6, fill);
    if (highlighted && !selected) {
      tft.drawRoundRect(col2X-1, y-1, col2W+2, btnH+2, 6, HIGHLIGHT_COLOR);
    }

    tft.setTextColor(selected ? BACKGROUND_COLOR : ST7796S_BLACK);
    char off[16]; snprintf(off, sizeof(off), "%+.1f mm", p.offset_mm);
    int lpx = col2X + (col2W - (int)strlen(off)*6)/2;
    tft.setCursor(lpx, y + (btnH/2)-4);
    tft.print(off);

    y += btnH + rowGap;
    drawn++;
  }

  // Draw BOTTOM (0.0mm) pinned
  if (y + btnH <= (int)tft.height()) {
    const VPoint &p = vt_points[idxBottom];
    // label
    tft.setTextSize(ROW_FONT_SIZE);
    tft.setTextColor(ROW_TEXT_COLOR, BACKGROUND_COLOR);
    char lab[16]; snprintf(lab, sizeof(lab), "%.1f mm", p.height_mm);
    tft.setCursor(col1X, y+8);
    tft.print(lab);

    // button (non-editable, gray)
    uint16_t fill = ST7796S_DARKGREY;
    tft.fillRoundRect(col2X, y, col2W, btnH, 6, fill);
    tft.setTextColor(ST7796S_BLACK);
    char off[16]; snprintf(off, sizeof(off), "%+.1f mm", p.offset_mm);
    int lpx = col2X + (col2W - (int)strlen(off)*6)/2;
    tft.setCursor(lpx, y + (btnH/2)-4);
    tft.print(off);
  }

  // Draw "…" indicators if not fully visible in middle window
  tft.setTextSize(1);
  tft.setTextColor(ST7796S_DARKGREY, BACKGROUND_COLOR);
  if (startMid > 1) {
    tft.setCursor(col2X + col2W + 8, listY + 2);
    tft.print("...");
  }
  if (startMid + middleCount < (int)vt_points.size()-1) {
    tft.setCursor(col2X + col2W + 8, (listY + (btnH+rowGap)*(1 + drawn)) - 10);
    tft.print("...");
  }
}

// Jump to Back (subtitle) on long press
static inline void vt_jumpToBack() {
  vt_highlight = -1;
  vt_editMode = false;
  vt_drawScreen();
}

// Move highlight (wrap around including back)
static void vt_moveHighlight(int dir) {
  if (vt_highlight == -1) {
    // from back → go to bottom or top depending on dir
    vt_highlight = (dir > 0 ? 0 : (int)vt_points.size()-1);
    return;
  }
  int n = (int)vt_points.size();
  int h = vt_highlight + (dir > 0 ? 1 : -1);
  if (h < 0) { vt_highlight = -1; return; }       // to Back
  if (h >= n) { vt_highlight = -1; return; }      // to Back
  vt_highlight = h;
}

// Apply offset change (±0.2 mm) live, save, and move
static void vt_applyOffsetDelta(float delta_mm) {
  if (vt_highlight <= 0 || vt_highlight >= (int)vt_points.size()-1) return; // not editable (0.0 or 20.0)
  VPoint &p = vt_points[vt_highlight];
  float newOff = p.offset_mm + delta_mm;

  // save as tenths
  int mm10_old = (int)roundf(p.offset_mm * 10.0f);
  int mm10_new = (int)roundf(newOff      * 10.0f);
  if (mm10_new != mm10_old) {
    writeOffset(p.height_mm, mm10_new);
    p.offset_mm = mm10_new / 10.0f;
  }

  // Create ticks at this height if missing (capture current IK)
  int t1=0, t2=0;
  if (!readTicksIfPresent(p.height_mm, t1, t2)) {
    if (solveTicksForHeight(p.height_mm, p.offset_mm, t1, t2)) {
      writeTicks(p.height_mm, t1, t2);
      p.hasTicks = true;
    }
  }
  // Move live
  int out1=0,out2=0;
  if (solveTicksForHeight(p.height_mm, p.offset_mm, out1, out2)) {
    runGroupPoseServos({ "arm1","arm2" }, { out1, out2 });
  }
}

// Select / Edit toggle or Back action
static bool vt_onPress() {
  if (vt_highlight == -1) {
    // Back
    return true; // signal exit
  }
  if (!vt_editMode) {
    // Enter edit mode only if editable
    bool canEdit = (vt_highlight > 0) && (vt_highlight < (int)vt_points.size()-1);
    vt_editMode = canEdit;
  } else {
    // Leave edit mode
    vt_editMode = false;
  }
  return false;
}

// Encoder handling inside tuning loop
static void vt_handleEncoder() {
  long enc = rotary.read() / 4;
  long delta = enc - vt_lastEnc;
  if (delta == 0) return;
  vt_lastEnc = enc;

  if (!vt_editMode) {
    vt_moveHighlight(delta > 0 ? 1 : -1);
    // On highlight, move arms to that height (using saved offset if any)
    if (vt_highlight >= 0) {
      float mm = vt_points[vt_highlight].height_mm;
      moveArmsVertically(mm);
    }
    vt_drawScreen();
  } else {
    // edit mode: change offset by ±0.2 mm per notch
    vt_applyOffsetDelta((delta > 0) ? +0.2f : -0.2f);
    vt_drawScreen();
  }
}

// Button handling: short/long press
static bool vt_handleButton() {
  bool btn = (digitalRead(ENC_BTN) == LOW);
  unsigned long now = millis();

  if (btn && vt_btnDownAt == 0) {
    vt_btnDownAt = now;
  }
  if (!btn && vt_btnDownAt != 0) {
    // released
    unsigned long held = now - vt_btnDownAt;
    vt_btnDownAt = 0;
    if (held > 650) {
      // long press -> jump to Back
      vt_jumpToBack();
      return false;
    } else {
      // short press -> select/edit/back
      return vt_onPress();
    }
  }
  return false;
}

// --------------- PUBLIC ENTRY ---------------
void runVerticalTuningUI() {
  // Prepare model and go to MID first
  vt_buildModel();

  // Move to 0.0 mm initially as requested order bottom-up, but you asked:
  // “starts with moving the arms in the mid (in cradle position)”
  // We'll set highlight to the 0.0 entry but first move to MID (your arm1_1/arm2_1).
  {
    int t1 = getParamValue("arm1_1");
    int t2 = getParamValue("arm2_1");
    if (t1 != 0 || t2 != 0) {
      runGroupPoseServos({ "arm1","arm2" }, { t1, t2 });
    }
  }
  // Highlight bottom (0.0 mm) afterwards
  vt_highlight = 0;
  vt_editMode  = false;

  // UI state
  pinMode(ENC_BTN, INPUT_PULLUP);
  vt_lastEnc = rotary.read() / 4;
  vt_btnDownAt = 0;

  vt_drawScreen();

  // Event loop: exit when back is pressed
  while (true) {
    vt_handleEncoder();
    if (vt_handleButton()) {
      // Back pressed
      break;
    }
    delay(10);
  }
}

/************ OPTIONAL: quick helpers to pre-create a point ************/
void saveArmsMidPositionTicks(float mm, int arm1_ticks, int arm2_ticks) {
  writeTicks(mm, arm1_ticks, arm2_ticks);
}
bool getArmsMidPositionTicks(float mm, int &arm1_ticks, int &arm2_ticks) {
  return readTicksIfPresent(mm, arm1_ticks, arm2_ticks);
}


/*
What this gives you

runVerticalTuningUI()
    A full screen with title “Vertical tuning” and a selectable Back subtitle. 
    It shows a list with 20.0 mm pinned at top, 0.0 mm pinned at bottom, 
    and a scrolling window of intermediate points.
        Rotate to move highlight.
        Short press toggles edit mode (for intermediate points only).
        While editing, rotate to change offset in ±0.2 mm, moves servos live, and saves.
        Long press jumps selection to Back (top). A short press there exits.

moveArmsVertically(mm)
    Moves using:
        Exact saved ticks (if saved for that mm),
        Linear interpolation between nearest saved points,
        Extrapolation beyond last saved point,
        2-link IK fallback using mid reference to derive the vertical line and per-height offset.

Storage
    Per-height ticks: arm1_midNNN, arm2_midNNN
    Per-height offset (tenths of mm): mid_offNNN
    Angle-to-ticks calibration: arm1_0deg, arm1_90deg, arm2_0deg, arm2_90deg
    Mid reference used: arm1_1, arm2_1

If you prefer the first screen position highlighted to be mid instead of 0.0 mm, 
just set vt_highlight to the index for 10.0 mm (which is 10.0*2 = 20 in the vector) 
and call moveArmsVertically(10.0f) right after vt_buildModel().
*/