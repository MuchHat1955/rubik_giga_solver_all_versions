/********************************************************************
 * MENUS, JSON FOR MENUS, ROTARY AND MENU DISPLAY
 ********************************************************************/

// ---------- JSON (drives the menus) ----------
const char *menu_json = R"json(
{
  "root": {
    "screen": {
      "title": "rubik cube solver",
      "subtitle": { "text": "ver 6 (oct 2025)" },
      "columns": 1,
      "rows": [
        { "cells": [ { "text": "solve cube", "selectable": true, "target": "solve cube" } ] },
        { "cells": [ { "text": "read cube",  "selectable": true, "target": "read cube"  } ] },
        { "cells": [ { "text": "tests",      "selectable": true, "target": "tests"      } ] },
        { "cells": [ { "text": "random cube","selectable": true, "target": "random cube"} ] }
      ]
    },

    "solve cube": { "screen": { "title": "solve cube", "subtitle": {"text":"back","selectable": true,"action":"back"}, "columns": 1, "rows": [] } },
    "read cube":  { "screen": { "title": "read cube",  "subtitle": {"text":"back","selectable": true,"action":"back"}, "columns": 1, "rows": [] } },
    "random cube":{ "screen": { "title": "random cube","subtitle": {"text":"back","selectable": true,"action":"back"}, "columns": 1, "rows": [] } },

    "tests": {
      "screen": {
        "title": "tests",
        "subtitle": { "text": "back", "selectable": true, "action": "back" },
        "columns": 1,
        "rows": [
          { "cells": [ { "text": "servo limits", "selectable": true, "target": "servo limits" } ] },
          { "cells": [ { "text": "poses",        "selectable": true, "target": "pose poses"   } ] },
          { "cells": [ { "text": "pose groups",  "selectable": true, "target": "pose groups"  } ] },
          { "cells": [ { "text": "sequences",  "selectable": true, "target": "sequences"  } ] },
          { "cells": [ { "text": "cube moves",   "selectable": true, "target": "cube moves"   } ] }
        ]
      }
    },

    "servo limits": {
      "screen": {
        "title": "set servo limits",
        "subtitle": { "text": "back", "selectable": true, "action": "back" },
        "columns": 6,
        "rows": [
          { "cells": [
              { "text": "arm1" },
              { "text": "value", "selectable": true, "param": true, "paramKey": "arm1" },
              { "text": "from"  },
              { "text": "min",  "selectable": true, "param": true, "paramKey": "arm1_min" },
              { "text": "to"    },
              { "text": "max",  "selectable": true, "param": true, "paramKey": "arm1_max" }
          ]},
          { "cells": [
              { "text": "arm2" },
              { "text": "value", "selectable": true, "param": true, "paramKey": "arm2" },
              { "text": "from"  },
              { "text": "min",  "selectable": true, "param": true, "paramKey": "arm2_min" },
              { "text": "to"    },
              { "text": "max",  "selectable": true, "param": true, "paramKey": "arm2_max" }
          ]},
          { "cells": [
              { "text": "wrist" },
              { "text": "value", "selectable": true, "param": true, "paramKey": "wrist" },
              { "text": "from"  },
              { "text": "min",  "selectable": true, "param": true, "paramKey": "wrist_min" },
              { "text": "to"    },
              { "text": "max",  "selectable": true, "param": true, "paramKey": "wrist_max" }
          ]},
          { "cells": [
              { "text": "grip1" },
              { "text": "value", "selectable": true, "param": true, "paramKey": "grip1" },
              { "text": "from"  },
              { "text": "min",  "selectable": true, "param": true, "paramKey": "grip1_min" },
              { "text": "to"    },
              { "text": "max",  "selectable": true, "param": true, "paramKey": "grip1_max" }
          ]},
          { "cells": [
              { "text": "grip2" },
              { "text": "value", "selectable": true, "param": true, "paramKey": "grip2" },
              { "text": "from"  },
              { "text": "min",  "selectable": true, "param": true, "paramKey": "grip2_min" },
              { "text": "to"    },
              { "text": "max",  "selectable": true, "param": true, "paramKey": "grip2_max" }
          ]},
          { "cells": [
              { "text": "base"  },
              { "text": "value", "selectable": true, "param": true, "paramKey": "base" },
              { "text": "from"  },
              { "text": "min",  "selectable": true, "param": true, "paramKey": "base_min" },
              { "text": "to"    },
              { "text": "max",  "selectable": true, "param": true, "paramKey": "base_max" }
          ]}
        ]
      }
    },

    "poses": {
      "screen": {
        "title": "poses",
        "subtitle": { "text":"back", "selectable": true, "action":"back" },
        "columns": 9,
        "rows": [
          { "cells": [
              { "text":"arm1" }, { "text":"0" },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"arm1_0" },
              { "text":"row1"  },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"arm1_1" },
              { "text":"row2"  },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"arm1_2" },
              { "text":"read"  },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"arm1_read" }
          ]},
          { "cells": [
              { "text":"arm2" }, { "text":"0" },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"arm2_0" },
              { "text":"row1"  },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"arm2_1" },
              { "text":"row2"  },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"arm2_2" },
              { "text":"read"  },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"arm2_read" }
          ]},
          { "cells": [
              { "text":"wrist" }, { "text":"0°" },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"wrist_0" },
              { "text":"90°"   },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"wrist_90" },
              { "text":"180°"  },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"wrist_180" },
              { "text":"-90°"  },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"wrist_minus90" }
          ]},
          { "cells": [
              { "text":"grip" }, { "text":"closed" },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"grip_open" },
              { "text":"open"  },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"grip_closed" },
              { "text":"na" }, { "text":"na" }, { "text":"na" }, { "text":"na" }
          ]},
          { "cells": [
              { "text":"base" }, { "text":"0°" },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"base_0" },
              { "text":"90°"   },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"base_90" },
              { "text":"180°"  },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"base_180" },
              { "text":"-90°"  },
              { "text":"value", "selectable":true, "pose":true, "poseKey":"base_minus90" }
          ]}
        ]
      }
    },

    "pose groups": {
      "screen": {
        "title": "pose groups",
        "subtitle": { "text":"back", "selectable":true, "action":"back" },
        "columns": 4,
        "rows": [
          { "cells": [
              { "text":"arms 0",    "selectable":true, "groupPose":true, "groupPoseKey":"arms_0" },
              { "text":"arms row1", "selectable":true, "groupPose":true, "groupPoseKey":"arms_row1" },
              { "text":"arms row2", "selectable":true, "groupPose":true, "groupPoseKey":"arms_row2" },
              { "text":"na" }
          ]},
          { "cells": [
              { "text":"arms read 1", "selectable":true, "groupPose":true, "groupPoseKey":"arms_read1" },
              { "text":"arms read 2,   "selectable":true, "groupPose":true, "groupPoseKey":"arms_read2" },
              { "text":"na" }, { "text":"na" },
              { "text":"na" }, { "text":"na" }
          ]},
          { "cells": [
              { "text":"wrist 0°",   "selectable":true, "groupPose":true, "groupPoseKey":"wrist_0" },
              { "text":"wrist 90°",  "selectable":true, "groupPose":true, "groupPoseKey":"wrist_90" },
              { "text":"wrist 180°", "selectable":true, "groupPose":true, "groupPoseKey":"wrist_180" },
              { "text":"wrist -90°", "selectable":true, "groupPose":true, "groupPoseKey":"wrist_minus90" }
          ]}
        ]
      }
    },

    "sequences": {
      "screen": {
        "title": "sequences",
        "subtitle": { "text":"back", "selectable":true, "action":"back" },
        "columns": 4,
        "rows": [
          { "cells": [
              { "text":"bottom+", "selectable":true, "sequence":true, "sequenceKey":"bottom_plus" },
              { "text":"bottom-", "selectable":true, "sequence":true, "sequenceKey":"bottom_minus" },
              { "text":"front to base",      "selectable":true, "sequence":true, "sequenceKey":"front_to_base" },
              { "text":"back to base",      "selectable":true, "sequence":true, "sequenceKey":"back_to_base" }
          ]},
          { "cells": [
              { "text":"left to base",   "selectable":true, "sequence":true, "sequenceKey":"left_to_base" },
              { "text":"right to base",   "selectable":true, "sequence":true, "sequenceKey":"right_to_base" },
              { "text":"top to base", "selectable":true, "sequence":true, "sequenceKey":"top_to_base" },
              { "text":"na"}
          ]},
          { "cells": [
                { "text":"read top", "selectable":true, "sequence":true, "sequenceKey":"read_top" }
                { "text":"read middle", "selectable":true, "sequence":true, "sequenceKey":"read_middle" }
                { "text":"na"}
                { "text":"na"}
          ]}
        ]
      }
    },

    "cube moves": {
      "screen": {
        "title": "cube moves",
        "subtitle": { "text":"back", "selectable":true, "action":"back" },
        "columns": 4,
        "rows": [
          { "cells": [
              { "text":"f+", "selectable":true, "sequence":true, "sequenceKey":"f_plus" },
              { "text":"f-", "selectable":true, "sequence":true, "sequenceKey":"f_minus" },
              { "text":"b+",      "selectable":true, "sequence":true, "sequenceKey":"b_plus" },
              { "text":"b-",      "selectable":true, "sequence":true, "sequenceKey":"b_minus" }
          ]},
          { "cells": [
              { "text":"u+",   "selectable":true, "sequence":true, "sequenceKey":"u_plus" },
              { "text":"u+",   "selectable":true, "sequence":true, "sequenceKey":"u_minus" },
              { "text":"d+", "selectable":true, "sequence":true, "sequenceKey":"d_plus" },
              { "text":"b-",      "selectable":true, "sequence":true, "sequenceKey":"d_minus" }
          ]},
          { "cells": [
              { "text":"l+", "selectable":true, "sequence":true, "sequenceKey":"l_plus" },
              { "text":"l-", "selectable":true, "sequence":true, "sequenceKey":"l_minus" },
              { "text":"r+",      "selectable":true, "sequence":true, "sequenceKey":"r_plus" },
              { "text":"r-",      "selectable":true, "sequence":true, "sequenceKey":"r_minus" }
          ]},
          { "cells": [
              { "text":"f++", "selectable":true, "sequence":true, "sequenceKey":"f_plusplus" },
              { "text":"b++",  "selctable":true, "sequence":true, "sequenceKey":"b_plusplus" },
              { "text":"u++",      "selectable":true, "sequence":true, "sequenceKey":"u_plusplus" },
              { "text":"d++",      "selectable":true, "sequence":true, "sequenceKey":"d_plusplus" }
          ]}.
          { "cells": [
              { "text":"l++", "selectable":true, "sequence":true, "sequenceKey":"l_plusplus" },
              { "text":"r++", "selectable":true, "sequence":true, "sequenceKey":"r_plusplus" },
              { "text":"na },
              { "text":"na"}
          ]}
        ]
      }
    }
  }
}
)json";

// ---------- JSON doc ----------
static const size_t DOC_SIZE = 64 * 1024;
DynamicJsonDocument doc(DOC_SIZE);

// ---------- Menu state ----------
static const size_t MAX_SELECTABLE = 96;

JsonObject current_page;
std::vector<JsonObject> page_stack;

struct SelEntry {
  bool isSubtitle{ false };
  int row{ -1 };
  int cell{ -1 };
  JsonObject cellObj;
};
std::vector<SelEntry> selectableList;

int selectableCount = 0;
int highlightIndex = 0;
int selectedIndex = -1;  // -1: none (editing when >=0)
int first_visible_row = 0;

// ---------- Box render cache (dirty redraw) ----------
struct BoxState {
  bool exists{ false };
  bool highlighted{ false };
  bool selected{ false };  // editing
  bool active{ false };    // servo in pose / sequence running, etc.
  String text;
  int16_t x{ 0 }, y{ 0 }, w{ 0 }, h{ 0 };
  bool isNumber{ false };
};

std::map<uint32_t, BoxState> lastDrawn;  // key = (row<<16)|cell for visible rows

uint32_t keyForRC(int r, int c) {
  return (uint32_t)((r & 0xFFFF) << 16) | (uint16_t)(c & 0xFFFF);
}

// ---------- Low-level helpers ----------
int textW(const String &s, int size) {
  return s.length() * 6 * size;
}

void drawActiveDot(int16_t cx, int16_t cy, bool on) {
  tft.fillCircle(cx, cy, 4, on ? ACTIVE_DOT_COLOR : INACTIVE_DOT_COLOR);
}

// Render a single box (button/number/label)
//  - states: normal / highlighted / selected(editing)
//  - active indicator: small dot at top-left inside the box
void drawBox(int16_t x, int16_t y, int16_t w, int16_t h,
             const String &text, bool selectable, bool highlighted, bool selected,
             bool active, bool isNumber, uint8_t textSize = ROW_TEXT_SIZE) {

  // background & frames
  uint16_t fill = BOX_FILL_COLOR;
  if (selected) fill = SELECT_COLOR;
  else if (highlighted) fill = BG_COLOR;

  // base fill (clear previous)
  tft.fillRoundRect(x, y, w, h, CORNER_R, fill);

  // frame for non-selected
  if (!selected) {
    uint16_t frame = highlighted ? HILIGHT_COLOR : BOX_FRAME_COLOR;
    tft.drawRoundRect(x, y, w, h, CORNER_R, frame);
  }

  // active indicator (top-left inside)
  if (selectable) {
    drawActiveDot(x + 8, y + 8, active);
  }

  // text color
  tft.setTextSize(textSize);
  tft.setTextColor(selected ? BG_COLOR : ROW_TEXT_COLOR);

  // for numbers we center on a fixed-width area (4 chars default)
  String disp = text;
  if (isNumber) {
    // fixed 4 width; pad/clip
    if ((int)disp.length() < 4) {
      while ((int)disp.length() < 4) disp = " " + disp;
    } else if ((int)disp.length() > 4) {
      disp = disp.substring(disp.length() - 4);
    }
  }

  int tw = textW(disp, textSize);
  int tx = x + (w - tw) / 2;
  int ty = y + (h / 2) - (8 * textSize / 2);

  // protect bounds
  if (tx < x + 2) tx = x + 2;
  if (ty < y + 2) ty = y + 2;

  tft.setCursor(tx, ty);
  tft.print(disp);

  // subtle pressed shading
  if (selected) {
    tft.drawFastHLine(x + 2, y + h - 2, w - 4, PRESSED_SHADE);
  }
}

// ---------- Build selectable list for a page ----------
void rebuildSelectableList() {
  selectableList.clear();
  selectableCount = 0;

  if (current_page.isNull()) return;

  if (current_page.containsKey("subtitle")) {
    JsonObject sub = current_page["subtitle"];
    if (sub["selectable"].as<bool>()) {
      SelEntry e;
      e.isSubtitle = true;
      e.cellObj = sub;
      selectableList.push_back(e);
      selectableCount++;
    }
  }

  JsonArray rows = current_page["rows"];
  for (int r = 0; r < (int)rows.size(); ++r) {
    JsonArray cells = rows[r]["cells"];
    for (int c = 0; c < (int)cells.size(); ++c) {
      JsonObject cell = cells[c];
      if (cell["selectable"].as<bool>()) {
        SelEntry e;
        e.isSubtitle = false;
        e.row = r;
        e.cell = c;
        e.cellObj = cell;
        selectableList.push_back(e);
        selectableCount++;
      }
    }
  }
  if (selectableCount == 0) {
    highlightIndex = 0;
    selectedIndex = -1;
  } else {
    if (highlightIndex < 0) highlightIndex = 0;
    if (highlightIndex >= selectableCount) highlightIndex = selectableCount - 1;
    if (selectedIndex >= selectableCount) selectedIndex = -1;
  }
}

int rowOfSel(int selIdx) {
  if (selIdx < 0 || selIdx >= (int)selectableList.size()) return -1;
  return selectableList[selIdx].isSubtitle ? -1 : selectableList[selIdx].row;
}

// ---------- Active-state hooks (wire real logic later) ----------
bool isPoseActive(const String &poseKey) {
  // TODO: compare actual servo positions with tolerance
  (void)poseKey;
  return false;
}
bool isGroupPoseActive(const String &groupPoseKey) {
  (void)groupPoseKey;
  return false;
}
bool isSequenceActive(const String &sequenceKey) {
  (void)sequenceKey;
  return false;
}

// ---------- Action dispatch (stub) ----------
void safeActionExecute(const String &key) {
  Serial.printf("Action: %s\n", key.c_str());
  // TODO: call your real Pose/GroupPose/Sequence executor
}

// ---------- Compute dynamic column widths for a row ----------
void computeRowColumnWidths(JsonArray cells, int columns, int rowY, std::vector<int> &xs, std::vector<int> &ws) {
  xs.clear();
  ws.clear();
  const int screenW = tft.width();

  // estimate width for each logical column (uniform per page column slot)
  // Strategy: for each cell, compute needed width from its display text.
  // Then, cap minimum to 80 px for number boxes, 72 px for labels.
  std::vector<int> need;
  need.reserve(cells.size());

  for (JsonObject cell : cells) {
    String disp;
    bool isNumber = false;

    if (cell["param"].as<bool>() || cell["pose"].as<bool>()) {
      String k = cell["paramKey"].as<String>();
      if (!k.length()) k = cell["poseKey"].as<String>();
      int v = getParamValue(k);
      disp = String(v);
      isNumber = true;
    } else {
      disp = cell["text"].as<String>();
      if (disp.equalsIgnoreCase("na")) disp = "";  // empty spacer
    }

    int size = ROW_TEXT_SIZE;
    int w = textW(disp.length() ? disp : " ", size) + 2 * CELL_PAD + 20;  // padding + wiggle
    if (isNumber) w = max(w, 80);
    else w = max(w, 72);
    need.push_back(w);
  }

  // Sum, then if exceeding screenW, squeeze proportionally; else, left-align, keep requested widths
  int sum = 0;
  for (int v : need) sum += v;
  int available = screenW - 12;  // margins
  float scale = (sum > available) ? (float)available / (float)sum : 1.0f;

  int x = 6;
  for (size_t i = 0; i < need.size(); ++i) {
    int w = (int)(need[i] * scale);
    if (w < 50) w = 50;
    xs.push_back(x);
    ws.push_back(w);
    x += w + 4;  // small gap
  }
}

// ---------- Page draw (only redraw dirty boxes) ----------
void drawMenuPage(JsonObject &page) {
  // full clear only on full page switch
  tft.fillScreen(BG_COLOR);
  lastDrawn.clear();

  // Title
  String title = page["title"].as<String>();
  tft.setTextSize(TITLE_SIZE);
  tft.setTextColor(TITLE_COLOR);
  int tW = textW(title, TITLE_SIZE);
  tft.setCursor((tft.width() - tW) / 2, 4);
  tft.print(title);

  // Subtitle box
  int subY = 40, subH = 28, subX = 10, subW = tft.width() - 20;
  bool subSelectable = false, subHL = false, subSel = false;
  String subText = "";
  if (page.containsKey("subtitle")) {
    JsonObject sub = page["subtitle"];
    subText = sub["text"].as<String>();
    subSelectable = sub["selectable"].as<bool>();
  }
  if (subSelectable && !selectableList.empty() && selectableList[0].isSubtitle) {
    subHL = (highlightIndex == 0);
    subSel = (selectedIndex == 0);
  }
  drawBox(subX, subY, subW, subH, subText, subSelectable, subHL, subSel, false, false, SUBTITLE_SIZE);

  // Rows
  int y = subY + subH + 8;
  int columns = page["columns"].as<int>();
  JsonArray rows = page["rows"];
  int numRows = rows.size();

  // scroll window based on highlight
  int hr = rowOfSel(highlightIndex);
  if (hr >= 0) {
    if (hr < first_visible_row) first_visible_row = hr;
    if (hr >= first_visible_row + VISIBLE_ROWS) first_visible_row = hr - VISIBLE_ROWS + 1;
  } else first_visible_row = 0;

  first_visible_row = max(0, min(first_visible_row, max(0, numRows - VISIBLE_ROWS)));

  for (int r = first_visible_row; r < first_visible_row + VISIBLE_ROWS && r < numRows; ++r) {
    JsonArray cells = rows[r]["cells"];

    // compute per-cell x/width for this row
    std::vector<int> xs, ws;
    computeRowColumnWidths(cells, columns, y, xs, ws);

    for (int c = 0; c < (int)cells.size(); ++c) {
      JsonObject cell = cells[c];
      String label = cell["text"].as<String>();
      if (label.equalsIgnoreCase("na")) {
        // empty spacer box — draw thin frame to keep grid consistent or skip completely
        // Here: draw a faint frame
        int16_t bx = xs[c], by = y, bw = ws[c], bh = ROW_H - 6;
        tft.drawRoundRect(bx, by, bw, bh, CORNER_R, BOX_FRAME_COLOR);
        continue;
      }

      bool isSelectable = cell["selectable"].as<bool>();
      // which selectable index is this?
      int selIdx = -1;
      for (int i = 0; i < (int)selectableList.size(); ++i) {
        if (!selectableList[i].isSubtitle && selectableList[i].row == r && selectableList[i].cell == c) {
          selIdx = i;
          break;
        }
      }
      bool isHL = (selIdx != -1 && selIdx == highlightIndex);
      bool isSel = (selIdx != -1 && selIdx == selectedIndex);

      // content display
      String disp;
      bool isNumber = false;
      bool isActive = false;

      if (cell["param"].as<bool>()) {
        String key = cell["paramKey"].as<String>();
        disp = String(getParamValue(key));
        isNumber = true;
      } else if (cell["pose"].as<bool>()) {
        String key = cell["poseKey"].as<String>();
        disp = String(getParamValue(key));
        isNumber = true;
        isActive = isPoseActive(key);
      } else if (cell["groupPose"].as<bool>()) {
        String key = cell["groupPoseKey"].as<String>();
        disp = label;
        isActive = isGroupPoseActive(key);
      } else if (cell["sequence"].as<bool>()) {
        String key = cell["sequenceKey"].as<String>();
        disp = label;
        isActive = isSequenceActive(key);
      } else {
        disp = label;
      }

      // dirty check
      uint32_t k = keyForRC(r, c);
      BoxState cur;
      cur.exists = true;
      cur.highlighted = isHL;
      cur.selected = isSel;
      cur.active = isActive;
      cur.text = disp;
      cur.x = xs[c];
      cur.y = y;
      cur.w = ws[c];
      cur.h = ROW_H - 6;
      cur.isNumber = isNumber;

      bool needRedraw = true;
      auto it = lastDrawn.find(k);
      if (it != lastDrawn.end()) {
        const BoxState &old = it->second;
        needRedraw = (old.highlighted != cur.highlighted || old.selected != cur.selected || old.active != cur.active || old.text != cur.text || old.w != cur.w || old.h != cur.h);
      }
      if (needRedraw) {
        drawBox(cur.x, cur.y, cur.w, cur.h, cur.text, isSelectable, cur.highlighted, cur.selected, cur.active, cur.isNumber);
        lastDrawn[k] = cur;
      }
    }
    y += ROW_H;
  }
}

// ---------- Encoder/Buttons ----------
int encoderDiv = 4;
long lastEnc = 0;
unsigned long lastBtnAt = 0;
bool lastBtnState = HIGH;

void ensureVisibleAfterMove() {
  int hr = rowOfSel(highlightIndex);
  if (hr >= 0) {
    if (hr < first_visible_row) first_visible_row = hr;
    if (hr >= first_visible_row + VISIBLE_ROWS) first_visible_row = hr - VISIBLE_ROWS + 1;
  } else first_visible_row = 0;
}

void applyEncoderDelta(long delta) {
  if (delta == 0 || selectableCount == 0) return;

  if (selectedIndex == -1) {
    // move highlight
    highlightIndex += (delta > 0 ? 1 : -1);
    if (highlightIndex < 0) highlightIndex = selectableCount - 1;
    if (highlightIndex >= selectableCount) highlightIndex = 0;
    ensureVisibleAfterMove();
    drawMenuPage(current_page);
    return;
  }

  // editing numeric (param/pose) if selectedIndex >= 0
  SelEntry entry = selectableList[selectedIndex];
  if (entry.isSubtitle) return;
  JsonObject cell = entry.cellObj;

  bool isParam = cell["param"].as<bool>();
  bool isPose = cell["pose"].as<bool>();
  if (!isParam && !isPose) return;

  String key = isParam ? cell["paramKey"].as<String>() : cell["poseKey"].as<String>();
  int cur = getParamValue(key);
  int minv = getParamMin(key);
  int maxv = getParamMax(key);

  int step = (abs(delta) >= 2) ? 4 : 1;  // accelerate with faster twist
  cur += (delta > 0 ? step : -step);
  cur = constrain(cur, minv, maxv);

  setParamValue(key, cur);
  drawMenuPage(current_page);
}

void handleButtonPress() {
  unsigned long now = millis();
  if (now - lastBtnAt < 180) return;
  lastBtnAt = now;

  if (selectableCount == 0) return;
  SelEntry entry = selectableList[highlightIndex];

  // subtitle “back”
  if (entry.isSubtitle) {
    JsonObject sub = entry.cellObj;
    if (String(sub["action"].as<const char *>()) == "back") {
      if (page_stack.size() > 1) {
        page_stack.pop_back();
        current_page = page_stack.back();
        rebuildSelectableList();
        selectedIndex = -1;
        highlightIndex = 0;
        first_visible_row = 0;
        drawMenuPage(current_page);
      }
    } else {
      // could toggle select state for subtitle if needed
      selectedIndex = (selectedIndex == highlightIndex) ? -1 : highlightIndex;
      drawMenuPage(current_page);
    }
    return;
  }

  JsonObject cell = entry.cellObj;

  // navigate to target page
  if (cell.containsKey("target")) {
    String target = cell["target"].as<String>();
    JsonObject root = doc["root"].as<JsonObject>();
    if (root.containsKey(target)) {
      JsonObject screen = root[target]["screen"].as<JsonObject>();
      page_stack.push_back(screen);
      current_page = screen;
      rebuildSelectableList();
      selectedIndex = -1;
      highlightIndex = 0;
      first_visible_row = 0;
      drawMenuPage(current_page);
      return;
    }
  }

  // action triggers
  if (cell["pose"].as<bool>()) {
    String k = cell["poseKey"].as<String>();
    safeActionExecute(k);
    return;
  }
  if (cell["groupPose"].as<bool>()) {
    String k = cell["groupPoseKey"].as<String>();
    safeActionExecute(k);
    return;
  }
  if (cell["sequence"].as<bool>()) {
    String k = cell["sequenceKey"].as<String>();
    safeActionExecute(k);
    return;
  }

  // toggle select (enter/exit edit)
  selectedIndex = (selectedIndex == highlightIndex) ? -1 : highlightIndex;
  drawMenuPage(current_page);
}