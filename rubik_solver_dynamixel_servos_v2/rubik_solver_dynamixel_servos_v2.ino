/* Unified ESP32 menu sketch
   - Adafruit_ST7796S_kbv 480x320
   - Static JSON menu (root.screen title/subtitle from JSON)
   - Rotary encoder + button
   - Scrolling (3 visible rows), centered columns
   - Select / Highlight / Param editing with min/max
   - Back button handling via "action":"back" in subtitle
*/

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7796S_kbv.h>
#include <Encoder.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>

// -------------------- Hardware pins (adjust to your wiring) -------------------- TODO
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 17

// Rotary encoder pins
#define ENC_A 18
#define ENC_B 19
#define ENC_BTN 21

Adafruit_ST7796S_kbv tft(TFT_CS, TFT_DC, TFT_RST);
Encoder rotary(ENC_A, ENC_B);

// -------------------- Visual config --------------------
#define VISIBLE_ROWS 3
#define TITLE_COLOR ST7796S_WHITE
#define SUBTITLE_COLOR ST7796S_YELLOW
#define ROW_TEXT_COLOR ST7796S_WHITE
#define HIGHLIGHT_COLOR ST7796S_CYAN
#define SELECT_COLOR ST7796S_CYAN
#define BACKGROUND_COLOR ST7796S_BLACK

#define TITLE_FONT_SIZE 3
#define SUBTITLE_FONT_SIZE 2
#define ROW_FONT_SIZE 2

// -------------------- JSON Menu (root.screen contains landing page title/subtitle) --------------------
const char *menu_json = R"rawliteral(
{
  "root": {
    "screen": {
      "title": "rubik cube solver",
      "subtitle": {
        "text": "ver 1 oct 2025",     
      },
      "columns": 1,
      "rows": [
        {
          "cells": [
            {
              "text": "solve cube",
              "selectable": true,
              "target": "solve cube"
            }
          ]
        },
        {
          "cells": [
            {
              "text": "read cube",
              "selectable": true,
              "target": "read cube"
            }
          ]
        },
        {
          "cells": [
            {
              "text": "tests",
              "selectable": true,
              "target": "tests"
            }
          ]
        },
        {
          "cells": [
            {
              "text": "random cube",
              "selectable": true,
              "target": "random cube"
            }
          ]
        }
      ]
    },
    "solve cube": {
      "screen": {
        "title": "solve cube",
        "subtitle": {
          "text": "back",
          "selectable": true,
          "action": "back"
        },
        "columns": 1,
        "rows": [
        ]
      }
    },
    "read cube": {
      "screen": {
        "title": "read cube",
        "subtitle": {
          "text": "back",
          "selectable": true,
          "action": "back"
        },
        "columns": 1,
        "rows": [
        ]
      }
    },
    "random cube": {
      "screen": {
        "title": "random cube",
        "subtitle": {
          "text": "back",
          "selectable": true,
          "action": "back"
        },
        "columns": 1,
        "rows": [
        ]
      }
    },
    "tests": {
      "screen": {
        "title": "tests",
        "subtitle": {
          "text": "back",
          "selectable": true,
          "action": "back"
        },
        "columns": 1,
        "rows": [
          {
            "cells": [
              {
                "text": "servo limits",
                "selectable": true,
                "target": "servo limits"
              }
            ]
          },
          {
            "cells": [
              {
                "text": "presets",
                "selectable": true,
                "target": "preset positions"
              }
            ]
          },
          {
            "cells": [
              {
                "text": "servo moves",
                "selectable": true,
                "target": "servo moves"
              }
            ]
          },
          {
            "cells": [
              {
                "text": "cube moves",
                "selectable": true,
                "target": "cube moves"
              }
            ]
          }
        ]
      }
    },
    "servo limits": {
      "screen": {
        "title": "set servo limits",
        "subtitle": {
          "text": "back",
          "selectable": true,
          "action": "back"
        },
        "columns": 6,
        "rows": [
          {
            "cells": [
              {
                "text": "arm1",               
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "arm1"
              },
              {
                "text": "from",               
              },
              {
                "text": "min",
                "selectable": true,
                "param": true,
                "paramKey": "arm1_min"
              },
              {
                "text": "to",
              },
              {
                "text": "max",
                "selectable": true,
                "param": true,
                "paramKey": "arm1_max"
              }
            ]
          },
          {
            "cells": [
              {
                "text": "arm2",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "arm2"
              },
              {
                "text": "from",
              },
              {
                "text": "min",
                "selectable": true,
                "param": true,
                "paramKey": "arm2_min"
              },
              {
                "text": "to",
              },
              {
                "text": "max",
                "selectable": true,
                "param": true,
                "paramKey": "arm2_max"
              }
            ]
          },
          {
            "cells": [
              {
                "text": "wrist",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "wrist"
              },
              {
                "text": "from",
              },
              {
                "text": "min",
                "selectable": true,
                "param": true,
                "paramKey": "wrist_min"
              },
              {
                "text": "to",
              },
              {
                "text": "max",
                "selectable": true,
                "param": true,
                "paramKey": "wrist_max"
              }
            ]
          },
          {
            "cells": [
              {
                "text": "grip1",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "grip1"
              },
              {
                "text": "from",
              },
              {
                "text": "min",
                "selectable": true,
                "param": true,
                "paramKey": "grip1_min"
              },
              {
                "text": "to",
              },
              {
                "text": "max",
                "selectable": true,
                "param": true,
                "paramKey": "grip1_max"
              }
            ]
          },
          {
            "cells": [
              {
                "text": "grip2",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "grip2"
              },
              {
                "text": "from",
              },
              {
                "text": "min",
                "selectable": true,
                "param": true,
                "paramKey": "grip2_min"
              },
              {
                "text": "to",
              },
              {
                "text": "max",
                "selectable": true,
                "param": true,
                "paramKey": "grip2_max"
              }
            ]
          },
          {
            "cells": [
              {
                "text": "base",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "base"
              },
              {
                "text": "from",
              },
              {
                "text": "min",
                "selectable": true,
                "param": true,
                "paramKey": "base_min"
              },
              {
                "text": "to",
              },
              {
                "text": "max",
                "selectable": true,
                "param": true,
                "paramKey": "base_max"
              }
            ]
          }
        ]
      }
    },
    "preset positions": {
      "screen": {
        "title": "preset positions",
        "subtitle": {
          "text": "back",
          "selectable": true,
          "action": "back"
        },
        "columns": 9,
        "rows": [
          {
            "cells": [
              {
                "text": "arm1",
              },
              {
                "text": "low",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "arm1_0"
              },
              {
                "text": "mid",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "arm1_1"
              },
              {
                "text": "top",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "arm1_2"
              },
              {
                "text": "na",
              },
              {
                "text": "na",
              }
            ]
          },
          {
            "cells": [
              {
                "text": "arm2",
              },
              {
                "text": "low",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "arm2_0"
              },
              {
                "text": "mid",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "arm2_1"
              },
              {
                "text": "top",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "arm2_2"
              },
              {
                "text": "na",
              },
              {
                "text": "na",
              }
            ]
          },
          {
            "cells": [
              {
                "text": "wrist",
              },
              {
                "text": "0°",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "wrist_0"
              },
              {
                "text": "90°",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "wrist_90"
              },
              {
                "text": "180°",
                "selectable": true
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "wrist_180"
              },
              {
                "text": "-90°",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "wrist_minus90"
              }
            ]
          },
          {
            "cells": [
              {
                "text": "grip",
              },
              {
                "text": "closed",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "grip_0"
              },
              {
                "text": "open",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "grip_1"
              },
              {
                "text": "na",
              },
              {
                "text": "na",
              },
              {
                "text": "na",
              },
              {
                "text": "na",
              }
            ]
          },
          {
            "cells": [
              {
                "text": "base",
              },
              {
                "text": "0°",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "base_0"
              },
              {
                "text": "90°",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "base_90"
              },
              {
                "text": "180°",
                "selectable": true
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "base_180"
              },
              {
                "text": "-90°",
              },
              {
                "text": "value",
                "selectable": true,
                "param": true,
                "paramKey": "base_minus90"
              }
            ]
          }
        ]
      }
    },
    "servo moves": {
      "screen": {
        "title": "servo moves",
        "subtitle": {
          "text": "back",
          "selectable": true,
          "action": "back"
        },
        "columns": 4,
        "rows": [
          {
            "cells": [
              {
                "text": "arms low",
                "selectable": true
              },
              {
                "text": "arms mid",
                "selectable": true
              },
              {
                "text": "arms top",
                "selectable": true
              },
              {
                "text": "na",
              }
            ]
          },
          {
            "cells": [
              {
                "text": "grip closed",
                "selectable": true
              },
              {
                "text": "grip open",
                "selectable": true
              },
              {
                "text": "na",
              },
              {
                "text": "na",
              }
            ]
          },
          {
            "cells": [
              {
                "text": "wrist 0°",
                "selectable": true
              },
              {
                "text": "wrist 90°",
                "selectable": true
              },
              {
                "text": "wrist 180°",
                "selectable": true
              },
              {
                "text": "wrist -90°",
                "selectable": true
              }
            ]
          },
          {
            "cells": [
              {
                "text": "base 0°",
                "selectable": true
              },
              {
                "text": "base 90°",
                "selectable": true
              },
              {
                "text": "base 180°",
                "selectable": true
              },
              {
                "text": "base -90°",
                "selectable": true
              }
            ]
          }
        ]
      }
    },
    "cube moves": {
      "screen": {
        "title": "cube moves",
        "subtitle": {
          "text": "back",
          "selectable": true,
          "action": "back"
        },
        "columns": 4,
        "rows": [
          {
            "cells": [
              {
                "text": "front+",
                "selectable": true
              },
              {
                "text": "front-",
                "selectable": true
              },
              {
                "text": "back+",
                "selectable": true
              },
              {
                "text": "back-",
                "selectable": true
              }
            ]
          },
          {
            "cells": [
              {
                "text": "left+",
                "selectable": true
              },
              {
                "text": "left-",
                "selectable": true
              },
              {
                "text": "right+",
                "selectable": true
              },
              {
                "text": "right-",
                "selectable": true
              }
            ]
          },
          {
            "cells": [
              {
                "text": "top+",
                "selectable": true
              },
              {
                "text": "top-",
                "selectable": true
              },
              {
                "text": "bottom+",
                "selectable": true
              },
              {
                "text": "bottom-",
                "selectable": true
              }
            ]
          }
        ]
      }
    }
  }
}
)rawliteral";

// -------------------- JSON doc size --------------------
static const size_t DOC_SIZE = 64 * 1024;
DynamicJsonDocument doc(DOC_SIZE);

// -------------------- Param storage and helpers --------------------
struct ParamDef {
  int value;
  int minv;
  int maxv;
};
std::map<String, ParamDef> paramStore;

int getParamValue(const String &k) {
  if (paramStore.find(k) == paramStore.end()) return 0;
  return paramStore[k].value;
}
int getParamMin(const String &k) {
  if (paramStore.find(k) == paramStore.end()) return 0;
  return paramStore[k].minv;
}
int getParamMax(const String &k) {
  if (paramStore.find(k) == paramStore.end()) return 0;
  return paramStore[k].maxv;
}
void setParamValue(const String &k, int v) {
  if (paramStore.find(k) == paramStore.end()) return;
  ParamDef &p = paramStore[k];
  if (v < p.minv) v = p.minv;
  if (v > p.maxv) v = p.maxv;
  p.value = v;
}

// -------------------- Menu state --------------------
JsonObject current_page;             // current screen JsonObject (page["screen"])
std::vector<JsonObject> page_stack;  // stack for Back
int selectableCount = 0;             // number of selectable items on the page (including subtitle if selectable)
int highlightIndex = 0;              // current highlight index among selectable items (0..selectableCount-1)
int selectedIndex = -1;              // selected index (if any), -1 = none
int first_visible_row = 0;           // top visible row index (for scrolling)
const int row_height = 40;           // px per row
int encoderDiv = 4;                  // divide encoder.read() for sensitivity
unsigned long lastBtnMillis = 0;
bool btnLast = HIGH;

// Helper struct for mapping selectable index -> location
struct SelEntry {
  bool isSubtitle;     // true = subtitle
  int row;             // row index (if isSubtitle==false)
  int cell;            // cell index in that row
  JsonObject cellObj;  // reference to cell object (valid while doc exists)
};
std::vector<SelEntry> selectableList;  // rebuilt each page change

// -------------------- Utility: center text helpers --------------------
int textWidthForSize(const String &s, int size) {
  return s.length() * 6 * size;
}

// -------------------- Build selectable list for page --------------------
void rebuildSelectableList() {
  selectableList.clear();
  selectableCount = 0;

  if (current_page.isNull()) return;

  // subtitle first if selectable
  if (current_page.containsKey("subtitle")) {
    JsonObject sub = current_page["subtitle"];
    if (sub.containsKey("selectable") && sub["selectable"].as<bool>()) {
      SelEntry e;
      e.isSubtitle = true;
      e.row = -1;
      e.cell = -1;
      e.cellObj = sub;
      selectableList.push_back(e);
      selectableCount++;
    }
  }

  JsonArray rows = current_page["rows"];
  for (int r = 0; r < (int)rows.size(); r++) {
    JsonArray cells = rows[r]["cells"];
    for (int c = 0; c < (int)cells.size(); c++) {
      JsonObject cell = cells[c];
      if (cell.containsKey("selectable") && cell["selectable"].as<bool>()) {
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

  // ensure highlightIndex in range
  if (selectableCount == 0) {
    highlightIndex = 0;
    selectedIndex = -1;
  } else {
    if (highlightIndex < 0) highlightIndex = 0;
    if (highlightIndex >= selectableCount) highlightIndex = selectableCount - 1;
    if (selectedIndex >= selectableCount) selectedIndex = -1;
  }
}

// -------------------- Find row index for a given selectable index (used for scrolling) ----
int findRowForSelectableIndex(int selIdx) {
  if (selIdx < 0 || selIdx >= (int)selectableList.size()) return 0;
  return selectableList[selIdx].isSubtitle ? -1 : selectableList[selIdx].row;
}

// -------------------- Drawing ----
void drawMenuPage(JsonObject &page) {
  tft.fillScreen(BACKGROUND_COLOR);

  // --- Title ---
  String title = page["title"].as<String>();
  tft.setTextSize(TITLE_FONT_SIZE);
  tft.setTextColor(TITLE_COLOR);
  int titleW = textWidthForSize(title, TITLE_FONT_SIZE);
  tft.setCursor((tft.width() - titleW) / 2, 4);
  tft.print(title);

  // --- Subtitle (draw as a centered "button" area at y=40) ---
  JsonObject subtitle = page["subtitle"];
  String subtitleText = subtitle["text"].as<String>();
  tft.setTextSize(SUBTITLE_FONT_SIZE);
  int subH = 28;
  int subY = 40;
  int subW = tft.width() - 20;
  int subX = 10;
  bool subSelectable = subtitle.containsKey("selectable") && subtitle["selectable"].as<bool>();
  bool subIsHighlighted = false;
  if (subSelectable && selectableList.size() > 0 && selectableList[0].isSubtitle) {
    if (highlightIndex == 0) subIsHighlighted = true;
  }

  // Draw subtitle background & highlight/selected visuals
  if (subSelectable && selectedIndex == 0) {
    tft.fillRoundRect(subX, subY, subW, subH, 6, SELECT_COLOR);
    tft.setTextColor(BACKGROUND_COLOR);
  } else {
    tft.fillRoundRect(subX, subY, subW, subH, 6, BACKGROUND_COLOR);
    tft.setTextColor(SUBTITLE_COLOR);
    if (subSelectable && subIsHighlighted && selectedIndex != 0) {
      tft.drawRoundRect(subX, subY, subW, subH, 6, HIGHLIGHT_COLOR);
    }
  }
  int subTextW = textWidthForSize(subtitleText, SUBTITLE_FONT_SIZE);
  tft.setCursor((tft.width() - subTextW) / 2, subY + (subH - 8));
  tft.print(subtitleText);

  // --- Rows (visible window of VISIBLE_ROWS) ---
  int columns = page["columns"].as<int>();
  JsonArray rows = page["rows"];
  int numRows = rows.size();

  // compute which row is highlighted currently
  int highlight_row = findRowForSelectableIndex(highlightIndex);
  if (highlight_row == -1) {
    // subtitle highlighted -> keep first_visible_row as 0
  } else {
    // adjust first_visible_row
    if (highlight_row < first_visible_row) first_visible_row = highlight_row;
    if (highlight_row >= first_visible_row + VISIBLE_ROWS) first_visible_row = highlight_row - VISIBLE_ROWS + 1;
    if (first_visible_row < 0) first_visible_row = 0;
    if (first_visible_row > max(0, numRows - VISIBLE_ROWS)) first_visible_row = max(0, numRows - VISIBLE_ROWS);
  }

  // drawing rows starting y:
  int y = subY + subH + 8;

  for (int r = first_visible_row; r < first_visible_row + VISIBLE_ROWS && r < numRows; r++) {
    JsonArray cells = rows[r]["cells"];
    int numCells = cells.size();
    int colWidth = tft.width() / columns;
    int totalWidth = numCells * colWidth;
    int x_offset = (tft.width() - totalWidth) / 2;  // center if fewer cells than columns

    for (int c = 0; c < numCells; c++) {
      JsonObject cell = cells[c];
      int x = x_offset + c * colWidth + 6;
      int w = colWidth - 12;
      int h = row_height - 10;

      // Determine if this cell corresponds to a selectable entry and if it's highlighted/selected
      bool isSelectable = cell.containsKey("selectable") && cell["selectable"].as<bool>();
      int selIdx = -1;
      for (int i = 0; i < (int)selectableList.size(); ++i) {
        if (!selectableList[i].isSubtitle && selectableList[i].row == r && selectableList[i].cell == c) {
          selIdx = i;
          break;
        }
      }
      bool isHighlighted = (selIdx != -1 && selIdx == highlightIndex);
      bool isSelected = (selIdx != -1 && selIdx == selectedIndex);

      // compute display text
      String display_text;
      if (cell.containsKey("param") && cell["param"].as<bool>()) {
        String key = cell.containsKey("paramKey") ? cell["paramKey"].as<String>() : cell["text"].as<String>();
        int v = getParamValue(key);
        display_text = String(v);
      } else {
        display_text = cell["text"].as<String>();
      }

      // Draw selected (filled) or normal
      if (isSelected) {
        tft.fillRect(x - 4, y - 2, w + 8, h + 4, SELECT_COLOR);
        tft.setTextColor(BACKGROUND_COLOR);
      } else {
        tft.setTextColor(ROW_TEXT_COLOR);
      }

      // Draw highlight as rounded rect if highlighted and not selected
      if (isHighlighted && !isSelected) {
        tft.drawRoundRect(x - 4, y - 2, w + 8, h + 4, 6, HIGHLIGHT_COLOR);
      }

      // Center text in cell area
      tft.setTextSize(ROW_FONT_SIZE);
      int tw = textWidthForSize(display_text, ROW_FONT_SIZE);
      int tx = x + (w - tw) / 2;
      int ty = y + (h / 2) - 6;
      tft.setCursor(max(0, tx), max(0, ty));
      tft.print(display_text);
    }

    y += row_height;
  }
}

// -------------------- Utility: map encoder delta to actions --------------------
void applyEncoderDelta(long delta) {
  if (delta == 0) return;
  if (selectableCount == 0) return;

  if (selectedIndex == -1) {
    // Not editing a selected item: move highlight
    highlightIndex += (delta > 0 ? 1 : -1);
    if (highlightIndex < 0) highlightIndex = selectableCount - 1;
    if (highlightIndex >= selectableCount) highlightIndex = 0;

    // ensure visible window updates
    int hr = findRowForSelectableIndex(highlightIndex);
    if (hr >= 0) {
      if (hr < first_visible_row) first_visible_row = hr;
      if (hr >= first_visible_row + VISIBLE_ROWS) first_visible_row = hr - VISIBLE_ROWS + 1;
    } else {
      first_visible_row = 0;
    }

    drawMenuPage(current_page);
  } else {
    // editing: increment/decrement param for selectedIndex
    if (selectedIndex < 0 || selectedIndex >= (int)selectableList.size()) return;
    SelEntry entry = selectableList[selectedIndex];
    if (entry.isSubtitle) return;
    JsonObject cell = entry.cellObj;
    if (!(cell.containsKey("param") && cell["param"].as<bool>())) return;
    String key = cell.containsKey("paramKey") ? cell["paramKey"].as<String>() : cell["text"].as<String>();
    int cur = getParamValue(key);
    int minv = getParamMin(key);
    int maxv = getParamMax(key);
    int step = 1;
    cur += (delta > 0 ? step : -step);
    if (cur < minv) cur = minv;
    if (cur > maxv) cur = maxv;
    setParamValue(key, cur);
    String menu_title = current_page["title"].as<String>();
    String button_name = cell["text"].as<String>();
    Serial.printf("onRotaryMove(%ld, %s, %s)\n", delta, menu_title.c_str(), button_name.c_str());
    drawMenuPage(current_page);
  }
}

// -------------------- Button handling --------------------
void handleButtonPress() {
  unsigned long now = millis();
  if (now - lastBtnMillis < 180) return;  // debounce
  lastBtnMillis = now;

  if (selectableCount == 0) return;
  if (highlightIndex < 0 || highlightIndex >= (int)selectableList.size()) return;
  SelEntry entry = selectableList[highlightIndex];

  if (entry.isSubtitle) {
    JsonObject sub = entry.cellObj;
    if (sub.containsKey("action") && String(sub["action"].as<const char *>()) == "back") {
      if (!page_stack.empty()) {
        current_page = page_stack.back();
        page_stack.pop_back();
        rebuildSelectableList();
        selectedIndex = -1;
        highlightIndex = 0;
        first_visible_row = 0;
        drawMenuPage(current_page);
      }
    }
    return;
  }

  JsonObject cell = entry.cellObj;

  // If the cell has a "target" (root rows created with target), open that page
  if (cell.containsKey("target")) {
    String target = cell["target"].as<String>();
    JsonObject root = doc["root"].as<JsonObject>();
    if (root.containsKey(target)) {
      JsonObject targetObj = root[target].as<JsonObject>();
      if (targetObj.containsKey("screen")) {
        page_stack.push_back(current_page);
        current_page = targetObj["screen"].as<JsonObject>();
        rebuildSelectableList();
        selectedIndex = -1;
        highlightIndex = 0;
        first_visible_row = 0;
        drawMenuPage(current_page);
        return;
      }
    }
  }

  int selIdx = highlightIndex;
  if (selectedIndex == -1) {
    selectedIndex = selIdx;
    String menu_title = current_page["title"].as<String>();
    String button_name = cell["text"].as<String>();
    Serial.printf("onSelect(%s, %s)\n", menu_title.c_str(), button_name.c_str());
  } else if (selectedIndex == selIdx) {
    selectedIndex = -1;
  } else {
    selectedIndex = selIdx;
  }

  int hr = findRowForSelectableIndex(highlightIndex);
  if (hr >= 0) {
    if (hr < first_visible_row) first_visible_row = hr;
    if (hr >= first_visible_row + VISIBLE_ROWS) first_visible_row = hr - VISIBLE_ROWS + 1;
  } else {
    first_visible_row = 0;
  }

  drawMenuPage(current_page);
}

// -------------------- Initialize param store --------------------
void initParamStore() {
  // XL-430 safe defaults
  paramStore["arm1"] = { 1024, 0, 3072 };
  paramStore["arm1_min"] = { 0, 0, 3072 };
  paramStore["arm1_max"] = { 3072, 0, 3072 };

  paramStore["arm2"] = { 1024, 0, 3072 };
  paramStore["arm2_min"] = { 0, 0, 3072 };
  paramStore["arm2_max"] = { 3072, 0, 3072 };

  paramStore["wrist"] = { 1024, 0, 3072 };
  paramStore["wrist_min"] = { 0, 0, 3072 };
  paramStore["wrist_max"] = { 3072, 0, 3072 };

  paramStore["grip1"] = { 1024, 0, 3072 };
  paramStore["grip1_min"] = { 0, 0, 3072 };
  paramStore["grip1_max"] = { 3072, 0, 3072 };

  paramStore["grip2"] = { 1024, 0, 3072 };
  paramStore["grip2_min"] = { 0, 0, 3072 };
  paramStore["grip2_max"] = { 3072, 0, 3072 };

  paramStore["base"] = { 1024, 0, 3072 };
  paramStore["base_min"] = { 0, 0, 3072 };
  paramStore["base_max"] = { 3072, 0, 3072 };
}

// -------------------- Setup & Loop --------------------
void setup() {
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(BACKGROUND_COLOR);

  pinMode(ENC_BTN, INPUT_PULLUP);

  // parse JSON
  DeserializationError err = deserializeJson(doc, menu_json);
  if (err) {
    Serial.print("JSON parse error: ");
    Serial.println(err.c_str());
    while (1) delay(1000);
  }

  initParamStore();

  // Set current_page to the landing screen defined in JSON: doc["root"]["screen"]
  JsonObject root = doc["root"].as<JsonObject>();
  if (root.containsKey("screen")) {
    current_page = root["screen"].as<JsonObject>();
  } else {
    // fallback: try to find the first screen
    for (JsonPair kv : root) {
      if (strcmp(kv.key().c_str(), "screen") == 0) continue;
      JsonObject obj = kv.value().as<JsonObject>();
      if (obj.containsKey("screen")) {
        current_page = obj["screen"].as<JsonObject>();
        break;
      }
    }
  }

  // push initial page on stack as the landing page
  page_stack.clear();
  page_stack.push_back(current_page);

  // build selectable list
  rebuildSelectableList();

  // initial draw
  drawMenuPage(current_page);
}

// track previous encoder value and button state
long lastEnc = 0;
bool lastBtnState = HIGH;

void loop() {
  // read encoder (coarse)
  long enc = rotary.read() / encoderDiv;
  long delta = enc - lastEnc;
  if (delta != 0) {
    applyEncoderDelta(delta);
    lastEnc = enc;
  }

  // read button
  bool btn = digitalRead(ENC_BTN);
  if (btn == LOW && lastBtnState == HIGH) {  // pressed (active low)
    handleButtonPress();
  }
  lastBtnState = btn;

  // tiny delay to reduce CPU usage
  delay(10);
}
