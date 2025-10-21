// Rubik solver — ESP32 + Dynamixel XL430 + 480x320 TFT
// Path-based menus, preset editing (live preview), preset moves with profile velocity,
// Preferences storage. Uses Dynamixel2Arduino (no GroupSyncWrite).

#include <Adafruit_GFX.h>
#include <Adafruit_ST7796S_kbv.h>
#include <Preferences.h>
#include <Dynamixel2Arduino.h>

// ---------------- HARDWARE CONFIG ----------------
#define TFT_CS 15
#define TFT_DC 2
#define TFT_RST -1  // -1 if not used
Adafruit_ST7796S_kbv tft(TFT_CS, TFT_DC, TFT_RST);

#define SCREEN_W 480
#define SCREEN_H 320

// Dynamixel serial (Serial1/Serial2 depending on board)
#define DXL_SERIAL Serial1
#define DXL_DIR_PIN 26  // half-duplex direction pin for TTL adapter; change for your wiring
#define DXL_BAUD 57600
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// Servo IDs (assumed 1..6, change if needed)
const uint8_t SERVO_IDS[6] = { 1, 2, 3, 4, 5, 6 };
const char* SERVO_NAMES[6] = { "S1", "S2", "S3", "S4", "S5", "S6" };

// Profile velocities (ticks/sec)
const uint32_t PROFILE_VELOCITY_PREVIEW = 150;
const uint32_t PROFILE_VELOCITY_MOVE = 400;

// Preferences namespace
Preferences prefs;

// ---------------- MENU PATHS ----------------
// All menu leaf paths and grouping via path strings
const char* menu_paths[] = {
  "/root/Solve cube",
  "/root/Read colors",
  "/root/Tests/Servo limits",
  "/root/Tests/Servo preset positions/Z0",
  "/root/Tests/Servo preset positions/Z1",
  "/root/Tests/Servo preset positions/Z2",
  "/root/Tests/Servo preset positions/GO",
  "/root/Tests/Servo preset positions/GC",
  "/root/Tests/Servo preset positions/X0",
  "/root/Tests/Servo preset positions/X-90",
  "/root/Tests/Servo preset positions/X+90",
  "/root/Tests/Servo preset positions/X+180",
  "/root/Tests/Servo preset positions/B0",
  "/root/Tests/Servo preset positions/B+90",
  "/root/Tests/Servo preset positions/B-90",
  "/root/Tests/Servo preset positions/B+180",
  "/root/Tests/Servo preset moves/Z0-Z1",
  "/root/Tests/Servo preset moves/Z1-Z2",
  "/root/Tests/Servo preset moves/Z2-Z1",
  "/root/Tests/Servo preset moves/Z1-Z0",
  "/root/Tests/Servo preset moves/GO-GC",
  "/root/Tests/Servo preset moves/GC-GO",
  "/root/Tests/Cube standard moves"
};
const int menu_count = sizeof(menu_paths) / sizeof(menu_paths[0]);

// ---------------- UI & STATE ----------------
String current_path = "/root";
int selected_index = 0;  // index within the visible list for current path
int scroll_offset = 0;
const int visible_rows = 6;
const int MAX_CHILDREN = 32;
const char* children_buf[MAX_CHILDREN];

// Editor mode state
bool in_preset_editor = false;
String editing_preset = "";
long editing_values[6] = { 0, 0, 0, 0, 0, 0 };  // local values in editor
int editor_sel = 0;                             // 0=Back,1=Save,2..7=servo slots
bool editor_edit_mode = false;                  // true when editing a single servo number

// Simple serial-based input for testing
// w = up, s = down, e = enter/click
// (You can replace this with actual encoder ISR code.)
// --------------------------------------------------

// ---------------- Dynamixel helpers ----------------

// Set profile velocity for servo id
void setProfileVelocityForAll(uint32_t velocity) {
  // for (int i = 0; i < 6; ++i) {
  //   dxl.setProfileVelocity(SERVO_IDS[i], velocity);
  // }

  for (int i = 0; i < 6; i++) {
    dxl.torqueOff(SERVO_IDS[i]);
    dxl.setOperatingMode(SERVO_IDS[i], OP_POSITION);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, SERVO_IDS[i], velocity);      // Set velocity
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, SERVO_IDS[i], 20);  // Set acceleration TODO
    dxl.torqueOn(SERVO_IDS[i]);
  }
}

// Set all servos to given targets quickly (no sync-write; fast loop)
void applyTargetsQuick(long targets[6]) {
  // write each servo goal quickly
  for (int i = 0; i < 6; ++i) {
    dxl.setGoalPosition(SERVO_IDS[i], targets[i]);
  }
}

// Blocking move: set velocity then set goals; wait until they reach within tolerance
void goToPresetByName(const String& presetName, uint32_t velocity = PROFILE_VELOCITY_MOVE) {
  long targets[6];
  // load from prefs, fallback to 0
  for (int i = 0; i < 6; i++) {
    String key = String("preset_") + presetName + "_s" + String(i + 1);
    targets[i] = prefs.getLong(key.c_str(), 0);
  }

  // set velocity
  setProfileVelocityForAll(velocity);

  // write goals quickly
  applyTargetsQuick(targets);

  // wait until done (blocking) with tolerance
  bool moving = true;
  while (moving) {
    moving = false;
    for (int i = 0; i < 6; i++) {
      long pos = dxl.getPresentPosition(SERVO_IDS[i]);
      if (abs(pos - targets[i]) > 3) {
        moving = true;
        break;
      }
    }
    delay(10);
  }
}

// Non-blocking preview: set velocity and goals, but do not wait
void previewPresetByName(const String& presetName, uint32_t velocity = PROFILE_VELOCITY_PREVIEW) {
  long targets[6];
  for (int i = 0; i < 6; i++) {
    String key = String("preset_") + presetName + "_s" + String(i + 1);
    targets[i] = prefs.getLong(key.c_str(), 0);
  }
  setProfileVelocityForAll(velocity);
  applyTargetsQuick(targets);
}

// ---------------- Menu utilities ----------------
bool startsWith(const char* s, const char* p) {
  return strncmp(s, p, strlen(p)) == 0;
}

// Get immediate children (folders and leafs) of prefix.
// children[] will be filled with pointers to strings (either original menu_paths entries for leaves
// or strdup()'d strings for folders). Caller does not free here.
int get_children(const char* prefix, const char** children, int max_children) {
  int count = 0;
  int plen = strlen(prefix);

  for (int i = 0; i < menu_count; ++i) {
    if (!startsWith(menu_paths[i], prefix)) continue;
    if (strcmp(menu_paths[i], prefix) == 0) continue;
    const char* rest = menu_paths[i] + plen;
    if (rest[0] == '/') rest++;
    const char* s = strchr(rest, '/');
    if (!s) {
      // leaf child: the full path is a leaf; add the full string
      if (count < max_children) children[count++] = menu_paths[i];
    } else {
      // folder: create prefix + token
      int token_len = s - rest;
      String child = String(prefix);
      if (!child.endsWith("/")) child += "/";
      child += String(rest).substring(0, token_len);
      bool found = false;
      for (int j = 0; j < count; j++)
        if (child == String(children[j])) {
          found = true;
          break;
        }
      if (!found && count < max_children) children[count++] = strdup(child.c_str());
    }
    if (count >= max_children) break;
  }
  return count;
}

String leafOf(const char* path) {
  const char* s = strrchr(path, '/');
  if (!s) return String(path);
  return String(s + 1);
}

// ---------------- Drawing helpers ----------------
void draw_title_center(const char* title) {
  tft.setTextSize(3);
  tft.setTextColor(0xFFFF);
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  int cx = max(0, (SCREEN_W - (int)w) / 2);
  tft.setCursor(cx, 8);
  tft.print(title);
}

void draw_menu_list(const char* title, const char** items, int item_count, int sel_index, int scroll_off) {
  tft.fillScreen(0x0000);  // black
  draw_title_center(title);
  tft.setTextSize(2);

  int y = 48;
  int boxW = SCREEN_W - 40;
  int boxH = 36;
  int display = min(visible_rows, max(0, item_count - scroll_off));

  for (int i = 0; i < display; ++i) {
    int idx = scroll_off + i;
    bool sel = (idx == sel_index);
    const char* full = items[idx];
    String name = (String(full) == "BACK") ? "..." : leafOf(full);
    int x = 20;
    if (sel) {
      tft.drawRoundRect(x, y, boxW, boxH, 6, 0xFFE0);  // highlight yellow
      tft.setTextColor(0xFFE0, 0x0000);
    } else {
      tft.setTextColor(0xFFFF, 0x0000);
    }
    tft.setCursor(x + 8, y + 7);
    tft.print(name);
    y += boxH + 8;
  }

  // page indicator
  int total_pages = (item_count + visible_rows - 1) / visible_rows;
  int cur_page = (scroll_off / visible_rows) + 1;
  char buf[16];
  sprintf(buf, "%d/%d", cur_page, total_pages);
  tft.setTextSize(2);
  tft.setTextColor(0xFFFF);
  tft.setCursor(SCREEN_W - 70, SCREEN_H - 28);
  tft.print(buf);
}

void draw_preset_editor(const String& presetName, int sel_idx, bool edit_mode) {
  tft.fillScreen(0x0000);
  draw_title_center(("Preset: " + presetName).c_str());
  tft.setTextSize(2);

  // Buttons
  auto drawButton = [&](int x, int y, int w, int h, const char* label, bool selected, bool filled) {
    if (filled) {
      tft.fillRoundRect(x, y, w, h, 6, 0x001F);  // blue filled
      tft.setTextColor(0xFFFF, 0x001F);
    } else if (selected) {
      tft.drawRoundRect(x, y, w, h, 6, 0x07E0);  // green border
      tft.setTextColor(0x07E0, 0x0000);
    } else {
      tft.setTextColor(0xFFFF, 0x0000);
    }
    tft.setCursor(x + 10, y + 6);
    tft.print(label);
  };

  drawButton(20, 42, 110, 36, "Back", sel_idx == 0 && !edit_mode, false);
  drawButton(150, 42, 110, 36, "Save", sel_idx == 1 && !edit_mode, false);

  // Draw servo boxes (3 rows x 2 cols)
  int boxW = 200, boxH = 44;
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 2; c++) {
      int idx = r * 2 + c;  // 0..5
      int x = (c == 0) ? 20 : 260;
      int y = 100 + r * (boxH + 12);
      bool selected = (sel_idx == (2 + idx));
      bool editing = (edit_mode && selected);
      if (editing) {
        tft.fillRoundRect(x, y, boxW, boxH, 8, 0x001F);
        tft.setTextColor(0xFFFF, 0x001F);
      } else if (selected) {
        tft.drawRoundRect(x, y, boxW, boxH, 8, 0xFFE0);
        tft.setTextColor(0xFFE0, 0x0000);
      } else {
        tft.setTextColor(0xFFFF, 0x0000);
      }
      char buf[32];
      sprintf(buf, "%s %04ld", SERVO_NAMES[idx], editing_values[idx]);
      tft.setCursor(x + 10, y + 12);
      tft.print(buf);
    }
  }

  // footer page indicator
  tft.setTextSize(2);
  tft.setTextColor(0xFFFF);
  tft.setCursor(SCREEN_W - 70, SCREEN_H - 28);
  tft.print("1/1");
}

// ---------------- Preset save/load helpers ----------------
void savePresetToPrefs(const String& presetName, long vals[6]) {
  for (int i = 0; i < 6; i++) {
    String key = String("preset_") + presetName + "_s" + String(i + 1);
    prefs.putLong(key.c_str(), vals[i]);
  }
}

void loadPresetFromPrefs(const String& presetName, long vals[6]) {
  for (int i = 0; i < 6; i++) {
    String key = String("preset_") + presetName + "_s" + String(i + 1);
    vals[i] = prefs.getLong(key.c_str(), 0);
  }
}

// ---------------- Path helpers ----------------
bool is_preset_position_path(const char* fullpath, String& presetName) {
  const char* p = strstr(fullpath, "/Servo preset positions/");
  if (!p) return false;
  p += strlen("/Servo preset positions/");
  presetName = String(p);
  return true;
}
bool is_preset_move_path(const char* fullpath, String& fromPreset, String& toPreset) {
  const char* p = strstr(fullpath, "/Servo preset moves/");
  if (!p) return false;
  p += strlen("/Servo preset moves/");
  String token = String(p);
  int dash = token.indexOf('-');
  if (dash < 0) return false;
  fromPreset = token.substring(0, dash);
  toPreset = token.substring(dash + 1);
  return true;
}

// ---------------- Menu handling ----------------
void show_current_menu() {
  int count = get_children(current_path.c_str(), children_buf, MAX_CHILDREN);
  bool has_back = current_path != String("/root");
  int total_items = count + (has_back ? 1 : 0);

  if (selected_index < 0) selected_index = 0;
  if (selected_index >= total_items) selected_index = total_items - 1;
  if (selected_index < scroll_offset) scroll_offset = selected_index;
  if (selected_index >= scroll_offset + visible_rows) scroll_offset = selected_index - visible_rows + 1;

  static const char* display_items[64];
  int di = 0;
  if (has_back) display_items[di++] = "BACK";
  for (int i = 0; i < count; i++) display_items[di++] = children_buf[i];

  draw_menu_list(current_path.c_str(), display_items, di, selected_index, scroll_offset);
}

// Called when user pushes the encoder/button to select
void menu_select() {
  int count = get_children(current_path.c_str(), children_buf, MAX_CHILDREN);
  bool has_back = current_path != String("/root");
  int total_items = count + (has_back ? 1 : 0);

  if (has_back && selected_index == 0) {
    // go up one level
    int pos = current_path.lastIndexOf('/');
    if (pos > 0) current_path = current_path.substring(0, pos);
    else current_path = "/root";
    selected_index = 0;
    scroll_offset = 0;
    show_current_menu();
    return;
  }

  const char* sel;
  if (has_back) sel = children_buf[selected_index - 1];
  else sel = children_buf[selected_index];

  // if folder -> go deeper
  int child_count = get_children(sel, children_buf, MAX_CHILDREN);
  if (child_count > 0) {
    current_path = String(sel);
    selected_index = 0;
    scroll_offset = 0;
    show_current_menu();
    return;
  }

  // leaf path action
  String presetName, fromP, toP;
  if (is_preset_position_path(sel, presetName)) {
    // enter preset editor
    in_preset_editor = true;
    editing_preset = presetName;
    loadPresetFromPrefs(editing_preset, editing_values);
    editor_sel = 0;
    editor_edit_mode = false;
    draw_preset_editor(editing_preset, editor_sel, editor_edit_mode);
    return;
  } else if (is_preset_move_path(sel, fromP, toP)) {
    // preview then go
    previewPresetByName(fromP, PROFILE_VELOCITY_PREVIEW);
    delay(200);
    goToPresetByName(toP, PROFILE_VELOCITY_MOVE);
    show_current_menu();
    return;
  } else {
    // other actions
    String leaf = leafOf(sel);
    if (leaf == "Solve cube") {
      tft.fillScreen(0x0000);
      draw_title_center("Solve cube");
      tft.setTextSize(2);
      tft.setTextColor(0xFFFF);
      tft.setCursor(10, 60);
      tft.print("Solver stub - implement later");
      delay(700);
    } else if (leaf == "Read colors") {
      tft.fillScreen(0x0000);
      draw_title_center("Read colors");
      tft.setTextSize(2);
      tft.setTextColor(0xFFFF);
      tft.setCursor(10, 60);
      tft.print("Color reader stub - implement later");
      delay(700);
    } else {
      tft.fillScreen(0x0000);
      draw_title_center(leaf.c_str());
      tft.setTextSize(2);
      tft.setTextColor(0xFFFF);
      tft.setCursor(10, 60);
      tft.print("Not implemented");
      delay(700);
    }
    show_current_menu();
    return;
  }
}

// ---------------- Editor handlers ----------------
void editor_draw() {
  draw_preset_editor(editing_preset, editor_sel, editor_edit_mode);
}

// When rotary moves in editor (delta can be ±1 steps per input)
void editor_rotate(int delta) {
  if (!editor_edit_mode) {
    // move selection
    editor_sel += (delta > 0) ? 1 : -1;
    if (editor_sel < 0) editor_sel = 0;
    if (editor_sel > 7) editor_sel = 7;
    editor_draw();
    return;
  }

  // editing a servo numeric value
  int idx = editor_sel - 2;
  if (idx < 0 || idx >= 6) return;
  // step size logic: single-step 1 for small changes, 10 for larger delta
  int step = (abs(delta) > 4) ? 10 : 1;
  long v = editing_values[idx];
  v += (delta > 0) ? step : -step;
  if (v < 0) v = 0;
  if (v > 4095) v = 4095;
  editing_values[idx] = v;
  // live preview: set profile a gentle preview velocity and write goals
  setProfileVelocityForAll(PROFILE_VELOCITY_PREVIEW);
  applyTargetsQuick(editing_values);
  editor_draw();
}

void editor_click() {
  if (!editor_edit_mode) {
    // selection action
    if (editor_sel == 0) {
      // Back -> leave editor and return to menu path
      in_preset_editor = false;
      editing_preset = "";
      show_current_menu();
      return;
    } else if (editor_sel == 1) {
      // Save: persist editing_values to prefs and apply them
      savePresetToPrefs(editing_preset, editing_values);
      // apply with move velocity
      goToPresetByName(editing_preset, PROFILE_VELOCITY_MOVE);
      // exit editor
      in_preset_editor = false;
      editing_preset = "";
      show_current_menu();
      return;
    } else {
      // enter edit mode on selected servo
      editor_edit_mode = true;
      editor_draw();
      return;
    }
  } else {
    // exit edit mode and keep the live value (already written)
    editor_edit_mode = false;
    editor_draw();
    return;
  }
}

// ---------------- Serial-driven input (simple for testing) ----------------
void onUp() {
  if (in_preset_editor) editor_rotate(1);
  else {
    selected_index--;
    // clamp and page handling
    int count = get_children(current_path.c_str(), children_buf, MAX_CHILDREN);
    bool has_back = current_path != String("/root");
    int total_items = count + (has_back ? 1 : 0);
    if (selected_index < 0) selected_index = 0;
    if (selected_index >= total_items) selected_index = total_items - 1;
    // scroll offset
    if (selected_index < scroll_offset) scroll_offset = selected_index;
    show_current_menu();

    // preview: if selected is a preset leaf, preview it
    const char* sel = nullptr;
    if (has_back && selected_index == 0) sel = nullptr;
    else {
      if (has_back) sel = children_buf[selected_index - 1];
      else sel = children_buf[selected_index];
    }
    if (sel) {
      String pname;
      if (is_preset_position_path(sel, pname)) previewPresetByName(pname, PROFILE_VELOCITY_PREVIEW);
    }
  }
}

void onDown() {
  if (in_preset_editor) editor_rotate(-1);
  else {
    selected_index++;
    int count = get_children(current_path.c_str(), children_buf, MAX_CHILDREN);
    bool has_back = current_path != String("/root");
    int total_items = count + (has_back ? 1 : 0);
    if (selected_index < 0) selected_index = 0;
    if (selected_index >= total_items) selected_index = total_items - 1;
    if (selected_index >= scroll_offset + visible_rows) scroll_offset = selected_index - visible_rows + 1;
    show_current_menu();

    // preview
    const char* sel = nullptr;
    if (has_back && selected_index == 0) sel = nullptr;
    else {
      if (has_back) sel = children_buf[selected_index - 1];
      else sel = children_buf[selected_index];
    }
    if (sel) {
      String pname;
      if (is_preset_position_path(sel, pname)) previewPresetByName(pname, PROFILE_VELOCITY_PREVIEW);
    }
  }
}

void onEnter() {
  if (in_preset_editor) editor_click();
  else menu_select();
}

// ---------------- Setup & main loop ----------------
void setupPinsPlaceholder() {
  // TODO: wire actual rotary encoder here (attach interrupts)
  // This sketch uses simple serial keyboard for testing.
}

void setup() {
  Serial.begin(115200);

  // Dynamixel
  DXL_SERIAL.begin(DXL_BAUD);
  dxl.begin();
  dxl.setPortProtocolVersion(2.0);
  for (int i = 0; i < 6; i++) {
    dxl.torqueOn(SERVO_IDS[i]);
    dxl.setOperatingMode(SERVO_IDS[i], OP_POSITION);
  }

  // Preferences
  prefs.begin("rubik", false);

  // TFT
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(0x0000);

  // initial menu
  current_path = "/root";
  selected_index = 0;
  scroll_offset = 0;
  show_current_menu();

  // placeholder for encoder pins
  setupPinsPlaceholder();
}

long lastSerialPos = 0;
void loop() {
  // simple serial control: 'w' up, 's' down, 'e' enter
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') onUp();
    else if (c == 's') onDown();
    else if (c == 'e') onEnter();
    // quick test: 'p name' to preview preset: e.g. pZ0  (not necessary)
  }

  // small sleep
  delay(10);
}
