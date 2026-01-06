#include <arduino.h>
#include "rubik_solver.h"
#include "logging.h"

bool is_valid_color_string(const String &str54) {

  if (str54.length() != 54) return false;

  int counts[6] = { 0 };  // W Y G B R O

  auto color_index = [](char c) -> int {
    switch (c) {
      case 'W': return 0;
      case 'Y': return 1;
      case 'G': return 2;
      case 'B': return 3;
      case 'R': return 4;
      case 'O': return 5;
      default: return -1;
    }
  };

  for (int i = 0; i < 54; i++) {
    char c = toupper(str54[i]);
    int idx = color_index(c);
    if (idx < 0) return false;  // invalid character
    counts[idx]++;
  }

  for (int i = 0; i < 6; i++) {
    if (counts[i] != 9) return false;
  }

  return true;
}

String getColorsForSolution(const String &solution) {

  auto face_to_color = [](char face) -> char {
    switch (face) {
      case 'U': return 'W';
      case 'D': return 'Y';
      case 'F': return 'G';
      case 'B': return 'B';
      case 'L': return 'R';
      case 'R': return 'O';
      default: return '?';
    }
  };

  String colors;
  int i = 0;

  while (i < solution.length()) {

    // Skip whitespace
    if (solution[i] == ' ') {
      i++;
      continue;
    }

    // Face letter
    char face = toupper(solution[i++]);
    char color = face_to_color(face);

    // Default is 1 turn
    int turns = 1;

    if (i < solution.length()) {
      if (solution[i] == '2') {
        turns = 2;
        i++;
      } else if (solution[i] == '\'' || solution[i] == '-') {
        turns = 1;  // still one atomic move
        i++;
      } else if (solution[i] == '+') {
        turns = 1;
        i++;
      }
    }

    // Append color once per atomic turn
    for (int t = 0; t < turns; t++) {
      colors += color;
    }
  }

  return colors;
}

/*
  Canonical orientation (Option A, enforced externally):

    U = White   (center index 4)
    D = Yellow  (center index 31)
    F = Green   (center index 22)
    L = Red     (center index 40)
    R = Orange  (center index 13)
    B = Blue    (center index 49)

  Cube string order (URFDLB):
    U:  0–8
    R:  9–17
    F: 18–26
    D: 27–35
    L: 36–44
    B: 45–53
*/

// ============================================================
// 1) Check top two layers solved
// ============================================================

bool is_solved_top_two_layers(const String &a_cube) {

  String cube = a_cube;
  cube.toUpperCase();

  // U face must be all white
  for (int i = 0; i < 9; i++) {
    if (cube[i] != 'W') return false;
  }

  // Side faces: top two rows must match center
  const int faces[4][6] = {
    { 18, 19, 20, 21, 22, 23 },  // F
    { 9, 10, 11, 12, 13, 14 },   // R
    { 45, 46, 47, 48, 49, 50 },  // B
    { 36, 37, 38, 39, 40, 41 }   // L
  };

  const int centers[4] = { 22, 13, 49, 40 };

  for (int f = 0; f < 4; f++) {
    char c = cube[centers[f]];
    for (int i = 0; i < 6; i++) {
      if (cube[faces[f][i]] != c) return false;
    }
  }

  return true;
}

// ============================================================
// 2) OLL (orient yellow face, D)
// ============================================================

struct oll_case_t {
  uint16_t mask;  // 9-bit mask of yellow on D
  const char *algo;
};

// D face indices (27–35):
// 0 1 2
// 3 4 5
// 6 7 8

static const oll_case_t oll_table[] = {
  { 0b000010000, "F R U R' U' F' " },            // Dot
  { 0b000111000, "F R U R' U' F' " },            // Line
  { 0b000011000, "F U R U' R' F' " },            // L-shape
  { 0b100010000, "R U R' U R U2 R' " },          // Sune
  { 0b001010000, "R U2 R' U' R U' R' " },        // Anti-Sune
  { 0b101010000, "R2 D R' U2 R D' R' U2 R' " },  // Headlights
  { 0b111111111, "" }                            // Already oriented
};

// ============================================================
// 3) PLL (permute last layer, face-only)
// ============================================================

struct pll_case_t {
  bool (*match)(const String &);
  const char *algo;
};

// Edge indices (bottom layer)
#define DF 25
#define DR 16
#define DB 52
#define DL 43

// Edge solved
static bool pll_edges_solved(const String &c) {
  return c[DF] == c[22] && c[DR] == c[13] && c[DB] == c[49] && c[DL] == c[40];
}

// Ua
static bool pll_ua(const String &c) {
  return c[DF] == c[13] && c[DR] == c[49] && c[DB] == c[40] && c[DL] == c[22];
}

// Ub
static bool pll_ub(const String &c) {
  return c[DF] == c[40] && c[DL] == c[49] && c[DB] == c[13] && c[DR] == c[22];
}

// H
static bool pll_h(const String &c) {
  return c[DF] == c[49] && c[DB] == c[22] && c[DR] == c[40] && c[DL] == c[13];
}

// Z
static bool pll_z(const String &c) {
  return c[DF] == c[40] && c[DL] == c[22] && c[DR] == c[49] && c[DB] == c[13];
}

// Face-only PLL table (NO M, NO rotations)
static const pll_case_t pll_table[] = {
  { pll_edges_solved, "" },
  { pll_ua, "R U' R U R U R U' R' U' R2 " },
  { pll_ub, "R2 U R U R' U' R' U' R' U R' " },
  { pll_h, "R2 L2 D2 R2 L2 U R2 L2 D2 R2 L2 " },
  { pll_z, "R U R' U R' U' R' U R U' R2 U' R' U R' " }
};

// ============================================================
// 4) AUF (align D so front edge matches front center)
// ============================================================

String align_bottom_layer(const String &cube) {
  if (cube[25] == cube[22]) return "";
  if (cube[34] == cube[22]) return "D ";
  if (cube[31] == cube[22]) return "D2 ";
  if (cube[28] == cube[22]) return "D' ";
  return "";
}

// ============================================================
// 5) Full bottom-layer solver
// ============================================================

String find_solution_for_bottom_layer(const String &a_cube) {

  String cube = a_cube;
  cube.toUpperCase();

  if (!is_solved_top_two_layers(cube)) {
    LOG_ERR("[SOLVER] top two layers not solved\n");
    return "";
  }

  String result;

  // ---------------- OLL ----------------
  uint16_t mask = 0;
  for (int i = 0; i < 9; i++) {
    if (cube[27 + i] == 'Y') mask |= (1 << i);
  }

  for (auto &o : oll_table) {
    if (o.mask == mask) {
      result += o.algo;
      break;
    }
  }

  // ---------------- AUF ----------------
  result += align_bottom_layer(cube);

  // ---------------- PLL ----------------
  for (auto &p : pll_table) {
    if (p.match(cube)) {
      result += p.algo;
      break;
    }
  }

  // ---------------- Final AUF ----------------
  result += align_bottom_layer(cube);

  return result;
}

String compress_moves(const String &moves) {

  String out;
  char last_face = 0;
  int last_amount = 0;  // +1, +2, +3 (mod 4)

  auto flush = [&]() {
    if (last_face == 0 || last_amount == 0) return;

    int a = last_amount % 4;
    if (a == 1) out += String(last_face) + " ";
    else if (a == 2) out += String(last_face) + "2 ";
    else if (a == 3) out += String(last_face) + "' ";

    last_face = 0;
    last_amount = 0;
  };

  int i = 0;
  while (i < moves.length()) {

    // Skip spaces
    if (moves[i] == ' ') {
      i++;
      continue;
    }

    char face = moves[i++];
    int amount = 1;

    if (i < moves.length()) {
      if (moves[i] == '2') {
        amount = 2;
        i++;
      } else if (moves[i] == '\'') {
        amount = 3;  // -1 mod 4
        i++;
      }
    }

    if (face == last_face) {
      last_amount += amount;
    } else {
      flush();
      last_face = face;
      last_amount = amount;
    }
  }

  flush();
  return out;
}

// ============================================================
// Serial interface with teensy solver
// ============================================================

static HardwareSerial *solver_serial = nullptr;
static uint32_t solver_timeout_ms = 3000;

static String last_solver_error;

// ============================================================
// Utilities
// ============================================================

static int count_moves_in_solution(const String &sol) {
  if (sol.isEmpty()) return 0;

  int count = 1;
  for (size_t i = 0; i < sol.length(); i++) {
    if (sol[i] == ' ') count++;
  }
  return count;
}

// ============================================================
// Begin
// ============================================================

bool solver_begin() {
  solver_serial = &SOLVER_SERIAL;
  solver_timeout_ms = 500;
  last_solver_error = "";

  solver_serial->begin(115200);
  solver_serial->setTimeout(timeout_ms);

  // flush startup noise
  delay(300);
  while (solver_serial->available())
    solver_serial->read();

  // probe with HELP
  solver_serial->println("HELP");

  unsigned long t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (solver_serial->available()) {
      String line = solver_serial->readStringUntil('\n');
      line.trim();
      if (line.startsWith("HELP")) {
        return true;
      }
    }
  }

  last_solver_error = "solver not responding";
  return false;
}

// ============================================================
// Version
// ============================================================

String solver_get_version() {
  if (!solver_serial) return "err";

  last_solver_error = "";

  solver_serial->println("HELP");

  unsigned long t0 = millis();
  while (millis() - t0 < solver_timeout_ms) {
    if (!solver_serial->available()) continue;

    String line = solver_serial->readStringUntil('\n');
    line.trim();

    // version=teensy_4_1_v2
    if (line.startsWith("version=")) {
      String v = line.substring(strlen("version="));
      v.replace("_", " ");
      return v;
    }
  }

  last_solver_error = "version timeout";
  LOG_ERR("[SOLVER] solver interface timeout\n")
  return "err";
}

// ============================================================
// Find solution
// ============================================================

bool solver_find_solution(const String &cube54,
                          String &out_solution,
                          int &out_move_count,
                          int *out_time_ms) {
  if (!solver_serial) {
    last_solver_error = "solver not initialized";
    return false;
  }

  last_solver_error = "";
  out_solution = "";
  out_move_count = 0;
  if (out_time_ms) *out_time_ms = 0;

  // Send command
  solver_serial->print("FINDSOLUTION cube=");
  solver_serial->println(cube54);

  unsigned long t0 = millis();

  while (millis() - t0 < solver_timeout_ms) {
    if (!solver_serial->available()) continue;

    String line = solver_serial->readStringUntil('\n');
    line.trim();

    // SOLUTION result=found solution=... move_count=22 time_ms=1109
    if (line.startsWith("SOLUTION")) {

      // --- result ---
      int r = line.indexOf("result=");
      if (r < 0) {
        last_solver_error = line;
        return false;
      }

      int r_end = line.indexOf(' ', r);
      String result = line.substring(r + 7,
                                     r_end < 0 ? line.length() : r_end);

      if (result != "found") {
        last_solver_error = line;
        LOG_ERR("[SOLVER] solver_error=%s\n", line.c_str());
        return false;
      }

      // --- solution ---
      int s = line.indexOf("solution=");
      if (s < 0) {
        last_solver_error = "solution missing";
        LOG_ERR("[SOLVER] solver_error=%s\n", line.c_str());
        return false;
      }

      int s_end = line.indexOf(" move_count=", s);
      String solution = line.substring(
        s + 9,
        s_end < 0 ? line.length() : s_end);

      solution.trim();
      out_solution = solution;

      // --- move_count ---
      int m = line.indexOf("move_count=");
      if (m < 0) {
        last_solver_error = "move_count missing";
        LOG_ERR("[SOLVER] solver_error=%s\n", line.c_str());
        return false;
      }

      int m_end = line.indexOf(' ', m);
      int move_count = line.substring(
                             m + 11,
                             m_end < 0 ? line.length() : m_end)
                         .toInt();

      // --- verify move count ---
      int computed = count_moves_in_solution(solution);
      if (computed != move_count) {
        last_solver_error =
          "move count mismatch (parsed=" + String(computed) + " reported=" + String(move_count) + ")";
        LOG_ERR("[SOLVER] solver_error=%s\n", line.c_str());
        return false;
      }

      out_move_count = move_count;

      // --- time_ms ---
      if (out_time_ms) {
        int t = line.indexOf("time_ms=");
        if (t >= 0) {
          *out_time_ms =
            line.substring(t + 8).toInt();
        }
      }

      return true;
    }

    // Any other line is considered an error
    if (!line.isEmpty()) {
      last_solver_error = line;
      LOG_ERR("[SOLVER] solver_error=%s\n", line.c_str());
    }
  }

  last_solver_error = "solver timeout";
  LOG_ERR("[SOLVER] solver_error=%s\n", line.c_str());
  return false;
}

// ============================================================
// Error
// ============================================================

String solver_get_last_error() {
  return last_solver_error;
}
