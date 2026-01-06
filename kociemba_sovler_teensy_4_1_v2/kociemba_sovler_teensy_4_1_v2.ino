/**********************************************************************
* Arduino/Teensy 4.1 port of Kociemba's algorithm for solving a Rubik's 
* cube.
*
* The (beautiful) original algorithm was designed by H Kociemba:
* c.f. http://kociemba.org/ for details.
*
* This code is a straighforward hack from the c-code available at:
* https://github.com/muodov/kociemba
*
* Requirement: the algo. needs 4.3MB of Flash memory for storing the 
* precomputed tables but just a few KB of RAM. It can be speed up by 
* allocating more RAM to store some cache tables in fast memory. 
*
* Usage:
* - allocate more RAM with kociemba::set_memory()
* - solve a cube with kociemba::solve()
*********************************************************************/

/**********************************************************************
//  HARDWARE is Teensy 4.1
// under ports menu should be a "teensy ports" section and it should
// connect to a port there

to debug recompile with serial instead of serial1

// TODO change below for debugging to Serial instead of Serial1
#define __serial Serial
*********************************************************************/

#include <Arduino.h>
#include "kociemba.h"

#define __serial Serial1 // this is for running
//#define __serial Serial // this is for debugging

// Teensy-specific
elapsedMillis em;

// Big RAM buffers (Teensy 4.1)
DMAMEM uint8_t buf479[479 * 1024];
uint8_t buf248[248 * 1024];

// Input buffer
static char line_buf[128];
static size_t line_len = 0;

// 5 randomly scrambled cube.
/*
char test_cube[5][55] = {
  "UBRDUFUDLBUBFRDFRDFLURFRBFDDLRLDDFBLRBLULBLURUUFFBRBLD",
  "RDRBUFFUUFRFDRULRFDRLBFBBFBDUDFDDBBRURLDLULLRDLBLBFULU",
  "DRLUUBFBRBLURRLRUBLRDDFDLFUFUFFDBRDUBRUFLLFDDBFLUBLRBD",
  "DLFRUUURLUBDLRBLURBBFLFDFFUURBFDRBBDLDRDLFDURRUFDBFBLL",
  "BUFUUDFBLURRFRLRFBDRFUFRBBFRDDFDDBULLLLDLBRLUURDBBLUFD"
};
*/

static int count_moves(const char *s) {
  if (!s || !*s) return 0;

  int count = 1;  // at least one move if non-empty
  while (*s) {
    if (*s == ' ') count++;
    s++;
  }
  return count;
}

static void str_to_upper(char *s) {
  while (*s) {
    if (*s >= 'a' && *s <= 'z')
      *s -= 32;
    s++;
  }
}

static bool is_color_format(const char *s) {
  for (int i = 0; i < 54; i++) {
    char c = s[i];
    if (!(c == 'W' || c == 'Y' || c == 'R' || c == 'O' || c == 'G' || c == 'B'))
      return false;
  }
  return true;
}

static bool colors_to_faces(const char *in, char *out) {
  for (int i = 0; i < 54; i++) {
    switch (in[i]) {
      case 'W': out[i] = 'U'; break;
      case 'Y': out[i] = 'D'; break;
      case 'R': out[i] = 'R'; break;
      case 'O': out[i] = 'L'; break;
      case 'G': out[i] = 'F'; break;
      case 'B': out[i] = 'B'; break;
      default: return false;
    }
  }
  out[54] = '\0';
  return true;
}

void print_help() {
  __serial.println("HELP teensy 4.1 solver v2");
  __serial.println("version=teensy_4_1_v2");
  __serial.println("COMMANDS:");
  __serial.println("FINDSOLUTION cube=<54 chars>");
  __serial.println("format cube=URFDLB... or cube=WYROGB...");
  __serial.println("examples");
  __serial.println("FINDSOLUTION cube=BUFUUDFBLURRFRLRFBDRFUFRBBFRDDFDDBULLLLDLBRLUURDBBLUFD");
  __serial.println("FINDSOLUTION cube=BWGWWYGBOWRRGRORGBYRGWGRBBGRYYGYYBWOOOOYOBROWWRYBBOWGY");
  __serial.println();
}

void setup() {
  __serial.begin(115200);
  while (!__serial) {
    delay(5);
  }
  __serial.printf("setup start\n");
  em = 0;
  kociemba::set_memory(buf479, buf248);

  __serial.printf("READY ram_init_ms=%d\n", (int)em);
  __serial.println();
  print_help();
  __serial.printf("setup end\n");
  __serial.println();
}

/**
 * Parse:
 *   FINDSOLUTION cube=XXXXXXXX...
 */
void handle_command(char *line) {

  str_to_upper(line);

  // HELP command
  if (strcmp(line, "HELP") == 0) {
    print_help();
    return;
  }

  // FINDSOLUTION
  if (strncmp(line, "FINDSOLUTION", 12) != 0) {
    __serial.println("ERR error=unknown_command");
    print_help();
    return;
  }

  char *cube = strstr(line, "CUBE=");
  if (!cube) {
    __serial.println("ERR error=missing_cube");
    __serial.println();
    return;
  }

  cube += 5;  // skip "CUBE="

  if (strlen(cube) != 54) {
    __serial.println("ERR error=cube_invalid_length");
    __serial.println();
    return;
  }

  char cube_faces[55];

  if (is_color_format(cube)) {
    if (!colors_to_faces(cube, cube_faces)) {
      __serial.println("ERR error=color_parse_failed");
      __serial.println();
      return;
    }
  } else {
    memcpy(cube_faces, cube, 54);
    cube_faces[54] = '\0';
  }

  __serial.println("starting solve...");
  em = 0;
  const char *res = kociemba::solve(cube_faces);
  int ms = (int)em;
  __serial.println("solve done");
  if (!res) {
    __serial.println("SOLUTION result=not_found");
    __serial.println();
  } else {
    int movecount = count_moves(res);
    __serial.printf(
      "SOLUTION result=found solution=%s move_count=%d time_ms=%d\n",
      res,
      movecount,
      ms);
    __serial.println();
  }
}

void loop() {
  while (__serial.available()) {
    char c = __serial.read();

    // line termination
    if (c == '\n' || c == '\r') {
      if (line_len > 0) {
        line_buf[line_len] = '\0';
        handle_command(line_buf);
        line_len = 0;
      }
    } else {
      if (line_len < sizeof(line_buf) - 1) {
        line_buf[line_len++] = c;
      }
    }
  }
}

/* end of file */
