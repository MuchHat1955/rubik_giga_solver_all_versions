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

#include <Arduino.h>

#include "kociemba.h"
bool kociemba_init_tables();


// 5 randomly scrambled cube.
char test_cube[5][55] = {
  "UBRDUFUDLBUBFRDFRDFLURFRBFDDLRLDDFBLRBLULBLURUUFFBRBLD",
  "RDRBUFFUUFRFDRULRFDRLBFBBFBDUDFDDBBRURLDLULLRDLBLBFULU",
  "DRLUUBFBRBLURRLRUBLRDDFDLFUFUFFDBRDUBRUFLLFDDBFLUBLRBD",
  "DLFRUUURLUBDLRBLURBBFLFDFFUURBFDRBBDLDRDLFDURRUFDBFBLL",
  "BUFUUDFBLURRFRLRFBDRFUFRBBFRDDFDDBULLLLDLBRLUURDBBLUFD"
};


uint32_t em_start_ms = 0;

/*
char buf479[479*1024];   // 479K in DMAMEM
char  buf248[248 * 1024];       // 248K on in DTCM 
*/

uint8_t *buf479;
uint8_t *buf248;

void setup() {
  Serial.begin(115200);

  em_start_ms = millis();
  while (!Serial && millis() < em_start_ms + 11000) { delay(5); }
  em_start_ms = millis();
  Serial.println();
  Serial.println("Setup started!");

  if (!kociemba_init_tables()) {
    Serial.println("Kociemba table init failed! Stopping program!");
    while (1) { delay(5); }
  }

  buf479 = (uint8_t *)malloc(479 * 1024);
  buf248 = (uint8_t *)malloc(248 * 1024);

  if (!buf479 || !buf248) {
    Serial.println("Memory allocation failed! Stopping program!");
    while (1) { delay(5); }
  }

  kociemba::set_memory(buf479, buf248);
  kociemba::set_memory(buf479, buf248);  // removing this line slows the computation by a factor of 4 (but saves a lot of RAM...)

  Serial.print("ram buffer created in ");
  Serial.print(millis() - em_start_ms);
  Serial.println(" ms.");

  Serial.println("Setup end!");
  Serial.println();
}


void loop() {
  for (auto s : test_cube) {
    em_start_ms = millis();
    const char *res = kociemba::solve(s);
    if (res == nullptr)
      Serial.println("no solution found :(");
    else {
      Serial.print("Solution [");
      Serial.print(res);
      Serial.print("] found in ");
      Serial.print(millis() - em_start_ms);
      Serial.println(" ms.");
    }
  }
}

/* end of file */
