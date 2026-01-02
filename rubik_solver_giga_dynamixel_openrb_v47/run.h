#ifndef RUN_H
#define RUN_H

#include <Arduino.h>

// -----------------------------------------------------------------------------
// Robot / Cube / UI action dispatchers
// All functions:
//   - return false if the key is NOT handled
//   - set `result` to the command execution result if handled
// -----------------------------------------------------------------------------

void buttonAction_run(const char *btn_key);

bool runRobotMovesByKey(const char* key, bool& result);
bool runCubeMoveByKey(const char* key, bool& result);

// Orientation / RUN presets
bool runRunOrientationByKey(const char* key, bool& result);

// Color sticker read (ONECOLOR 1..6)
bool runColorStickerByKey(const char* key, bool& result);

// Solve / system / misc
bool runSolveCubeRunSolutionByKey(const char* key, bool& result);
bool runSolveCubeFindSolutionByKey(const char* key, bool& result);
bool runSystemByKey(const char* key, bool& result);

// Color reading / orientation helpers
bool runColorReadByKey(const char* key, bool& result);
bool runSolveReadByKey(const char* key, bool& result);
bool runColorOrientationByKey(const char* key, bool& result);



#endif  // RUN_H

