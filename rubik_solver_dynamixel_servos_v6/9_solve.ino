/********************************************************************
 * RUN THE MOVES FOR SOLVING
 ********************************************************************/

// ==================== READ FULL ROW OF 18 CELLS ====================
// Reads 3x3 visible cells per layer (left, center, right) * 6 cube faces
// Only reads the 18 edge/center cells in current layer orientation
void runSolve() {
}

void applyMove(const String &move) {
    // Basic mapping example (simplified)
    // You can expand this with proper 3x3 rotation logic
    // for F, R, U, etc. + directions (+/-/2)
    Serial.printf("Applying move: %s\n", move.c_str());
    // TODO: implement face rotation logic here.
}


std::vector<String> solveMoves = {
  "F", "R", "U", "R'", "U'", "F'", "L", "D", "L'", "U2",
  "B", "R2", "F2", "L'", "D'", "U", "R", "F", "L2", "B2", "U2"
};
std::vector<uint8_t> moveStates(solveMoves.size(), 0);  // 0=pending

int currentMove = 0;

void updateSolveUI() {
  drawProgressBar(tft, 260, solveMoves, currentMove, moveStates);
}

// Call this after each move:
void advanceSolveProgress(bool ok) {
  moveStates[currentMove] = ok ? 2 : 3;
  currentMove++;
  if (currentMove < (int)moveStates.size())
    moveStates[currentMove] = 1;  // mark as active
  updateSolveUI();
}

