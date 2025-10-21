/********************************************************************
 * RUN SOLVE SEQUENCE WITH PROGRESS BAR + COLOR DETECTION
 ********************************************************************/

// ----- Main orchestrator -----
void runSolveCube() {
  solvingActive = true;
  Serial.println("\n=== Starting cube solve sequence ===");

  unsigned long startSolve = millis();
  unsigned long lastProgressRedraw = 0;

  std::fill(moveStates.begin(), moveStates.end(), STATE_PENDING);
  if (!solveMoves.empty()) moveStates[0] = STATE_ACTIVE;
  currentMove = 0;

  drawProgressBarGeneric(tft, 260, solveMoves, currentMove, moveStates);
  drawSolveStats(tft, 230, 0, solveMoves.size(), 0);

  while (solvingActive && currentMove < (int)solveMoves.size()) {
    String currentMoveName = solveMoves[currentMove];
    Serial.printf("\n▶ Move %2d/%d: %s\n",
                  currentMove + 1, (int)solveMoves.size(),
                  currentMoveName.c_str());

    String hardwareMove = convertStandardMove(currentMoveName);
    bool ok = runCubeStandardMove(hardwareMove);

    cubeColors.applyMove(currentMoveName);
    cubeColors.drawToTFT(String(toupper(currentMoveName[0])) + "4");

    moveStates[currentMove] = ok ? STATE_DONE_OK : STATE_ERROR;
    Serial.printf("   -> %s (%s)\n", currentMoveName.c_str(), ok ? "OK" : "FAILED");

    unsigned long startWait = millis();
    while (millis() - startWait < MOVE_PAUSE_MS) {
      if (digitalRead(ENC_BTN) == LOW) {
        Serial.println("❌ Solve cancelled by user.");
        solvingActive = false;
        break;
      }
      delay(10);
    }

    if (!solvingActive) break;
    currentMove++;
    if (currentMove < (int)moveStates.size())
      moveStates[currentMove] = STATE_ACTIVE;

    unsigned long now = millis();
    if (now - lastProgressRedraw > 200) {
      drawProgressBarGeneric(tft, 260, solveMoves, currentMove, moveStates);
      drawSolveStats(tft, 230, currentMove, solveMoves.size(), now - startSolve);
      lastProgressRedraw = now;
    }
  }

  unsigned long totalElapsed = millis() - startSolve;
  if (solvingActive)
    Serial.printf("✅ Cube solve completed in %.2fs\n", totalElapsed / 1000.0f);
  else
    Serial.printf("⏹️ Solve aborted at move %d after %.2fs\n",
                  currentMove, totalElapsed / 1000.0f);

  solvingActive = false;
  drawProgressBarGeneric(tft, 260, solveMoves, currentMove, moveStates);
  drawSolveStats(tft, 230, currentMove, solveMoves.size(), totalElapsed);
}

// =============================================================
// ---------------- READ MODE UI -------------------------------
// Example for color scanning bands

void drawReadProgress(const std::vector<String> &bands,
                      const std::vector<uint8_t> &states,
                      int currentBand) {
  drawProgressBarGeneric(tft, 260, bands, currentBand, states, true);
}
