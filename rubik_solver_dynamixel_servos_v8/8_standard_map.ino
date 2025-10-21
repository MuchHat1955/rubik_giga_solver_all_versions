/********************************************************************
 * RUBIK'S CUBE MOVE CONVERSION & ORIENTATION TRACKER
 * --------------------------------------------------
 *  - Converts solver moves (F,R,U,F',U2) to robot format (f+, f-, f++)
 *  - Maps logical → hardware orientation
 *  - Tracks cube rotations and executes real flips
 ********************************************************************/

// -----------------------------------------------------
// CONVERT STANDARD RUBIK MOVES
// -----------------------------------------------------

String convertStandardMove(const String &standardMove) {
  if (standardMove.length() == 0) return "";

  // Normalize: lowercase first letter for robot format
  char face = tolower(standardMove.charAt(0));
  String converted = String(face);

  // Handle modifier
  if (standardMove.length() == 1) {
    // No modifier = clockwise (f+)
    converted += "+";
  } else {
    char mod = standardMove.charAt(1);
    if (mod == '\'') {
      converted += "-";  // counterclockwise
    } else if (mod == '2') {
      converted += "++";  // double turn
    } else {
      // Keep unknown modifier as-is
      converted += standardMove.substring(1);
    }
  }

  return converted;
}


// -----------------------------------------------------
// RUN CUBE MOVE (MAP LOGICAL → HARDWARE ORIENTATION)
// -----------------------------------------------------

void runCubeStandardMove(const String &moveName) {
  if (moveName.length() == 0) return;

  // Convert F,F',F2 → f+, f-, f++
  String logicalActionName = convertStandardMove(moveName);

  // Extract logical face
  char logicalActionLetter = toupper(logicalActionName.charAt(0));

  // Map to hardware orientation
  char hardwareActionLetter = cubeOri.getMappedMove(logicalActionLetter);

  // Rebuild mapped move name (e.g., f+ → r+ if F→R)
  String hardwareActionName = String((char)tolower(hardwareActionLetter));
  if (logicalActionName.length() > 1)
    hardwareActionName += logicalActionName.substring(1);

  // Debug log
  Serial.printf(
    "[runCubeStandardMove] Logical: %s  →  Hardware: %s  (Ori: %s)\n",
    moveName.c_str(), hardwareActionName.c_str(), cubeOri.getOrientation().c_str());

  // Execute mapped action if known
  if (actionStore.find(hardwareActionName) != actionStore.end()) {
    actionExecute(hardwareActionName);
  } else {
    Serial.printf("⚠️ [runCubeStandardMove] Action not found: %s\n", hardwareActionName.c_str());
  }
}


// -----------------------------------------------------
// CUBE ORIENTATION TRACKER CLASS
// -----------------------------------------------------

class CubeOrientationTracker {
public:
  CubeOrientationTracker() {
    reset();
  }

  // Reset to default: cube resting normally (Front=F, Right=R, Up=U)
  void reset() {
    front = 'F';
    right = 'R';
    up = 'U';
    left = 'L';
    back = 'B';
    down = 'D';
    logOrientation("reset");
  }

  // Short form "FRU"
  String getOrientation() const {
    String s;
    s += front;
    s += right;
    s += up;
    return s;
  }

  // Full 6-face mapping "FRULBD"
  String getFullOrientation() const {
    String s;
    s += front;
    s += right;
    s += up;
    s += left;
    s += back;
    s += down;
    return s;
  }

  // Map logical face (F,R,U,L,B,D) to current cube orientation
  char getMappedMove(char logicalMove) const {
    logicalMove = toupper(logicalMove);
    switch (logicalMove) {
      case 'F': return front;
      case 'R': return right;
      case 'U': return up;
      case 'L': return left;
      case 'B': return back;
      case 'D': return down;
      default: return '?';
    }
  }

  // Rotate cube physically: the specified face moves *to the base (down)*.
  // Also triggers the matching hardware action sequence.
  void rotateCube(const String &op) {
    String s = op;
    s.toLowerCase();

    char oldF = front, oldR = right, oldU = up, oldL = left, oldB = back, oldD = down;

    Serial.printf("[Orientation] Rotating cube: %s\n", s.c_str());

    // Call the matching hardware action (if it exists)
    if (actionStore.find(s) != actionStore.end()) {
      actionExecute(s);
    } else {
      Serial.printf("⚠️ [Orientation] No hardware action for '%s'\n", s.c_str());
    }

    // Update logical orientation model
    if (s == "front_to_base") {
      // Front face moved down
      front = oldU;
      down = oldF;
      up = oldB;
      back = oldD;
      left = oldL;
      right = oldR;

    } else if (s == "back_to_base") {
      back = oldU;
      down = oldB;
      up = oldF;
      front = oldD;
      left = oldL;
      right = oldR;

    } else if (s == "right_to_base") {
      right = oldU;
      up = oldL;
      left = oldD;
      down = oldR;
      front = oldF;
      back = oldB;

    } else if (s == "left_to_base") {
      left = oldU;
      up = oldR;
      right = oldD;
      down = oldL;
      front = oldF;
      back = oldB;

    } else if (s == "top_to_base") {
      up = oldD;
      down = oldU;
      front = oldB;
      back = oldF;
      left = oldL;
      right = oldR;

    } else {
      Serial.printf("⚠️ [Orientation] Unknown rotation '%s'\n", s.c_str());
      return;
    }

    logOrientation(s);
  }

private:
  char front, right, up, left, back, down;

  void logOrientation(const String &op) const {
    Serial.printf("[Orientation] After '%s' → %s (Full:%s)\n",
                  op.c_str(),
                  getOrientation().c_str(),
                  getFullOrientation().c_str());
  }
};

// -----------------------------------------------------
// GLOBAL INSTANCE
// -----------------------------------------------------
CubeOrientationTracker cubeOri;
