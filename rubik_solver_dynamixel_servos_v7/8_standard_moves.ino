/********************************************************************
 * RUN THE MOVES FOR SOLVING
 ********************************************************************/

String convertStandardMove(const String &standardMove) {
  if (standardMove.length() == 0) return "";

  // Extract the face letter
  char face = tolower(standardMove.charAt(0));  // convert to lowercase for your format
  String converted = String(face);

  // Look at any modifier after the face
  if (standardMove.length() == 1) {
    // Plain move = clockwise
    converted += "+";
  } else {
    char mod = standardMove.charAt(1);
    if (mod == '\'') {
      converted += "-";  // counterclockwise
    } else if (mod == '2') {
      converted += "++";  // double turn
    } else {
      // Unrecognized modifier—keep as-is
      converted += standardMove.substring(1);
    }
  }

  return converted;
}


void runCubeStandarfMove(const String &moveName) {
  if (moveName.length() == 0) return;

  String logicalActionName = convertStandardMove(moveName);

  // 1️⃣ Extract the logical face (first letter, uppercase)
  char logicalActionLetter = toupper(logicalActionName.charAt(0));

  // 2️⃣ Map the logical face to current hardware orientation
  char hardwareActionLetter = cubeOri.getMappedMove(logicalActionLetter);

  // 3️⃣ Rebuild the full hardware move name by replacing only the first letter
  String hardwareActionName = String(hardwareActionLetter);
  if (actionName.length() > 1) {
    hardwareActionName += logicalActionName.substring(1);  // preserve +, -, 180, etc.
  }

  // 4️⃣ Log for debug
  Serial.printf(
    "[runCubeMove] Standard move: %s  →  Mapped hardware: %s  (current ori: %s)\n",
    moveName.c_str(), hardwareActionName.c_str(), cubeOri.getOrientation().c_str());

  // 5️⃣ Execute mapped action (assume already in actionStore)
  actionExecute(hardwareActionName);
}


class CubeOrientationTracker {
public:
  CubeOrientationTracker() {
    reset();
  }

  // Reset to the default cube orientation (Front=F, Right=R, Up=U)
  void reset() {
    front = 'F';
    right = 'R';
    up = 'U';
    left = 'L';
    back = 'B';
    down = 'D';
    logOrientation("reset");
  }

  // Return current short orientation, e.g. "FRU"
  String getOrientation() const {
    String s;
    s += front;
    s += right;
    s += up;
    return s;
  }

  // Return full 6-face orientation, e.g. "FRULBD"
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

  // Map a logical move (F,R,U,L,B,D) to the current real cube face
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

  // Rotate cube physically (the specified face moves to the BASE)
  void rotateCube(const String &op) {
    String s = op;
    s.toLowerCase();

    char oldF = front, oldR = right, oldU = up, oldL = left, oldB = back, oldD = down;

    if (s == "front_to_base") {
      // Front face moved down
      front = oldU;
      down = oldF;
      up = oldB;
      back = oldD;
      left = oldL;
      right = oldR;
    } else if (s == "back_to_base") {
      // Back face moved down
      front = oldD;
      down = oldB;
      up = oldF;
      back = oldU;
      left = oldL;
      right = oldR;
    } else if (s == "right_to_base") {
      // Right face moved down
      right = oldU;
      up = oldL;
      left = oldD;
      down = oldR;
      front = oldF;
      back = oldB;
    } else if (s == "left_to_base") {
      // Left face moved down
      left = oldU;
      up = oldR;
      right = oldD;
      down = oldL;
      front = oldF;
      back = oldB;
    } else if (s == "top_to_base") {
      // Cube flipped upside down
      up = oldD;
      down = oldU;
      front = oldB;
      back = oldF;
      left = oldL;
      right = oldR;
    } else {
      Serial.printf("[Orientation] Unknown rotation '%s'\n", s.c_str());
      return;
    }

    logOrientation(s);
  }

private:
  char front, right, up, left, back, down;

  void logOrientation(const String &op) const {
    Serial.printf("[Orientation] After '%s' -> %s (Full:%s)\n",
                  op.c_str(),
                  getOrientation().c_str(),
                  getFullOrientation().c_str());
  }
};