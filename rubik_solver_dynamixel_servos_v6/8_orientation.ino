/********************************************************************
 * RUN THE MOVES FOR SOLVING
 ********************************************************************/

class CubeOrientationTracker {
public:
  CubeOrientationTracker() { reset(); }

  // Reset to the default cube orientation (Front=F, Right=R, Up=U)
  void reset() {
    front = 'F';
    right = 'R';
    up    = 'U';
    left  = 'L';
    back  = 'B';
    down  = 'D';
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
    s += front; s += right; s += up;
    s += left;  s += back;  s += down;
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
      default:  return '?';
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
      down  = oldF;
      up    = oldB;
      back  = oldD;
      left  = oldL;
      right = oldR;
    }
    else if (s == "back_to_base") {
      // Back face moved down
      front = oldD;
      down  = oldB;
      up    = oldF;
      back  = oldU;
      left  = oldL;
      right = oldR;
    }
    else if (s == "right_to_base") {
      // Right face moved down
      right = oldU;
      up    = oldL;
      left  = oldD;
      down  = oldR;
      front = oldF;
      back  = oldB;
    }
    else if (s == "left_to_base") {
      // Left face moved down
      left  = oldU;
      up    = oldR;
      right = oldD;
      down  = oldL;
      front = oldF;
      back  = oldB;
    }
    else if (s == "top_to_base") {
      // Cube flipped upside down
      up    = oldD;
      down  = oldU;
      front = oldB;
      back  = oldF;
      left  = oldL;
      right = oldR;
    }
    else {
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


// example of usage

CubeOrientationTracker cubeOri;

void setup() {
  Serial.begin(115200);
  cubeOri.reset();  // Logs FRU

  cubeOri.rotateCube("front_to_base");  // Logs new orientation
  cubeOri.rotateCube("right_to_base");
  cubeOri.rotateCube("top_to_base");

  char mapped = cubeOri.getMappedMove('F');
  Serial.printf("To execute logical F -> hardware %c\n", mapped);
}
