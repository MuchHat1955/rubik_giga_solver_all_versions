#include "vertical_kinematics.h"
#include "servo_manager.h"
#include "param_store.h"
#include "logging.h"
#include <math.h>

extern ServoManager servoManager;
extern bool logging_on;

// ----------------------------------------------------------
//                    GLOBALS / CONSTANTS
// ----------------------------------------------------------
static constexpr int MAX_REFS = 6;
static constexpr float DEG2RAD = 3.1415926f / 180.0f;

static RefPoint refs[MAX_REFS];
static int refCountVar = 0;

static float armAmm = 48.0f;
static float armBmm = 48.0f;
static float offsetXmm = 60.0f;

// ----------------------------------------------------------
//              INTERNAL HELPERS AND CONVERSION
// ----------------------------------------------------------
static inline int encodeFloat(float val) { return (int)round(val * 100.0f); }
static inline float decodeFloat(int val) { return ((float)val) / 100.0f; }

// ----------------------------------------------------------
//                     KEY HELPERS
// ----------------------------------------------------------
static const char* arm1Keys[MAX_REFS] = {
  "arm1_v_pt1", "arm1_v_pt2", "arm1_v_pt3",
  "arm1_v_pt4", "arm1_v_pt5", "arm1_v_pt6"
};
static const char* arm2Keys[MAX_REFS] = {
  "arm2_v_pt1", "arm2_v_pt2", "arm2_v_pt3",
  "arm2_v_pt4", "arm2_v_pt5", "arm2_v_pt6"
};
static const char* wristKeys[MAX_REFS] = {
  "wrist_v_pt1", "wrist_v_pt2", "wrist_v_pt3",
  "wrist_v_pt4", "wrist_v_pt5", "wrist_v_pt6"
};

// ----------------------------------------------------------
//                    INIT / LOAD / SAVE
// ----------------------------------------------------------
void vkinInit() {
    LOG_SECTION_START("vkinInit");

    refCountVar = 0;
    armAmm = 48.0f;
    armBmm = 48.0f;
    offsetXmm = 60.0f;

    for (int i = 0; i < MAX_REFS; i++) {
        int a1 = getParamValue(arm1Keys[i]);
        int a2 = getParamValue(arm2Keys[i]);
        int wr = getParamValue(wristKeys[i]);

        // A reference point is considered valid only if all three are nonzero
        if (a1 == 0 && a2 == 0 && wr == 0) {
            refs[i] = {0, 0, 0, 0, 0};
            continue;
        }

        // Decode to float mm/deg
        refs[i].aDeg = decodeFloat(a1);
        refs[i].bDeg = decodeFloat(a2);
        refs[i].xMm  = decodeFloat(wr);  // reuse wrist slot for x offset or auxiliary data
        refs[i].yMm  = (float)(i + 1) * 2.0f;  // estimated spacing (you can override later)
        refs[i].diffX = 0;

        refCountVar = max(refCountVar, i + 1);

        LOG_SECTION_START_VAR("loaded ref", "index", String(i));
        LOG_VAR2("a", refs[i].aDeg, "b", refs[i].bDeg);
        LOG_VAR2("x", refs[i].xMm, "y", refs[i].yMm);
        LOG_SECTION_END();
    }

    LOG_VAR("refCount", refCountVar);
    LOG_SECTION_END();
}

// ----------------------------------------------------------
//                SAVE OR CLEAR INDIVIDUAL REFS
// ----------------------------------------------------------
static void saveRefToParams(int index) {
    if (index < 0 || index >= MAX_REFS) return;
    const RefPoint& r = refs[index];

    setParamValue(arm1Keys[index], encodeFloat(r.aDeg));
    setParamValue(arm2Keys[index], encodeFloat(r.bDeg));
    setParamValue(wristKeys[index], encodeFloat(r.xMm));
}

void clearRefAtIndex(int index) {
    if (index < 0 || index >= MAX_REFS) return;

    LOG_SECTION_START_VAR("clearRefAtIndex", "index", String(index));

    refs[index] = {0, 0, 0, 0, 0};
    setParamValue(arm1Keys[index], 0);
    setParamValue(arm2Keys[index], 0);
    setParamValue(wristKeys[index], 0);

    while (refCountVar > 0 &&
           refs[refCountVar - 1].aDeg == 0 &&
           refs[refCountVar - 1].bDeg == 0)
        refCountVar--;

    LOG_SECTION_END();
}

// ----------------------------------------------------------
//                        ADD / SET
// ----------------------------------------------------------
void addRef(float yMm, float xMm, float aDeg, float bDeg) {
    int index = (refCountVar < MAX_REFS) ? refCountVar : MAX_REFS - 1;
    refs[index] = { yMm, xMm, aDeg, bDeg, 0.0f };
    if (index >= refCountVar) refCountVar = index + 1;

    saveRefToParams(index);

    LOG_SECTION_START_VAR("addRef", "index", String(index));
    LOG_VAR2("y", yMm, "x", xMm);
    LOG_VAR2("a", aDeg, "b", bDeg);
    LOG_SECTION_END();
}

void setRefAtIndex(int index, float yMm, float xMm, float aDeg, float bDeg) {
    if (index < 0 || index >= MAX_REFS) return;
    refs[index] = { yMm, xMm, aDeg, bDeg, 0.0f };
    refCountVar = max(refCountVar, index + 1);
    saveRefToParams(index);

    LOG_SECTION_START_VAR("setRefAtIndex", "index", String(index));
    LOG_VAR2("y", yMm, "x", xMm);
    LOG_VAR2("a", aDeg, "b", bDeg);
    LOG_SECTION_END();
}

// ----------------------------------------------------------
//                 ACCESSORS / UTILITIES
// ----------------------------------------------------------
bool hasRef(int index) { return (index < refCountVar && refs[index].aDeg != 0); }
int refCount() { return refCountVar; }

void printRefs() {
    LOG_SECTION_START("printRefs");
    for (int i = 0; i < refCountVar; i++) {
        if (!hasRef(i)) continue;
        LOG_VAR2_("i", i, "y", refs[i].yMm);
        LOG_VAR2_CONT_("x", refs[i].xMm, "a", refs[i].aDeg);
        LOG_VAR2_CONT("b", refs[i].bDeg, "Δx", refs[i].diffX);
    }
    LOG_SECTION_END();
}

// ----------------------------------------------------------
//                LEAST-SQUARES SOLVER
// ----------------------------------------------------------
void solveLeastSquares() {
    LOG_SECTION_START("solveLeastSquares");

    if (refCountVar < 3) {
        LOG_VAR("warning", "need ≥3 refs");
        LOG_SECTION_END();
        return;
    }

    float sumAA=0, sumBB=0, sumAB=0, sumAX=0, sumBX=0;
    for (int i=0; i<refCountVar; i++) {
        if (!hasRef(i)) continue;

        float A = refs[i].aDeg * DEG2RAD;
        float B = refs[i].bDeg * DEG2RAD;
        float cosA = cosf(A), cosB = cosf(B);

        sumAA += cosA * cosA;
        sumBB += cosB * cosB;
        sumAB += cosA * cosB;
        sumAX += cosA * (refs[i].xMm - offsetXmm);
        sumBX += cosB * (refs[i].xMm - offsetXmm);
    }

    float det = sumAA * sumBB - sumAB * sumAB;
    if (fabs(det) < 1e-6f) {
        LOG_VAR("error", "degenerate LS system");
        LOG_SECTION_END();
        return;
    }

    armAmm = (sumAX*sumBB - sumBX*sumAB) / det;
    armBmm = (sumBX*sumAA - sumAX*sumAB) / det;

    for (int i=0; i<refCountVar; i++) {
        if (!hasRef(i)) continue;
        float A = refs[i].aDeg * DEG2RAD;
        float B = refs[i].bDeg * DEG2RAD;
        float xPred = offsetXmm + armAmm*cosf(A) + armBmm*cosf(B);
        refs[i].diffX = refs[i].xMm - xPred;
    }

    LOG_VAR2("a(mm)", armAmm, "b(mm)", armBmm);
    LOG_VAR("offsetX(mm)", offsetXmm);
    LOG_SECTION_END();
}

// ----------------------------------------------------------
//               COMPUTE RMS X ERROR
// ----------------------------------------------------------
float computeXError() {
    if (refCountVar == 0) return 0;
    float sum = 0; int cnt = 0;
    for (int i=0; i<refCountVar; i++) {
        if (!hasRef(i)) continue;
        sum += refs[i].diffX * refs[i].diffX;
        cnt++;
    }
    return (cnt == 0) ? 0 : sqrtf(sum / cnt);
}

// ----------------------------------------------------------
//            MOVE BOTH ARMS TO Y TARGET
// ----------------------------------------------------------
static inline int angleToTicks(float angleDeg) {
    const float TICKS_PER_DEG = 3414.0f / 270.0f;
    return (int)(angleDeg * TICKS_PER_DEG);
}

void moveToY(float yTargetMm) {
    LOG_SECTION_START_VAR("moveToY", "targetY(mm)", String(yTargetMm));

    float bestAngle = 45.0f;
    float bestDiff = 1e9f;

    for (float ang=10.0f; ang<80.0f; ang+=0.1f) {
        float A = ang * DEG2RAD;
        float B = ang * DEG2RAD;
        float yPred = armAmm * sinf(A) - armBmm * sinf(B);
        float diff = fabsf(yPred - yTargetMm);
        if (diff < bestDiff) {
            bestDiff = diff;
            bestAngle = ang;
        }
    }

    int ticksLeft  = angleToTicks(bestAngle);
    int ticksRight = angleToTicks(180.0f - bestAngle);

    servoManager.moveServoToTicks("11", ticksLeft);
    servoManager.moveServoToTicks("12", ticksRight);

    LOG_VAR2("angle(deg)", bestAngle, "error(mm)", bestDiff);
    LOG_VAR2("ticksL", ticksLeft, "ticksR", ticksRight);

    LOG_SECTION_END();
}

// ----------------------------------------------------------
//              ACCESSORS FOR UI DISPLAY
// ----------------------------------------------------------
float getA() { return armAmm; }
float getB() { return armBmm; }
float getOffsetX() { return offsetXmm; }
