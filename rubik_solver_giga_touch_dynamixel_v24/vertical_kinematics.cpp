#include "vertical_kinematics.h"
#include "servo_manager.h"
#include "logging.h"
#include <math.h>

extern ServoManager servoManager;

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
//                        HELPERS
// ----------------------------------------------------------
void vkinInit() {
    refCountVar = 0;
    armAmm = 48.0f;
    armBmm = 48.0f;
    offsetXmm = 60.0f;
}

void addRef(float yMm, float xMm, float aDeg, float bDeg) {
    if (refCountVar >= MAX_REFS) return;
    refs[refCountVar++] = { yMm, xMm, aDeg, bDeg, 0 };
    LOGF("Ref[%d] y=%.2f x=%.2f a=%.1f b=%.1f", refCountVar-1, yMm, xMm, aDeg, bDeg);
}

void clearRefs() {
    refCountVar = 0;
    LOG("Refs cleared");
}

bool hasRef(int index) {
    return index < refCountVar;
}

int refCount() { return refCountVar; }

void printRefs() {
    for (int i=0; i<refCountVar; i++) {
        LOGF("[%d] y=%.2f x=%.2f a=%.1f b=%.1f", i, refs[i].yMm, refs[i].xMm, refs[i].aDeg, refs[i].bDeg);
    }
}

// ----------------------------------------------------------
//                LEAST-SQUARES SOLVER
// ----------------------------------------------------------
void solveLeastSquares() {
    if (refCountVar < 3) {
        LOGW("Need ≥3 refs for solve");
        return;
    }

    float sumAA=0, sumBB=0, sumAB=0, sumAX=0, sumBX=0;
    for (int i=0; i<refCountVar; i++) {
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
        LOGE("Degenerate LS system");
        return;
    }

    armAmm = (sumAX*sumBB - sumBX*sumAB) / det;
    armBmm = (sumBX*sumAA - sumAX*sumAB) / det;

    for (int i=0; i<refCountVar; i++) {
        float A = refs[i].aDeg * DEG2RAD;
        float B = refs[i].bDeg * DEG2RAD;
        float xPred = offsetXmm + armAmm*cosf(A) + armBmm*cosf(B);
        refs[i].diffX = refs[i].xMm - xPred;
    }

    LOGF("Solved: a=%.2fmm  b=%.2fmm  offset=%.2fmm", armAmm, armBmm, offsetXmm);
}

// ----------------------------------------------------------
//               COMPUTE RMS X ERROR
// ----------------------------------------------------------
float computeXError() {
    if (refCountVar == 0) return 0;
    float sum = 0;
    for (int i=0; i<refCountVar; i++)
        sum += refs[i].diffX * refs[i].diffX;
    return sqrtf(sum / refCountVar);
}

// ----------------------------------------------------------
//            MOVE BOTH ARMS TO Y TARGET
// ----------------------------------------------------------
void moveToY(float yTargetMm) {
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

    float ticksLeft  = servoManager.angleToTicks(11, bestAngle);
    float ticksRight = servoManager.angleToTicks(12, 180.0f - bestAngle);

    servoManager.moveServoToTicks(11, ticksLeft);
    servoManager.moveServoToTicks(12, ticksRight);

    LOGF("moveToY(%.2fmm) => θ=%.2f°, L:%d R:%d", yTargetMm, bestAngle, (int)ticksLeft, (int)ticksRight);
}

// ----------------------------------------------------------
//              ACCESSORS FOR UI DISPLAY
// ----------------------------------------------------------
float getA() { return armAmm; }
float getB() { return armBmm; }
float getOffsetX() { return offsetXmm; }
