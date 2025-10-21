#pragma once
#include <Arduino.h>

struct RefPoint {
    float yMm;
    float xMm;
    float aDeg;
    float bDeg;
    float diffX;
};

void vkinInit();
void addRef(float yMm, float xMm, float aDeg, float bDeg);
void clearRefs();
bool hasRef(int index);
int  refCount();
void solveLeastSquares();
void moveToY(float yTargetMm);
float computeXError();
void printRefs();
float getA();
float getB();
float getOffsetX();
