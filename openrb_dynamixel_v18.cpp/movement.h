#pragma once
#include "movement.h"
#include <Arduino.h>
#include "servos.h"
#include "vertical_kinematics.h"
#include "utils.h"

// -------------------------------------------------------------------
//                        MOVEMENT COMMANDS
// -------------------------------------------------------------------

// Smooth movement (single servo)
bool cmdMoveSmooth(uint8_t id, int goal);

// Coordinated moves
bool cmdMoveYSyncSmooth(double goal1_deg, double goal2_deg, double goalG_deg);
bool cmdMoveXSyncSmooth(int delta1, int delta2, int deltaG);

// XYG coordinated motion
bool cmdMoveSmoothXYG(double goal_xmm, double goal_ymm,
                      bool keepXconstant,
                      double tol_xmm, double tol_ymm, double tol_gdeg,
                      double oneTickMm, double oneTickDeg,
                      bool nudge_x_enabled = true,
                      bool nudge_y_enabled = false,
                      bool nudge_g_enabled = false);

// Gripper-only motion
bool cmdMoveSmoothG(double goal_gdeg, double tol_gdeg, double oneTickDeg);

// Basic axis commands
bool cmdMoveYmm(double y_mm);
bool cmdMoveXmm(double x_mm);

// Test motion routines
void cmdTestMove(uint8_t id, int count = 40);
void cmdTestMoveX(int rel_ticks);

// -------------------------------------------------------------------
//                      CONSTANTS & MACROS
// -------------------------------------------------------------------
#define KEEP_X_CONSTANT true
#define KEEP_Y_CONSTANT false
