#pragma once
#include <Arduino.h>

// Run the vertical tune wizard (auto arm geometry detection)
void runVerticalTuningUI();

// Data results stored globally (for now)
extern float arm1_length_mm;
extern float arm2_length_mm;
extern float base_x_offset_mm;
