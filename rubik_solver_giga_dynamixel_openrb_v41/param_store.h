#pragma once
#include "utils.h"
#include <string>

void initParamStore();

// ---------------- Parameter access ----------------
double getParamValue(const char* k);
double getParamValue(std::string& k);
void setParamValue(const char* k, double v);
void setParamValue(std::string& k, double v);
void increment_pose_param_in_pose_and_param_stores(const char* k, int delta);

#define PARAM_VAL_NA 999.0
