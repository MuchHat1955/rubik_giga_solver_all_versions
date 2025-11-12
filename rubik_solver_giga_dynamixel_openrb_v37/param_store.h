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

// ---------------- Action dispatch ----------------
void runAction(const char* key);
void runAction(const std::string& key);

#define PARAM_VAL_NA 999.0
