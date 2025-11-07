#pragma once
#include "utils.h"
#include <string>

void initParamStore();

// ---------------- Parameter access ----------------
int  getParamValue(const char* k);
int  getParamValue(std::string& k);
void setParamValue(const char* k, int v);
void setParamValue(std::string& k, int v);
void incrementParam(const char* k, int delta);

// ---------------- Action dispatch ----------------
void runAction(const char* key);
void runAction(const std::string& key);
