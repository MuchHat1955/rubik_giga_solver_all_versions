// param_store.h
#pragma once
#include <string>
void initParamStore();
int  getParamValue(const std::string& k);
void setParamValue(const std::string& k, int v);