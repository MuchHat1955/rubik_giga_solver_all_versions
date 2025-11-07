// param_store.h
#pragma once
#include "utils.h"

void initParamStore();

int getParamValue(const char* k);
int getParamValue(std::string& k);

void setParamValue(const char* k, int v);
void setParamValue(std::string& k, int v);

void runAction(const char* key);
void runAction(std::string& key);

void incrementParam(const char* k, int v);

