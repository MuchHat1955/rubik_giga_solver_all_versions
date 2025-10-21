// param_store.h
#pragma once
#include <string>
void initParamStore();

int getParamValue(const char* k);
int getParamValue(const std::string& k);

void setParamValue(const char* k, int v);
void setParamValue(const std::string& k, int v);

void incrementParam(const char* key, int delta);
void incrementParam(const std::string& key, int delta);

void runAction(const char* key);
void runAction(const std::string& key);