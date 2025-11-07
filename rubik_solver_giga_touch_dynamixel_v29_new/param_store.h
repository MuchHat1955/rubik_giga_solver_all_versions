// param_store.h
#pragma once

void initParamStore();

int getParamValue(char* k);
int getParamValue(std::string& k);

void setParamValue(char* k, int v);
void setParamValue(std::string& k, int v);

void runAction(char* key);
void runAction(std::string& key);

void incrementParam(char* k, int v);
