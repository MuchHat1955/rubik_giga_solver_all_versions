// action_store.cpp
#include "action_store.h"
#include "param_store.h"
#include "servos.h"
#include <map>
#include <vector>
#include <string>
#include <Arduino.h>

enum ActionType{ ACTION_POSE, ACTION_GROUP, ACTION_SEQUENCE, ACTION_UNKNOWN };
struct ActionDef{ ActionType type{ACTION_UNKNOWN}; std::vector<std::string> keys; int delay_ms{0};
  ActionDef(){} ActionDef(ActionType t,std::initializer_list<std::string>k,int d=0):type(t),keys(k),delay_ms(d){} };

static std::map<std::string, ActionDef> store;
void runSolveCube(); void runReadCube(); void runScramble(int);
bool runGroupPoseServos(const std::vector<std::string>&, const std::vector<int>&);
static inline std::string baseOf(const std::string& poseKey){ size_t i=poseKey.find('_'); return (i==std::string::npos)?poseKey:poseKey.substr(0,i); }

void actionExecute(const std::string& key){
  auto it=store.find(key);
  if(it==store.end()){
    if(key=="run_solve"){ runSolveCube(); return; }
    if(key=="run_read"){  runReadCube();  return; }
    if(key.rfind("scramble_",0)==0){ int n=atoi(key.c_str()+9); if(n<=0)n=12; runScramble(n); return; }
    Serial.println("Unknown action: %s\n" + String(key.c_str())); return;
  }
  ActionDef &act=it->second;
  switch(act.type){
    case ACTION_POSE:{
      std::string servo=baseOf(act.keys[0]); int v=getParamValue(act.keys[0]);
      if(act.keys[0]=="grip_open"||act.keys[0]=="grip_closed")
        runGroupPoseServos({"grip1","grip2"},{v,v});
      else runGroupPoseServos({servo},{v});
    }break;
    case ACTION_GROUP:{
      std::vector<std::string> s; std::vector<int> t;
      for(auto &k:act.keys){
        if(k=="grip_open"||k=="grip_closed"){ int v=getParamValue(k); s.push_back("grip1");t.push_back(v); s.push_back("grip2");t.push_back(v); }
        else { s.push_back(baseOf(k)); t.push_back(getParamValue(k)); }
      }
      runGroupPoseServos(s,t);
    }break;
    case ACTION_SEQUENCE:{
      for(auto &k: act.keys){ actionExecute(k); if(act.delay_ms>0) delay(act.delay_ms); }
    }break;
    default: break;
  }
}

void initActionStore(){
  store.clear();
  for(auto s: {"arm1","arm2","wrist","grip1","grip2","base"}){
    for(auto suf: {"_0","_1","_2","_90","_180","_minus90","_read1"})
      store[std::string(s)+suf]=ActionDef(ACTION_POSE,{std::string(s)+suf});
  }
  store["grip_open"]=ActionDef(ACTION_GROUP,{"grip_open"});
  store["grip_closed"]=ActionDef(ACTION_GROUP,{"grip_closed"});

  store["arms_0"]=ActionDef(ACTION_GROUP,{"arm1_0","arm2_0"});
  store["arms_row1"]=ActionDef(ACTION_GROUP,{"arm1_1","arm2_1"});
  store["arms_row2"]=ActionDef(ACTION_GROUP,{"arm1_2","arm2_2"});
  store["arms_read1"]=ActionDef(ACTION_GROUP,{"arm1_read1","arm2_read1"});

  store["prog_arm1"]=ActionDef(ACTION_SEQUENCE,{"arm1_0"},200);
  store["prog_arm2"]=ActionDef(ACTION_SEQUENCE,{"arm2_0"},200);
  store["prog_wrist"]=ActionDef(ACTION_SEQUENCE,{"wrist_0"},200);
  store["prog_grip1"]=ActionDef(ACTION_SEQUENCE,{"grip_open","grip_closed"},200);
  store["prog_grip2"]=ActionDef(ACTION_SEQUENCE,{"grip_open","grip_closed"},200);
  store["prog_base"]=ActionDef(ACTION_SEQUENCE,{"base_0","base_90","base_minus90"},200);
  store["prog_test_all"]=ActionDef(ACTION_SEQUENCE,{"arms_0","grip_open","grip_closed","wrist_90","wrist_0"},200);

  for(auto f: {"f","b","l","r","u","d"}){
    store[std::string(f)+"+"]=ActionDef(ACTION_SEQUENCE,{"arms_row1","wrist_90","grip_closed"},200);
    store[std::string(f)+"-"]=ActionDef(ACTION_SEQUENCE,{"grip_closed","base_90","grip_open"},200);
    store[std::string(f)+"++"]=ActionDef(ACTION_SEQUENCE,{"arms_row1","wrist_90","grip_closed"},200);
  }

  store["read_top"]=ActionDef(ACTION_SEQUENCE,{"arms_read1","base_90"},300);
  store["read_middle"]=ActionDef(ACTION_SEQUENCE,{"arms_read1","base_90"},300);
  store["show_sensor"]=ActionDef(ACTION_SEQUENCE,{"arms_read1"},0);
  store["vertical_tune"]=ActionDef(ACTION_SEQUENCE,{"arms_row1","arms_row2","wrist_0"},200);
}
