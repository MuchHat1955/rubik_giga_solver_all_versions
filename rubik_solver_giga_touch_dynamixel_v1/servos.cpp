// servos.cpp
#include <Arduino.h>
#include "servos.h"
#include "param_store.h"
#include <Dynamixel2Arduino.h>

// ---- EDIT to match your wiring ----
#define DXL_SERIAL  Serial1
#define DXL_DIR_PIN 2
#define DXL_BAUD    57600

static Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

struct { const char* name; uint8_t id; } smap[] = {
  {"arm1",11},{"arm2",12},{"wrist",13},{"grip1",14},{"grip2",15},{"base",16},{nullptr,0}
};

static int getId(const std::string& n){ for(int i=0;smap[i].name;++i) if(n==smap[i].name) return smap[i].id; return -1; }

void dynamixel_begin(){
  DXL_SERIAL.begin(DXL_BAUD);
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(2.0);
}

int readServoTicks(const std::string& name){
  int id=getId(name); if(id<0) return 0;
  return dxl.getPresentPosition(id);
}

int getServoTicksForAngle(const std::string& servo, float angle_deg,
                          int, int){
  int z  = getParamValue(servo+"_0_tick");
  int p  = getParamValue(servo+"_plus_tick");
  int pd = getParamValue(servo+"_plus_deg"); if (pd==0) pd=90;
  long d=(long)p-(long)z;
  long t=z + (long)(d*(angle_deg/(float)pd));
  if(t<0)t=0; if(t>4095)t=4095;
  return (int)t;
}

void rotateBaseToAngle(int angle_deg){
  int base0=getParamValue("base_0");
  int base90=getParamValue("base_90");
  long d=(long)base90-(long)base0;
  long t=base0 + d*angle_deg/90;
  if(t<0)t=0; if(t>4095)t=4095;
  int id=getId("base");
  dxl.setGoalPosition(id, (int)t);
}
