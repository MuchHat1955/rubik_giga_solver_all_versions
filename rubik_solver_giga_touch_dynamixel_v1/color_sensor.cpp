// color_sensor.cpp
#include "color_sensor.h"
#include <Adafruit_TCS34725.h>
#include <Wire.h>
#include <Arduino.h>

static Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_4X);

void color_sensor_begin(){
  if (!tcs.begin()) Serial.println("[TCS] not found");
  else              Serial.println("[TCS] ready");
}

static void rgb2hsv(float r,float g,float b,float& h,float& s,float& v){
  float mx=max(r,max(g,b)), mn=min(r,min(g,b)); v=mx; float d=mx-mn; s=(mx<=0.0001f)?0.f:(d/mx);
  if(d<0.0001f){ h=0; return; }
  if(mx==r) h=(g-b)/d; else if(mx==g) h=2.f+(b-r)/d; else h=4.f+(r-g)/d; h*=60.f; if(h<0) h+=360.f;
}
static char classify(float r,float g,float b){
  float h,s,v; rgb2hsv(r,g,b,h,s,v); v/=255.f;
  if(v>0.85f||s<0.12f) return 'W';
  if(h>=0&&h<20) return 'R'; if(h>=20&&h<45) return 'O'; if(h>=45&&h<90) return 'Y';
  if(h>=90&&h<160) return 'G'; if(h>=160&&h<250) return 'B'; return 'X';
}

char readColorSensor(){
  uint16_t r,g,b,c; tcs.getRawData(&r,&g,&b,&c);
  if(c<50) return 'X';
  float rf=(float)r/c*255.f, gf=(float)g/c*255.f, bf=(float)b/c*255.f;
  return classify(rf,gf,bf);
}
char readColorSmoothed(int samples,int delayMs){
  float rs=0,gs=0,bs=0; int valid=0;
  for(int i=0;i<samples;i++){
    uint16_t r,g,b,c; tcs.getRawData(&r,&g,&b,&c);
    if(c>50){ rs+=(float)r/c*255.f; gs+=(float)g/c*255.f; bs+=(float)b/c*255.f; valid++; }
    delay(delayMs);
  }
  if(!valid) return 'X';
  return classify(rs/valid, gs/valid, bs/valid);
}
