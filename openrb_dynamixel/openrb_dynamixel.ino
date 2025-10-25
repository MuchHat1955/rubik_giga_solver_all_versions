#include <Dynamixel2Arduino.h>
#include <math.h>

#define DXL_SERIAL   Serial1
#define DXL_DIR_PIN  -1
#define PROTOCOL     2.0

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// managed servo IDs
uint8_t servo_ids[] = {11, 12, 13, 14, 15, 16};
const uint8_t SERVO_COUNT = sizeof(servo_ids) / sizeof(servo_ids[0]);

const int      STALL_CURRENT_mA   = 1000;
const uint8_t  CHECK_INTERVAL_MS  = 20;
const uint32_t STATUS_INTERVAL_MS = 300;

bool stalled_flags[6] = {false,false,false,false,false,false};
unsigned long last_check_ms = 0;

// ----------------------------------------------------
// helpers
// ----------------------------------------------------
int index_of_id(uint8_t id){
  for(uint8_t i=0;i<SERVO_COUNT;i++) if(servo_ids[i]==id) return i;
  return -1;
}
bool is_managed(uint8_t id){ return index_of_id(id)>=0; }
bool stalled_for_id(uint8_t id){
  int i=index_of_id(id);
  return (i>=0)?stalled_flags[i]:false;
}

// ----------------------------------------------------
// LED helpers using high-level API
// ----------------------------------------------------
void ledOn(uint8_t id){ if(dxl.ping(id)) dxl.ledOn(id); }
void ledOff(uint8_t id){ if(dxl.ping(id)) dxl.ledOff(id); }

void ledBlink(uint8_t id,uint16_t period_ms=500){
  static unsigned long last_toggle[256]={0};
  static bool state[256]={false};
  unsigned long now=millis();
  if(now-last_toggle[id]>=period_ms){
    last_toggle[id]=now;
    state[id]=!state[id];
    if(state[id]) dxl.ledOn(id);
    else dxl.ledOff(id);
  }
}

// ----------------------------------------------------
// stall / error monitor
// ----------------------------------------------------
void monitor_stalls_subset(const uint8_t* ids,uint8_t count){
  if(millis()-last_check_ms<CHECK_INTERVAL_MS) return;
  last_check_ms=millis();

  for(uint8_t i=0;i<count;i++){
    uint8_t id=ids[i];
    if(!dxl.ping(id)) continue;
    int curr = dxl.getPresentCurrent(id);
    int temp = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE,id);
    if(curr>STALL_CURRENT_mA || temp>70){
      int idx=index_of_id(id);
      if(idx>=0) stalled_flags[idx]=true;
      dxl.torqueOff(id);
      // indicate fault
      ledBlink(id,300);
    }
  }
}

// ----------------------------------------------------
// print servo status
// ----------------------------------------------------
void print_status(const uint8_t* ids,uint8_t count){
  Serial.print("STATUS ");
  for(uint8_t i=0;i<count;i++){
    uint8_t id=ids[i];
    if(!dxl.ping(id)){
      Serial.print(id);Serial.print(" X X X");
      if(i+1<count)Serial.print(' ');
      continue;
    }
    int pos  = dxl.getPresentPosition(id);
    int temp = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE,id);
    int curr = dxl.getPresentCurrent(id);

    Serial.print(id);Serial.print(' ');
    Serial.print(pos);
    if(stalled_for_id(id)) Serial.print('S');
    Serial.print(' ');
    Serial.print(temp);Serial.print(' ');
    Serial.print(curr);
    if(i+1<count)Serial.print(' ');
  }
  Serial.println();
}

// ----------------------------------------------------
// motion smoothing with LED behaviour
// ----------------------------------------------------
float ease_cos(float t){ return 0.5f-0.5f*cosf(t*PI); }

void smooth_move_sync(const uint8_t* ids,const int* goals,uint8_t count,uint32_t duration_ms){
  // prepare servos
  for(uint8_t i=0;i<count;i++){
    dxl.torqueOff(ids[i]);
    dxl.setOperatingMode(ids[i],OP_POSITION);
    dxl.torqueOn(ids[i]);
    if(is_managed(ids[i])) ledOn(ids[i]);   // LED on while moving
  }

  int starts[6];
  for(uint8_t i=0;i<count;i++){
    starts[i]=dxl.getPresentPosition(ids[i]);
  }

  const uint32_t step_ms=20;
  uint32_t steps=max<uint32_t>(1,duration_ms/step_ms);
  uint32_t t0=millis(),last_stat=0;

  for(uint32_t s=0;s<=steps;s++){
    uint32_t now=millis();
    float t=(float)(now-t0)/(float)duration_ms;
    if(t>1)t=1;
    float k=ease_cos(t);

    for(uint8_t i=0;i<count;i++){
      if(stalled_for_id(ids[i])) continue;
      int pos=starts[i]+(int)((goals[i]-starts[i])*k);
      dxl.setGoalPosition(ids[i],pos);
    }

    monitor_stalls_subset(ids,count);
    if(now-last_stat>=STATUS_INTERVAL_MS){
      print_status(servo_ids,SERVO_COUNT);
      last_stat=now;
    }
    if(t>=1)break;
    delay(step_ms);
  }

  delay(10);
  monitor_stalls_subset(ids,count);
  print_status(servo_ids,SERVO_COUNT);

  // restore LED states
  for(uint8_t i=0;i<count;i++){
    if(!dxl.ping(ids[i])) continue;
    if(stalled_for_id(ids[i])) ledBlink(ids[i],300);
    else ledOff(ids[i]); // standby
  }
}

// ----------------------------------------------------
// command handlers
// ----------------------------------------------------
void handle_move(const String& line){
  uint8_t ids[6]; int goals[6]; uint8_t count=0; uint32_t dur=1000;
  char buf[160]; line.toCharArray(buf,sizeof(buf));
  char* tok=strtok(buf," "); tok=strtok(NULL," ");
  while(tok&&count<6){
    if(strcasecmp(tok,"DURATION")==0)break;
    int id=atoi(tok); tok=strtok(NULL," "); if(!tok)break;
    int pos=atoi(tok); ids[count]=id; goals[count]=pos; count++; tok=strtok(NULL," ");
  }
  if(tok&&strcasecmp(tok,"DURATION")==0){
    char* d=strtok(NULL," ");
    if(d){ long v=atol(d); if(v>0) dur=v; }
  }
  if(count==0){ Serial.println("ERR MOVE"); return; }
  smooth_move_sync(ids,goals,count,dur);
}

void handle_read(const String& line){
  uint8_t ids[6]; uint8_t cnt=0; char buf[96];
  line.toCharArray(buf,sizeof(buf));
  char* tok=strtok(buf," "); tok=strtok(NULL," ");
  while(tok&&cnt<6){ ids[cnt++]=atoi(tok); tok=strtok(NULL," "); }
  if(cnt==0) print_status(servo_ids,SERVO_COUNT);
  else       print_status(ids,cnt);
}

void handle_program(const String& line){
  int old_id,new_id;
  if(sscanf(line.c_str(),"PROGRAM %d %d",&old_id,&new_id)==2){
    if(!dxl.ping(old_id)){ Serial.println("ERR ON PROGRAM: ID not found"); return; }
    dxl.torqueOff(old_id);
    bool ok=dxl.writeControlTableItem(ControlTableItem::ID,old_id,new_id);
    if(ok){
      int idx=index_of_id(old_id); if(idx>=0) servo_ids[idx]=new_id;
      Serial.println("PROGRAM OK");
    } else Serial.println("ERR ON PROGRAM");
  } else Serial.println("ERR ON PROGRAM");
}

// ----------------------------------------------------
void setup(){
  Serial.begin(115200);
  while(!Serial);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(PROTOCOL);

  Serial.println("=== OpenRB 6-Servo Controller ===");

  for(uint8_t i=0;i<SERVO_COUNT;i++){
    uint8_t id=servo_ids[i];
    if(dxl.ping(id)){
      dxl.torqueOff(id);
      dxl.setOperatingMode(id,OP_POSITION);
      dxl.torqueOn(id);
      stalled_flags[i]=false;
      ledOff(id);  // standby off
      Serial.print("Servo ");Serial.print(id);Serial.println(" OK");
    }else{
      Serial.print("Servo ");Serial.print(id);Serial.println(" not found!");
    }
  }

  Serial.println("Commands: MOVE <id pos>...[DURATION ms], READ [id...], PROGRAM <old> <new>, LEDON <id>, LEDOFF <id>");
}

// ----------------------------------------------------
void loop(){
  // blink any stalled servos continuously
  for(uint8_t i=0;i<SERVO_COUNT;i++){
    if(stalled_flags[i]) ledBlink(servo_ids[i],300);
  }

  if(!Serial.available()) return;
  String line=Serial.readStringUntil('\n');
  line.trim(); if(line.length()==0) return;

  if(line.startsWith("MOVE"))         handle_move(line);
  else if(line.startsWith("READ"))    handle_read(line);
  else if(line.startsWith("PROGRAM")) handle_program(line);
  else if(line.startsWith("LEDON")){
    int id=atoi(line.substring(6).c_str());
    dxl.ledOn(id);
    Serial.println("LED ON");
  }
  else if(line.startsWith("LEDOFF")){
    int id=atoi(line.substring(7).c_str());
    dxl.ledOff(id);
    Serial.println("LED OFF");
  }
  else Serial.println("ERR CMD");
}
