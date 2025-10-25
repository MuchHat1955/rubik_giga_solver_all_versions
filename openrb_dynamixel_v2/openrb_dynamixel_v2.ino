#include <Dynamixel2Arduino.h>
#include <math.h>

#define DXL_SERIAL   Serial1
#define DXL_DIR_PIN  -1
#define PROTOCOL     2.0
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ---------------- CONFIGURABLE IDs ----------------
#define ID_LEFT   11
#define ID_RIGHT  12
#define ID_WRIST  13
uint8_t servo_ids[] = {ID_LEFT, ID_RIGHT, ID_WRIST, 14, 15, 16};
const uint8_t SERVO_COUNT = sizeof(servo_ids) / sizeof(servo_ids[0]);

// ---------------- CONSTANTS ----------------
const int STALL_CURRENT_mA = 1000;
const uint8_t CHECK_INTERVAL_MS = 20;

// ---------------- CALIBRATION ----------------
float arm_length_mm = 0;     // 0 = unset
int tick_zero_left  = -1;    // -1 = unset
int tick_zero_right = -1;

bool has_L  = false;
bool has_ZL = false;
bool has_ZR = false;

// ---------------- FLAGS ----------------
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
// LED helpers
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
    if(state[id]) dxl.ledOn(id); else dxl.ledOff(id);
  }
}

// ----------------------------------------------------
// Small single-servo micro-move with overshoot
// ----------------------------------------------------
void smallSingleServoMove(uint8_t id,
                          int rel_ticks,
                          uint32_t max_time_ms = 150,
                          int overshoot_ticks = 2) {
  if (!dxl.ping(id)) return;
  if (rel_ticks == 0) return;

  dxl.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, id, 1);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, 100);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, 30);

  bool tq = dxl.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id);
  if (!tq) dxl.torqueOn(id);

  int start = dxl.getPresentPosition(id);
  int goal  = start + rel_ticks;
  if (rel_ticks > 10)  goal = start + 10;
  if (rel_ticks < -10) goal = start - 10;

  int overshoot_goal = goal + (rel_ticks > 0 ? overshoot_ticks : -overshoot_ticks);
  dxl.setGoalPosition(id, overshoot_goal);
  delay(overshoot_ticks * 8);
  dxl.setGoalPosition(id, goal);

  const int TOL = 1;
  uint32_t t0 = millis();
  while (millis() - t0 < max_time_ms) {
    int pos = dxl.getPresentPosition(id);
    if (abs(pos - goal) <= TOL) break;
    delay(5);
  }

  int finalPos = dxl.getPresentPosition(id);
  Serial.print("STEP ID ");Serial.print(id);
  Serial.print(" rel=");Serial.print(rel_ticks);
  Serial.print(" goal=");Serial.print(goal);
  Serial.print(" final=");Serial.println(finalPos);
}

// ----------------------------------------------------
// Monitor stalls
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
      ledBlink(id,300);
    }
  }
}

// ----------------------------------------------------
// MOVEY: symmetric vertical move
// ----------------------------------------------------
void moveY(int rel_ticks){
  if(!has_L || !has_ZL || !has_ZR){
    Serial.println("ERR: arm not calibrated (L/Z missing)");
    return;
  }
  if (!dxl.ping(ID_LEFT) || !dxl.ping(ID_RIGHT)) return;

  dxl.torqueOn(ID_LEFT);
  dxl.torqueOn(ID_RIGHT);

  int tL = dxl.getPresentPosition(ID_LEFT);
  int tR = dxl.getPresentPosition(ID_RIGHT);
  int gL = tL + rel_ticks;
  int gR = tR - rel_ticks;

  dxl.setGoalPosition(ID_LEFT, gL);
  dxl.setGoalPosition(ID_RIGHT, gR);
  delay(10);
}

// ----------------------------------------------------
// MOVEX: differential motion for small lateral adjust
// ----------------------------------------------------
void moveX(int rel_ticks){
  if(!has_L || !has_ZL || !has_ZR){
    Serial.println("ERR: arm not calibrated (L/Z missing)");
    return;
  }
  if (!dxl.ping(ID_LEFT) || !dxl.ping(ID_RIGHT)) return;

  dxl.torqueOn(ID_LEFT);
  dxl.torqueOn(ID_RIGHT);

  int tL = dxl.getPresentPosition(ID_LEFT);
  int tR = dxl.getPresentPosition(ID_RIGHT);
  int gL = tL + rel_ticks;
  int gR = tR + rel_ticks;

  dxl.setGoalPosition(ID_LEFT, gL);
  dxl.setGoalPosition(ID_RIGHT, gR);
  delay(10);
}

// ----------------------------------------------------
// Print current status
// ----------------------------------------------------
void print_status(const uint8_t* ids,uint8_t count){
  Serial.print("STATUS ");
  for(uint8_t i=0;i<count;i++){
    uint8_t id=ids[i];
    if(!dxl.ping(id)){ Serial.print(id);Serial.print(" X X X"); continue; }
    int pos = dxl.getPresentPosition(id);
    Serial.print(id);Serial.print(':');Serial.print(pos);
    if(i+1<count)Serial.print(' ');
  }
  Serial.println();
}

// ----------------------------------------------------
void setup(){
  Serial.begin(115200);
  while(!Serial);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(PROTOCOL);

  Serial.println("=== OpenRB Arm Motion Controller (with calibration safety) ===");

  for(uint8_t i=0;i<SERVO_COUNT;i++){
    uint8_t id=servo_ids[i];
    if(dxl.ping(id)){
      dxl.torqueOff(id);
      dxl.setOperatingMode(id,OP_POSITION);
      dxl.torqueOn(id);
      stalled_flags[i]=false;
      ledOff(id);
      Serial.print("Servo ");Serial.print(id);Serial.println(" OK");
    } else {
      Serial.print("Servo ");Serial.print(id);Serial.println(" not found!");
    }
  }

  Serial.println("Supported Commands:");
  Serial.println("  MOVEY <ticks>         (vertical, symmetric)");
  Serial.println("  MOVEX <ticks>         (lateral adjust, same direction)");
  Serial.println("  STEP  <id> <rel> [ms] [overshoot]");
  Serial.println("  SETL  <mm>            (set arm length)");
  Serial.println("  SETZ  <id> <ticks>    (set zero tick for id)");
  Serial.println("  READ                  (show current ticks)");
}

// ----------------------------------------------------
void loop(){
  if(!Serial.available()) return;
  String line=Serial.readStringUntil('\n');
  line.trim(); if(line.length()==0) return;

  if(line.startsWith("MOVEY")){
    int val = line.substring(6).toInt();
    moveY(val);
  }
  else if(line.startsWith("MOVEX")){
    int val = line.substring(6).toInt();
    moveX(val);
  }
  else if(line.startsWith("STEP")){
    int id=0,rel=0,ms=150,ov=2;
    sscanf(line.c_str(),"STEP %d %d %d %d",&id,&rel,&ms,&ov);
    smallSingleServoMove(id,rel,ms,ov);
  }
  else if(line.startsWith("SETL")){
    arm_length_mm = line.substring(5).toFloat();
    has_L = arm_length_mm > 0;
    Serial.print("Arm length set to ");Serial.print(arm_length_mm);Serial.println(" mm");
  }
  else if(line.startsWith("SETZ")){
    int id=0,ticks=0;
    sscanf(line.c_str(),"SETZ %d %d",&id,&ticks);
    if(id==ID_LEFT){ tick_zero_left=ticks; has_ZL=true; }
    else if(id==ID_RIGHT){ tick_zero_right=ticks; has_ZR=true; }
    Serial.print("Zero tick for ");Serial.print(id);Serial.print(" = ");Serial.println(ticks);
  }
  else if(line.startsWith("READ")){
    print_status(servo_ids,SERVO_COUNT);
  }
  else {
    Serial.println("ERR CMD");
  }
}
