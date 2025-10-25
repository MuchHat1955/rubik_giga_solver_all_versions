#include <Dynamixel2Arduino.h>
#define DXL_SERIAL Serial1
#define DXL_DIR_PIN -1
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  Serial.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  uint8_t id = 14;
  int goal = 1656;
  bool movePlus = true;

  delay(2222);
  Serial.println();
  Serial.println();
  Serial.println("~~~~ start setup ~~~~");
  int pos = dxl.getPresentPosition(id);

  Serial.print("current pos ");
  Serial.println(pos);
  if (pos > goal) movePlus = false;

  Serial.print("goal is ");
  Serial.println(goal);
  dxl.ledOn(id);
  int offset = 10;
  if (movePlus and goal < pos + 10) offset = 5;
  if (!movePlus and goal > pos - 10) offset = 5;
  int updatedGoal = goal + offset;
  if (!movePlus) updatedGoal = goal - offset;
  Serial.print("updated goal is ");
  Serial.println(updatedGoal);
  dxl.torqueOn(id);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.setGoalPosition(id, updatedGoal);  //use default encoder value OP_POSITION
  dxl.setGoalVelocity(id, 15.0, UNIT_PERCENT);
    dxl.setGoalVelocity(id, 15.0, UNIT_PERCENT);
  int totalDelay = 0;
  for (int i = 0; i < 65; i++) {
    delay(30);
    totalDelay += 20;
    pos = dxl.getPresentPosition(id);
    if (movePlus && pos >= goal + 1) break;
    if (!movePlus && pos <= goal - 1) break;
  }
  dxl.torqueOff(id);
  dxl.ledOff(id);
  Serial.print("delay ");
  Serial.println(totalDelay);

  delay(333);
  pos = dxl.getPresentPosition(id);
  Serial.print("final pos ");
  Serial.println(pos);
  Serial.print("~~~~ end setup ~~~~");
}

void loop() {
  delay(5);
}
