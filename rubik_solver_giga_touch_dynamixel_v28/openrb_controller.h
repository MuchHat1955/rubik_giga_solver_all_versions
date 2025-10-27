#include "openrb_controller.h"

OpenRBController openrb(Serial2, 115200);

void setup() {
  Serial.begin(115200);
  openrb.begin();

  Serial.println("=== Giga connected to OpenRB ===");

  // Move and cache final XY
  XYStatus xy;
  if (openrb.moveToYSync(80.0, &xy))
    Serial.printf("Y=%.2f X=%.2f  ticks=(%d,%d)\n", xy.y_mm, xy.x_mm, xy.a1_ticks, xy.a2_ticks);

  // Move one servo and cache
  openrb.moveServoSync(11, 2100);

  // Fetch cached XY
  XYStatus cachedXY = openrb.getLastXY();
  if (cachedXY.valid)
    Serial.printf("Cached XY: X=%.2f Y=%.2f\n", cachedXY.x_mm, cachedXY.y_mm);

  // Fetch cached servo position
  ServoStatus s;
  if (openrb.getLastServo(11, s))
    Serial.printf("Cached servo %d pos=%d\n", s.id, s.pos);

  // Re-read full servo list
  ServoStatus table[10];
  if (openrb.readServosSync(table, 10)) {
    Serial.println("Current servo table:");
    for (int i = 0; i < 6; i++)
      if (table[i].valid)
        Serial.printf("ID=%d pos=%d\n", table[i].id, table[i].pos);
  }
}
