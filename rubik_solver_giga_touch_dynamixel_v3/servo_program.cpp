#include "servos.h"
#include "ui_touch.h"
#include <Dynamixel2Arduino.h>
#include <vector>

void runServoProgrammingWizard() {
  lv_obj_clean(lv_scr_act());
  lv_obj_t *lbl = lv_label_create(lv_scr_act());
  lv_label_set_text(lbl, "Searching for servo ID 1...");
  lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 10);

  // Try to ping ID 1 (factory default)
  int old_id = 1;
  if (dxl.ping(old_id)) {
    lv_label_set_text(lbl, "Found servo ID 1. Assigning new IDs...");
    delay(500);

    int new_ids[] = {11, 12, 13, 14, 15, 16};
    for (int i = 0; i < 6; i++) {
      dxl.torqueOff(old_id);
      dxl.setID(old_id, new_ids[i]);
      dxl.torqueOn(new_ids[i]);
      dxl.ledOn(new_ids[i]);
      delay(300);
      dxl.ledOff(new_ids[i]);
    }
    lv_label_set_text(lbl, "Programming complete!");
  } else {
    lv_label_set_text(lbl, "No servo found at ID 1.");
  }
  lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);
  delay(1500);
  ui_show_main_menu();
}
