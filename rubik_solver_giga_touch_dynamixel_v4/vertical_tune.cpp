#include "vertical_tune.h"
#include "group_motion.h"
#include "servos.h"
#include "param_store.h"
#include "ui_touch.h"
#include "action_store.h"
#include <cmath>
#include <vector>
#include <string>

// Estimated geometry constants (will be measured)
float arm1_length_mm = 48.0f;
float arm2_length_mm = 48.0f;
float base_x_offset_mm = 20.0f;

// ------------------------------------------------------
// Helper: move arms stepwise and measure compensation
// ------------------------------------------------------
static void measure_vertical_line() {
  // Move to mid pose
  actionExecute("arms_row1");

  // Move up stepwise for observation
  for (int i = 0; i <= 20; i += 5) {
    moveArmsVertically((float)i);
    delay(500);
  }
  // Move back to base
  actionExecute("arms_row1");
}

// ------------------------------------------------------
// Wizard UI
// ------------------------------------------------------
void runVerticalTuningUI() {
  lv_obj_clean(lv_scr_act());

  lv_obj_t *lbl = lv_label_create(lv_scr_act());
  lv_label_set_text(lbl, "Running Vertical Tune...");
  lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 10);

  // Phase 1: move stepwise for observation
  measure_vertical_line();

  // Phase 2: compute and save parameters
  // (for now, we use fixed constants; future: use sensor feedback)
  arm1_length_mm = 48.0f;
  arm2_length_mm = 48.0f;
  base_x_offset_mm = 20.0f;

  // Save to param store for persistence
  setParamValue("arm1_length_mm", (int)(arm1_length_mm * 100));
  setParamValue("arm2_length_mm", (int)(arm2_length_mm * 100));
  setParamValue("base_x_offset_mm", (int)(base_x_offset_mm * 100));

  lv_label_set_text(lbl, "Vertical Tune Complete.\nResults saved.");
  lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);

  delay(1500);
  ui_show_main_menu();
}
