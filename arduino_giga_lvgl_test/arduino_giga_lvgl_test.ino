#include "Arduino_H7_Video.h"
#include "lvgl.h"
#include "Arduino_GigaDisplayTouch.h"

Arduino_H7_Video         Display(800, 480, GigaDisplayShield);
Arduino_GigaDisplayTouch Touch;

static lv_obj_t * status_label;

// ======================================================
//  Event callback for menu buttons
// ======================================================
static void btn_event_cb(lv_event_t * e) {
  lv_obj_t * btn  = (lv_obj_t *)lv_event_get_target(e);
  const char * name = (const char *)lv_event_get_user_data(e);
  lv_label_set_text_fmt(status_label, "%s pressed!", name);
}

// ======================================================
//  Setup
// ======================================================
void setup() {
  Display.begin();
  Touch.begin();

  // ---------- Dark background ----------
  lv_color_t bg_color   = lv_color_hex(0x101215);
  lv_color_t text_color = lv_color_hex(0xE0E8FF);
  lv_obj_set_style_bg_color(lv_scr_act(), bg_color, 0);
  lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);

  // ---------- Button and label styles ----------
  static lv_style_t style_btn;
  lv_style_init(&style_btn);
  lv_style_set_bg_color(&style_btn, lv_color_hex(0x223344));
  lv_style_set_border_width(&style_btn, 0);
  lv_style_set_radius(&style_btn, 10);
  lv_style_set_text_font(&style_btn, &lv_font_montserrat_26);
  lv_style_set_bg_opa(&style_btn, LV_OPA_COVER);

  static lv_style_t style_label;
  lv_style_init(&style_label);
  lv_style_set_text_color(&style_label, text_color);
  lv_style_set_text_font(&style_label, &lv_font_montserrat_26);

  // ---------- Title ----------
  lv_obj_t * title = lv_label_create(lv_scr_act());
  lv_label_set_text(title, "Rubik Solver - Touch Menu");
  lv_obj_add_style(title, &style_label, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

  // ---------- Button labels with LVGL symbols ----------
  const char * names[] = {
    LV_SYMBOL_PLAY      "  Solve Cube",
    LV_SYMBOL_IMAGE     "  Read Colors",
    LV_SYMBOL_REFRESH   "  Random Cube",
    LV_SYMBOL_LIST      "  Tests"
  };

  // ---------- Directly place buttons on screen ----------
  int startY = 80;             // starting vertical position
  int spacing = 80;            // distance between buttons

  for (int i = 0; i < 4; i++) {
    lv_obj_t * btn = lv_btn_create(lv_scr_act());
    lv_obj_add_style(btn, &style_btn, 0);
    lv_obj_set_size(btn, 640, 70);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, startY + i * spacing);

    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, names[i]);
    lv_obj_add_style(label, &style_label, 0);
    lv_obj_center(label);

    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, (void *)names[i]);
  }

  // ---------- Status label ----------
  status_label = lv_label_create(lv_scr_act());
  lv_label_set_text(status_label, "Tap any button...");
  lv_obj_add_style(status_label, &style_label, 0);
  lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID, 0, -10);
}

// ======================================================
//  Loop
// ======================================================
void loop() {
  lv_timer_handler();
  delay(5);
}
