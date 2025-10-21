// color_sensor.h
#pragma once
char readColorSensor();
char readColorSmoothed(int samples=5, int delayMs=40);
void color_sensor_begin();
