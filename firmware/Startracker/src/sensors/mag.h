#pragma once
#include <stdbool.h>

typedef struct {
    float heading_mag;   // raw compass heading, degrees (0-360)
    float heading_true;  // corrected for declination
    float x_uT;          // raw X field in microtesla
    float y_uT;          // raw Y field in microtesla
    bool valid;
} MagData;

void mag_begin();
MagData mag_read();
void mag_calibrate();