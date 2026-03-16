#pragma once
#include <stdbool.h>

// The magnetometer tells us which direction is (magnetic) north.
// We then apply a "declination" correction to get TRUE north —
// the direction toward the geographic north pole, which is what
// we actually need to aim at for polar alignment.
//
// Magnetic north and true north are different! In Pécs the difference
// is about 4.5 degrees. Ignoring this would point you 4.5° off.

typedef struct {
    float heading_mag;   // raw compass heading, degrees (0-360)
    float heading_true;  // corrected for declination — use this one
    float x_uT;          // raw X field in microtesla (for debugging)
    float y_uT;          // raw Y field in microtesla
} MagData;

// Call once at startup
void mag_begin();

// Get the latest reading
MagData mag_read();

// Run the calibration routine — see mag.cpp for explanation
// Call this once before your first observing session
void mag_calibrate();
