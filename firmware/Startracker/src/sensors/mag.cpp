#include "mag.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <IIS2MDCSensor.h>
#include <math.h>

// Look up your exact value at: https://www.magnetic-declination.com
#define DECLINATION_DEG  5.38f

// Set to 0 initially. Run mag_calibrate() once to measure real values
static float cal_offset_x = 0.0f;
static float cal_offset_y = 0.0f;

static IIS2MDCSensor mag(&Wire);
static MagData data = {};

void mag_begin() {
    // Wire must already be initialised before this is called
    mag.begin();
    mag.Enable();
    Serial.println("IIS2MDCTR ready");
}

MagData mag_read() {
    int32_t raw[3];
    mag.GetAxes(raw);
 
    // Convert milligauss → microtesla (1 mGauss = 0.1 µT)
    float x = raw[0] * 0.1f;
    float y = raw[1] * 0.1f;
 
    // Apply hard iron correction
    x -= cal_offset_x;
    y -= cal_offset_y;
 
    // Compute heading — atan2 returns -180 to +180
    float heading = atan2f(y, x) * 180.0f / M_PI;
 
    // Shift to 0–360
    if (heading < 0.0f) heading += 360.0f;
 
    // Apply declination to get true north
    float true_heading = heading + DECLINATION_DEG;
    if (true_heading >= 360.0f) true_heading -= 360.0f;
    if (true_heading <    0.0f) true_heading += 360.0f;
 
    data.heading_mag  = heading;
    data.heading_true = true_heading;
    data.x_uT         = x;
    data.y_uT         = y;
    data.valid         = true;
 
    return data;
}

//   Temporarily call mag_calibrate() at the end of setup().
//   Flash the board. Open serial monitor.
//   Slowly rotate the whole tracker through a full horizontal circle,
//   taking at least 20 seconds.
//   Copy the two printed numbers into cal_offset_x and cal_offset_y above.
//   Remove the mag_calibrate() call and reflash.
void mag_calibrate() {
    Serial.println("Mag cal: slowly rotate tracker 360°...");

    float xmin =  1e9f, xmax = -1e9f;
    float ymin =  1e9f, ymax = -1e9f;

    unsigned long end = millis() + 20000;
    while (millis() < end) {
        int32_t raw[3];
        mag.GetAxes(raw);
        float x = raw[0] * 0.1f;
        float y = raw[1] * 0.1f;
        if (x < xmin) xmin = x;  if (x > xmax) xmax = x;
        if (y < ymin) ymin = y;  if (y > ymax) ymax = y;
        delay(50);
    }

    Serial.println("Done. Paste these into mag.cpp:");
    Serial.printf("  cal_offset_x = %.2ff;\n", (xmax + xmin) / 2.0f);
    Serial.printf("  cal_offset_y = %.2ff;\n", (ymax + ymin) / 2.0f);
}