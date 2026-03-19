#include "tof.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

#ifndef SIMULATE_HARDWARE
  #define SIMULATE_HARDWARE 0
#endif

#if !SIMULATE_HARDWARE
#include <Wire.h>
#include <vl53l4cd_class.h>   // from stm32duino/VL53L4CD library

// Pass Wire and the I2C address to the driver.
// The VL53L4CD library takes the I2C bus object and an optional XSHUT pin.
// We don't use XSHUT (-1 = not connected).
static VL53L4CD sensor(&Wire, -1);
#endif

// -------------------------------------------------------------------------
// Distance → altitude angle
//
// atan returns the angle whose tangent is (rise / run).
// rise = how much further the beam travels compared to flat = d - TOF_FLAT_MM
// run  = horizontal distance from sensor to hinge = TOF_ARM_MM
//
// This is exact for the geometry shown in tof.h.
// The sign convention: more distance = larger angle = tilting up.
// -------------------------------------------------------------------------
static float dist_to_altitude(float dist_mm) {
    float rise = dist_mm - TOF_FLAT_MM;
    return atan2f(rise, TOF_ARM_MM) * 180.0f / M_PI;
}

void tof_begin() {
#if SIMULATE_HARDWARE
    Serial.println("ToF: simulated");
#else
    // Wire must already be initialised by imu_begin() before this is called
    sensor.begin();
    sensor.VL53L4CD_Off();

    uint8_t status = sensor.InitSensor(TOF_ADDR << 1);   // library expects 8-bit address
    if (status != VL53L4CD_ERROR_NONE) {
        Serial.printf("ToF: VL53L4CD init failed (status=%d)\n", status);
        Serial.println("ToF: check wiring and I2C address");
        return;
    }

    // Set ranging timing budget — 50 ms gives good accuracy without being slow
    sensor.VL53L4CD_SetRangeTiming(50, 0);
    sensor.VL53L4CD_StartRanging();
    Serial.println("ToF: VL53L4CD ready");
#endif
}

TofReading tof_read() {
#if SIMULATE_HARDWARE
    // Simulate the distance for a 46° platform altitude
    // rise = tan(46°) * TOF_ARM_MM ≈ 1.036 * 80 = 82.9
    // d = TOF_FLAT_MM + rise = 120 + 82.9 = 202.9
    TofReading r = { 202.9f, 46.0f, true };
    return r;
#else
    TofReading r = { 0.0f, 0.0f, false };

    uint8_t data_ready = 0;
    sensor.VL53L4CD_CheckForDataReady(&data_ready);
    if (!data_ready) return r;

    VL53L4CD_Result_t result;
    sensor.VL53L4CD_GetResult(&result);
    sensor.VL53L4CD_ClearInterrupt();

    // Only use the reading if the sensor reports a valid range status
    // Status 0 = valid, anything else = out of range or error
    if (result.range_status != 0) return r;

    r.distance_mm = (float)result.distance_mm;
    r.altitude_deg = dist_to_altitude(r.distance_mm);
    r.valid = true;
    return r;
#endif
}
