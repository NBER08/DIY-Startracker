#include "imu.h"
#include "config.h"
#include <Arduino.h>

#ifndef SIMULATE_HARDWARE
  #define SIMULATE_HARDWARE 0
#endif

#if !SIMULATE_HARDWARE
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <math.h>

static BNO08x            sensor;
static CameraOrientation data       = {};
static float             yaw_ref    = 0.0f;   // yaw at the moment tracking started
static bool              has_ref    = false;

static void quaternion_to_az_alt(float w, float x, float y, float z,
                                  float *az, float *alt, float *yaw_out) {
    // Yaw = rotation around vertical axis = azimuth
    float raw_yaw = atan2f(2.0f*(w*z + x*y),
                            1.0f - 2.0f*(y*y + z*z)) * 180.0f / M_PI;
    // Pitch = rotation around side axis = altitude above horizon
    float raw_pitch = asinf(2.0f*(w*y - z*x)) * 180.0f / M_PI;

    // Convert yaw to 0–360 compass bearing
    *az      = (raw_yaw < 0.0f) ? raw_yaw + 360.0f : raw_yaw;
    *alt     = raw_pitch;
    *yaw_out = raw_yaw;
}
#endif

static float yaw_ref_val = 0.0f;
static bool  has_ref_val = false;

void imu_begin() {
#if SIMULATE_HARDWARE
    Serial.println("IMU: simulated");
#else
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!sensor.begin(IMU_ADDR_CAMERA, Wire)) {
        Serial.println("IMU: camera BNO085 not found!");
    } else {
        sensor.enableRotationVector(10);   // 100 Hz
        Serial.println("IMU: camera OK");
    }
#endif
}

CameraOrientation imu_get_camera() {
#if SIMULATE_HARDWARE
    CameraOrientation d = { 180.0f, 45.0f, 180.0f, true };
    return d;
#else
    if (sensor.getSensorEvent() &&
        sensor.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
        float az, alt, yaw;
        quaternion_to_az_alt(
            sensor.getQuatReal(), sensor.getQuatI(),
            sensor.getQuatJ(),    sensor.getQuatK(),
            &az, &alt, &yaw
        );
        data.az_deg  = az;
        data.alt_deg = alt;
        data.yaw_deg = yaw;
        data.valid   = true;
    }
    return data;
#endif
}

void imu_reset_tracking_reference() {
#if !SIMULATE_HARDWARE
    CameraOrientation cur = imu_get_camera();
    if (cur.valid) {
        yaw_ref_val = cur.yaw_deg;
        has_ref_val = true;
    }
#endif
}

float imu_get_tracking_error_deg() {
#if SIMULATE_HARDWARE
    return 0.0f;
#else
    if (!has_ref_val || !data.valid) return 0.0f;
    float diff = data.yaw_deg - yaw_ref_val;
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
#endif
}
