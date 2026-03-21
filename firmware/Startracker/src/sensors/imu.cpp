#include "imu.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <math.h>

static BNO08x imu;
static CameraOrientation orientation = {};

// -----------------------------------------------------------------------------
// Convert quaternion → azimuth and altitude
//
// The BNO085 outputs a quaternion representing absolute orientation.
// We convert it to two angles the rest of the code can use:
//   az_deg  = rotation around vertical axis = compass heading = azimuth
//   alt_deg = tilt above horizontal = altitude above horizon
// -----------------------------------------------------------------------------
static void quaternion_to_az_alt(float w, float x, float y, float z, float *az, float *alt) {
    // Yaw -> azimuth
    float yaw = atan2f(2.0f * (w*z + x*y),
                       1.0f - 2.0f * (y*y + z*z)) * 180.0f / M_PI;
    // Pitch -> altitude
    float pitch = asinf(2.0f * (w*x - y*z)) * 180.0f / M_PI;

    //Wrap yaw
    *az  = (yaw < 0.0f) ? yaw + 360.0f : yaw;
    *alt = pitch;
}

// -----------------------------------------------------------------------------
// Angular difference between two azimuth values, accounting for wrap-around.
// e.g. difference between 5° and 355° is 10°, not 350°.
// -----------------------------------------------------------------------------
static float az_diff(float az1, float az2) {
    float diff = az1 - az2;
    if (diff < -180.0f) diff += 360.0f;
    if (diff >  180.0f) diff -= 360.0f;
    return fabsf(diff);
}

void imu_begin() {
    Wire.begin(); // Call first in main.cpp!!!!
    if (!imu.begin(IMU_ADDR, Wire)) {
        Serial.println("IMU: BNO085 not found — check wiring and address");
        return;
    }

    imu.enableRotationVector(10); // 10 Hz
    Serial.println("IMU: BNO085 ready");
}

CameraOrientation imu_get_camera() {
    if (imu.getSensorEvent() && imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
        float az, alt;
        quaternion_to_az_alt(imu.getQuatReal(), imu.getQuatI(), imu.getQuatJ(), imu.getQuatK(), &az, &alt);
        orientation.az_deg = az;
        orientation.alt_deg = alt;
        orientation.valid = true;
    }
    return orientation;
}

bool imu_is_on_target(float target_az_deg, float target_alt_deg, float tolerance_deg){
    if (!orientation.valid) return false;

    float az_error = az_diff(orientation.az_deg, target_az_deg);
    float alt_error = fabsf(orientation.alt_deg - target_alt_deg);

    return (az_error <= tolerance_deg && alt_error <= tolerance_deg);
}