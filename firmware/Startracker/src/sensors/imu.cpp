#include "imu.h"
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <math.h>

// Two BNO085 sensors on I2C bus 0.
// The difference between them is just the I2C address,
// set by the PS1 pin on each chip.
#define I2C_SDA  8
#define I2C_SCL  9
#define ADDR_MOUNT   0x4A   // PS1 = LOW  → address 0x4A
#define ADDR_CAMERA  0x4B   // PS1 = HIGH → address 0x4B

static BNO08x imu_mount;
static BNO08x imu_camera;

static ImuData mount_data  = {};
static ImuData camera_data = {};

// -------------------------------------------------------------------------
// Convert a quaternion (w, x, y, z) into Euler angles (pitch, roll, yaw).
//
// A quaternion is how the BNO085 gives us orientation internally.
// It's more mathematically stable than Euler angles, but harder to read.
// We convert to degrees so a human (and our code) can understand it.
// -------------------------------------------------------------------------
static void quaternion_to_euler(float w, float x, float y, float z,
                                 float *pitch, float *roll, float *yaw) {
    // These are standard conversion formulas — you don't need to derive them
    *yaw   = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)) * 180.0f / M_PI;
    *pitch = asin( 2*(w*y - z*x))                     * 180.0f / M_PI;
    *roll  = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))  * 180.0f / M_PI;
}

// -------------------------------------------------------------------------
// Read one sensor and fill an ImuData struct.
// We ask for "rotation vector" reports — the BNO085 does all the
// sensor fusion internally and just hands us a quaternion.
// -------------------------------------------------------------------------
static void read_sensor(BNO08x &sensor, ImuData &out) {
    if (!sensor.getSensorEvent()) return;   // no new data yet

    if (sensor.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
        float w = sensor.getQuatReal();
        float x = sensor.getQuatI();
        float y = sensor.getQuatJ();
        float z = sensor.getQuatK();
        quaternion_to_euler(w, x, y, z, &out.pitch, &out.roll, &out.yaw);
        out.valid = true;
    }
}

// -------------------------------------------------------------------------
// Public functions
// -------------------------------------------------------------------------

void imu_begin() {
    Wire.begin(I2C_SDA, I2C_SCL);

    // Init mount IMU
    if (!imu_mount.begin(ADDR_MOUNT, Wire)) {
        Serial.println("IMU: mount sensor not found! Check wiring.");
    } else {
        // Ask the BNO085 to start sending rotation vectors every 10ms (100Hz)
        imu_mount.enableRotationVector(10);
        Serial.println("IMU: mount sensor OK");
    }

    // Init camera IMU
    if (!imu_camera.begin(ADDR_CAMERA, Wire)) {
        Serial.println("IMU: camera sensor not found! Check wiring.");
    } else {
        imu_camera.enableRotationVector(10);
        Serial.println("IMU: camera sensor OK");
    }
}

ImuData imu_get_mount() {
    read_sensor(imu_mount, mount_data);
    return mount_data;
}

ImuData imu_get_camera() {
    read_sensor(imu_camera, camera_data);
    return camera_data;
}