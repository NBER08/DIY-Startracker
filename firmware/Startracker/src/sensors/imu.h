#pragma once
#include <stdbool.h>

// All we need from the IMU is:
//   1. Is the platform level? (for alignment)
//   2. How much is the camera drifting? (for tracking correction)

typedef struct {
    float pitch;  // tilt forward/back in degrees
    float roll;   // tilt left/right in degrees
    float yaw;    // rotation around vertical in degrees
    bool  valid;
} ImuData;

// Call once at startup
void imu_begin();

// Get latest orientation from the MOUNT IMU (on the tracker body)
ImuData imu_get_mount();

// Get latest orientation from the CAMERA IMU (on the camera plate)
ImuData imu_get_camera();
