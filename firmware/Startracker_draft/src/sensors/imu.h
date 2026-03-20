#pragma once
#include <stdbool.h>

// =============================================================================
//  imu.h — single BNO085 mounted on the camera
//
//  Used for two things:
//    1. Camera pointing — tells us where the camera is currently aimed (Az/Alt)
//    2. Tracking error  — yaw drift while tracking
//
//  The platform altitude is no longer measured here.
//  That is now handled by tof.h (VL53L4CD distance sensor).
// =============================================================================

typedef struct {
    float az_deg;    // camera azimuth  0–360°, clockwise from North
    float alt_deg;   // camera altitude 0–90° above horizon
    float yaw_deg;   // raw yaw (same as az_deg, kept for tracking error calc)
    bool  valid;
} CameraOrientation;

void               imu_begin();
CameraOrientation  imu_get_camera();

// Tracking drift: how far the camera yaw has moved since tracking started.
// Returns 0.0 while not tracking. Call imu_reset_tracking_reference()
// when tracking starts so drift is measured from that moment.
void  imu_reset_tracking_reference();
