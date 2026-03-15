#pragma once

// =============================================================================
//  sensors/imu.h  —  dual BNO085 driver
//
//  Two BNO085 sensors on I2C bus 0:
//    MOUNT  (0x4A, PS1=0) — fixed to the tracker body / polar axis
//    CAMERA (0x4B, PS1=1) — mounted on the camera plate
//
//  The BNO085 runs its own internal AHRS and outputs rotation vectors
//  (quaternions) directly. We do NOT implement a Madgwick/Mahony filter —
//  the chip does that for us.
//
//  The key output is imu_get_tracking_error_arcsec():
//    It computes the differential quaternion q_err = q_mount⁻¹ ⊗ q_camera
//    and converts it to an angular error in arcseconds.
//    This feeds the closed-loop correction in hal/gptimer.h.
// =============================================================================

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    // Quaternion components (unit quaternion, w² + x² + y² + z² = 1)
    float w, x, y, z;

    // Euler angles derived from quaternion — for display and debugging only.
    // Do NOT use these for control — use the quaternion directly.
    float yaw_deg;      // rotation about vertical axis (heading)
    float pitch_deg;    // nose up/down
    float roll_deg;     // bank left/right

    // Calibration status (0=uncal, 1=low, 2=medium, 3=fully calibrated)
    uint8_t cal_status;

    bool valid;         // false until first sample received
} imu_data_t;

// Initialise both sensors and start the IMU task.
// Must be called after i2c_hal_init(I2C_BUS_SENSORS).
void imu_init(void);

// Get latest orientation from each sensor.
// Both are safe to call from any task.
imu_data_t imu_get_mount(void);
imu_data_t imu_get_camera(void);

// Returns the angular error between mount and camera axes in arcseconds.
// Positive = camera is ahead of mount (step rate too slow).
// Negative = camera is behind mount (step rate too fast).
// Returns 0.0 if either sensor is not yet calibrated.
double imu_get_tracking_error_arcsec(void);

// Returns the mount tilt from horizontal in degrees.
// Used during alignment to verify the polar axis is level before slewing.
double imu_get_tilt_deg(void);

// True when both sensors report cal_status >= 2 on all axes.
bool imu_is_calibrated(void);
