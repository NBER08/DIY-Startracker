#pragma once
#include <stdbool.h>

typedef struct {
    float az_deg;    // camera azimuth  0–360°, clockwise from North
    float alt_deg;   // camera altitude 0–90° above horizon
    bool  valid;     // false until first reading arrives
} CameraOrientation;

void imu_begin();
CameraOrientation imu_get_camera();
bool imu_is_on_target(float target_az_deg, float target_alt_deg, float tolerance_deg);