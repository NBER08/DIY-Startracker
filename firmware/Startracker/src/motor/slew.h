#pragma once

// =============================================================================
//  slew.h — platform azimuth slew and camera altitude motor
//
//  The platform altitude is set by hand — no motor for it.
//  This module controls:
//    1. Platform azimuth motor  — rotates the whole mount to face north
//    2. Camera altitude motor   — tilts only the camera to point at a target
// =============================================================================

// ---- Platform azimuth ----

void slew_begin();

// Rotate the platform toward target_az_deg.
// current_az_deg comes from the magnetometer.
// Moves in small steps and returns — call repeatedly from loop() until done.
// Returns true when within tolerance.
bool slew_az_toward(float target_az_deg, float current_az_deg,
                    float tolerance_deg = 0.5f);

// Stop azimuth motor and de-energise (saves power, holds by friction)
void slew_az_stop();

// ---- Camera altitude motor ----
// Tilts the camera relative to the polar axis.
// angle_deg: polar distance from the pole (0 = pointing at pole, 90 = equator)
// This is a blocking move — it runs to completion before returning.
void camera_tilt_to(float angle_deg);

// Current camera tilt angle (tracked by step count, not a sensor)
float camera_get_tilt_deg();

// Stop camera motor
void camera_tilt_stop();
