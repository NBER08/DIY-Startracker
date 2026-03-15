#pragma once

// The motor module has one job: spin the polar axis at the sidereal rate.
// Everything else in the system either starts it, stops it, or
// nudges it slightly faster/slower.

// Call once at startup
void motor_begin();

// Start spinning at the sidereal rate (the default)
void motor_start_tracking();

// Stop the motor completely
void motor_stop();

// Nudge the rate up or down by a tiny amount.
// correction_deg: how many degrees the camera has drifted.
//   Positive = camera is ahead → slow down
//   Negative = camera is behind → speed up
void motor_apply_correction(float correction_deg);
