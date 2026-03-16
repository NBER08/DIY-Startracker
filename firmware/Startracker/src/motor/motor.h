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

