#pragma once
#include <stdint.h>

// The motor module has one job: spin the polar axis at the sidereal rate.
// Everything else in the system either starts it, stops it, or
// nudges it slightly faster/slower.

void motor_begin();
void motor_start_tracking();
void motor_stop();
uint64_t motor_get_step_count();   // added so main.cpp can report steps

// Slew the polar axis by a signed number of steps at a fast rate.
// Positive = forward (east), negative = backward (west).
// Blocking — used during camera pointing before resuming tracking.
void motor_slew_steps(int64_t steps);
