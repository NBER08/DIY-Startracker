#pragma once
#include "astro.h"
#include "sensors/mag.h"
#include "sensors/imu.h"

// =============================================================================
//  slew.h — move the tracker to face the celestial pole
//
//  Hardware:
//    PCA9535  — I2C I/O expander (address 0x20)
//               Turns one I2C wire into 16 GPIO output pins
//    TB6612FNG — dual H-bridge motor driver (×2, one per axis)
//               Converts logic-level signals into actual motor current
//
//  Two axes:
//    AZIMUTH  — rotates the tracker left/right to face north
//    ALTITUDE — tilts the tracker up/down to match your latitude
//
//  The slew motors are only used during alignment.
//  Once aligned they stop and the tracking motor takes over.
// =============================================================================

// How close is "close enough" to stop slewing (degrees)
#define SLEW_TOLERANCE_DEG   0.5f

// Slew to the pole using all available sensor feedback.
// Blocks until aligned or timeout.
// Returns true if alignment succeeded within tolerance.
bool slew_to_pole(PoleDirection target, int timeout_sec);

// Move one axis by a small fixed amount — useful for manual nudging
void slew_az_nudge(bool clockwise);
void slew_alt_nudge(bool upward);

// Stop all motors immediately
void slew_stop();

// Call once at startup
void slew_begin();
