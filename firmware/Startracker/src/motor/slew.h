#pragma once

// =============================================================================
//  motor/slew.h  —  TB6612FNG azimuth + altitude slew motors
//
//  Two TB6612FNG H-bridge ICs driven via a PCA9535 I2C I/O expander (I2C1).
//  Each TB6612FNG has two H-bridge channels; we use one per axis:
//    SLEW_AZ   — azimuth rotation (finding North)
//    SLEW_ALT  — altitude (setting polar elevation = latitude)
//
//  These motors are used only during STATE_SLEWING_TO_POLE.
//  They are slow, position-controlled moves — timing is not critical.
//
//  Stepper sequencing (half-step, 8 states) is done in software by
//  the slew task, which simply writes the PCA9535 output register over I2C.
// =============================================================================

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SLEW_AZ  = 0,   // azimuth axis
    SLEW_ALT = 1,   // altitude axis
} slew_axis_t;

typedef enum {
    SLEW_DIR_POSITIVE = 0,  // AZ: clockwise / ALT: up
    SLEW_DIR_NEGATIVE = 1,  // AZ: counter-clockwise / ALT: down
} slew_dir_t;

// Initialise I2C expander and both TB6612FNG chips.
// Must be called after i2c_hal_init(I2C_BUS_EXPANDER).
void slew_init(void);

// Move one axis a given number of steps in a given direction.
// Blocking — does not return until the move is complete.
// step_delay_ms: time between motor steps (lower = faster, min ~5ms)
void slew_move_steps(slew_axis_t axis, slew_dir_t dir,
                     uint32_t steps, uint32_t step_delay_ms);

// Move one axis to reach a target angle.
// current_deg:  from IMU tilt (ALT) or magnetometer heading (AZ)
// target_deg:   from astro_get_pole_direction()
// Returns true when within tolerance_deg of target.
bool slew_move_to_angle(slew_axis_t axis,
                        double target_deg,
                        double current_deg,
                        double tolerance_deg);

// Emergency stop — de-energise both motors immediately.
// Call from fault handler.
void slew_stop_all(void);

// De-energise a motor to save power after a move is complete.
// The TB6612FNG will hold position passively via mechanical friction.
void slew_release(slew_axis_t axis);
