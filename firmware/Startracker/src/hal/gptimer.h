#pragma once

// =============================================================================
//  hal/gptimer.h  —  hardware timer for sidereal step pulse generation
//
//  This is the timing-critical core of the tracker. The timer fires a
//  hardware interrupt that pulses the TMC2209 STEP pin at the sidereal rate.
//
//  Rules:
//    - sidereal_timer_init() must be called from a task pinned to Core 1
//    - The ISR itself is IRAM_ATTR and never touches FreeRTOS primitives
//    - All other tasks run on Core 0
// =============================================================================

#include <stdint.h>
#include "esp_err.h"

// Tracking rate presets
typedef enum {
    TRACK_RATE_SIDEREAL = 0,    // stars  — 86164.0905 s/rev
    TRACK_RATE_LUNAR,           // Moon   — 89309.4 s/rev
    TRACK_RATE_SOLAR,           // Sun    — 86400.0 s/rev
    TRACK_RATE_STOPPED,         // motor holds position, no stepping
} track_rate_t;

// Initialise GPTimer and register the ISR.
// Does NOT start the timer — call sidereal_timer_start() from the TRACK state.
esp_err_t sidereal_timer_init(void);

// Start/stop the timer. Safe to call from any FreeRTOS task.
void sidereal_timer_start(void);
void sidereal_timer_stop(void);

// Change step rate without stopping the timer.
// Takes effect on the next alarm cycle.
void sidereal_timer_set_rate(track_rate_t rate);

// Apply a fine correction in parts-per-million to the current rate.
// Used by the closed-loop IMU feedback path.
// +1 ppm → slightly faster, -1 ppm → slightly slower.
void sidereal_timer_apply_ppm_correction(int32_t ppm);

// Total step count since last start — for position tracking.
uint64_t sidereal_timer_get_step_count(void);

// Reset step counter to zero (call when entering TRACK state).
void sidereal_timer_reset_count(void);
