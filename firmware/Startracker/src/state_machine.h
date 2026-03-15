#pragma once

// =============================================================================
//  state_machine.h  —  system state machine
//
//  This is the only place that owns mutable system state.
//  All other modules expose read-only getters and action functions.
//  The state machine reads sensor data, decides what to do, and calls
//  those action functions.
//
//  State transition diagram:
//
//    BOOT
//      └─→ WAITING_FOR_GPS      (after hardware init completes)
//            └─→ MAG_CALIBRATE  (after valid 3D GPS fix)
//                  └─→ SLEWING_TO_POLE   (after mag cal complete or skipped)
//                        └─→ TRACKING    (after slew within tolerance)
//                              └─→ (stays here until stopped or fault)
//
//    Any state ──→ ERROR         (on hardware fault or CMD_STOP)
//    ERROR    ──→ BOOT           (on CMD_REALIGN or power cycle)
// =============================================================================

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
//  States
// =============================================================================

typedef enum {
    STATE_BOOT             = 0,  // initialising hardware, not yet ready
    STATE_WAITING_FOR_GPS  = 1,  // waiting for valid 3D GPS fix
    STATE_MAG_CALIBRATE    = 2,  // running hard/soft iron calibration routine
    STATE_SLEWING_TO_POLE  = 3,  // slew motors moving to computed pole direction
    STATE_TRACKING         = 4,  // sidereal ISR running, closed-loop IMU active
    STATE_ERROR            = 5,  // fault — motor, sensor, or power failure
} system_state_t;

// =============================================================================
//  System status struct — a complete snapshot of the system at any moment.
//  Read-only outside state_machine.c. Updated every tick.
// =============================================================================

typedef struct {
    system_state_t  state;

    // GPS
    double          lat_deg;
    double          lon_deg;
    int64_t         utc_ms;
    uint8_t         gps_satellites;
    bool            gps_valid;

    // Computed pole direction (updated when GPS becomes valid)
    double          pole_alt_deg;
    double          pole_az_deg;

    // Current motor orientation (from IMU tilt + mag heading)
    double          mount_alt_deg;
    double          mount_az_deg;

    // Tracking quality
    double          tracking_error_arcsec;   // from dual IMU differential
    uint32_t        steps_since_start;       // from GPTimer

    // Environment
    float           temperature_c;
    float           pressure_hpa;
    float           humidity_rh;
    bool            dew_warning;

    // Power
    float           motor_rail_mv;
    float           motor_current_ma;
    bool            battery_low;
    bool            motor_jammed;

    // Last LoRa
    int16_t         last_rssi_dbm;

    // Error info
    const char     *error_reason;   // human-readable, set on entering ERROR state
} system_status_t;

// =============================================================================
//  Camera control  (triggered by LoRa commands or direct API)
// =============================================================================

typedef struct {
    uint32_t focus_delay_ms;    // how long to hold focus before firing shutter
    uint32_t exposure_ms;       // shutter open duration (0 = single shot mode)
    uint16_t burst_count;       // number of frames for CMD_SHUTTER_BURST
    uint32_t interval_ms;       // interval between burst frames
} camera_params_t;

// =============================================================================
//  API
// =============================================================================

// Initialise all hardware and create all FreeRTOS tasks.
// Call once from app_main() on Core 0.
void state_machine_init(void);

// Get a snapshot of current system status.
// Safe to call from any task — protected by mutex internally.
system_status_t state_machine_get_status(void);

// Get just the current state (cheaper than full status snapshot).
system_state_t state_machine_get_state(void);

// Trigger an emergency stop and enter ERROR state.
// reason: short string stored in status.error_reason.
void state_machine_fault(const char *reason);

// Camera control — called by LoRa command handler.
void state_machine_trigger_camera(const camera_params_t *params);

// Skip magnetometer calibration and use last saved cal data.
// Only valid if mag_load_calibration() returned true.
void state_machine_skip_mag_cal(void);
