#pragma once

// =============================================================================
//  sensors/mag.h  —  IIS2MDCTR magnetometer driver
//
//  Used for finding magnetic north during the ALIGN phase.
//  Heading is corrected for:
//    1. Hard and soft iron distortion (calibration ellipsoid, stored in NVS)
//    2. Magnetic declination at your GPS location
//    3. Board tilt (using the mount BNO085 quaternion for tilt compensation)
//
//  IMPORTANT: Hard/soft iron calibration must be performed after final
//  assembly with all hardware powered on (motors energised). The calibration
//  routine rotates the sensor through all orientations and fits an ellipsoid.
//  Re-calibrate if you add or remove hardware near the sensor.
// =============================================================================

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float x_uT;         // raw magnetic field, X axis, µTesla
    float y_uT;         // raw magnetic field, Y axis
    float z_uT;         // raw magnetic field, Z axis

    float heading_mag_deg;      // magnetic heading 0–360°, uncorrected
    float heading_true_deg;     // heading corrected for declination + tilt
    float declination_deg;      // applied magnetic declination (from GPS)

    bool  calibrated;   // false until cal data loaded from NVS
    bool  valid;        // false until first sample
} mag_data_t;

// Initialise the IIS2MDCTR and start the magnetometer task.
// Must be called after i2c_hal_init(I2C_BUS_SENSORS).
void mag_init(void);

// Get latest magnetometer reading.
// heading_true_deg is ready only when calibrated=true and gps_is_ready().
mag_data_t mag_get(void);

// Blocking calibration routine. Call from the MAG_CALIBRATE state.
// Prompts the user (via LoRa or serial) to slowly rotate the tracker
// through all orientations. Takes ~60 seconds. Saves result to NVS.
// Returns true on success, false if insufficient motion detected.
bool mag_run_calibration(void);

// Load calibration from NVS. Returns true if valid cal data exists.
// Called automatically by mag_init() — you rarely need to call this directly.
bool mag_load_calibration(void);

// Clear stored calibration from NVS (forces re-calibration on next boot).
void mag_clear_calibration(void);

// Apply GPS-derived magnetic declination correction.
// Call this once after first GPS fix. Declination is calculated internally
// from the GPS coordinates using the WMM2020 simplified model.
void mag_set_declination_from_gps(double lat_deg, double lon_deg, int year);
