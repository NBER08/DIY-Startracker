#pragma once

// =============================================================================
//  astro/astro_math.h  —  astronomical coordinate calculations
//
//  PURE FUNCTIONS. No hardware, no FreeRTOS, no global state.
//  All inputs passed as arguments, result returned by value or output pointer.
//
//  This entire module can be compiled and tested on a desktop PC:
//    gcc test_astro_math.c astro_math.c -lm -o test_astro
//
//  Verify outputs against Stellarium before trusting them on hardware.
//
//  Coordinate systems used:
//    Equatorial  — RA (right ascension, hours) / Dec (declination, degrees)
//    Horizontal  — Alt (altitude, degrees above horizon) / Az (azimuth, degrees
//                  clockwise from true North)
// =============================================================================

#include <stdint.h>

// Polaris coordinates (J2000.0 epoch)
// These drift slowly — update every ~10 years is sufficient
#define POLARIS_RA_HRS      2.5303      // 2h 31m 49s
#define POLARIS_DEC_DEG     89.2641     // 89° 15' 51"

// =============================================================================
//  Core calculation chain
// =============================================================================

// Convert Unix timestamp (ms) to Julian Date
double astro_unix_ms_to_jd(int64_t unix_ms);

// Greenwich Mean Sidereal Time from Julian Date, in degrees (0–360)
double astro_jd_to_gmst_deg(double jd);

// Local Sidereal Time in degrees
// lon_deg: East is positive (Pécs ≈ +18.23°)
double astro_local_sidereal_time_deg(double gmst_deg, double lon_deg);

// Hour angle of an object, in degrees (-180 to +180)
// Negative = object east of meridian (rising), positive = west (setting)
double astro_hour_angle_deg(double lst_deg, double ra_hrs);

// Convert equatorial coordinates to horizontal (Alt/Az)
// ha_deg:  hour angle in degrees
// dec_deg: declination in degrees
// lat_deg: observer latitude in degrees (North positive)
// alt_out: altitude above horizon in degrees (output)
// az_out:  azimuth from North, clockwise, in degrees (output)
void astro_equatorial_to_horizontal(double ha_deg,  double dec_deg,
                                    double lat_deg,
                                    double *alt_out, double *az_out);

// =============================================================================
//  Top-level convenience function — this is what the state machine calls
// =============================================================================

// Given observer position and time, return where to point the polar axis.
// Output alt/az is the direction of the north celestial pole (corrected
// for Polaris's ~0.74° offset from the true pole).
void astro_get_pole_direction(double lat_deg,   double lon_deg,
                               int64_t unix_ms,
                               double *alt_out,  double *az_out);

// =============================================================================
//  Refraction correction
// =============================================================================

// Atmospheric refraction in arcminutes using Bennett's formula.
// h_deg:        true altitude above horizon in degrees
// pressure_hpa: from BME280
// temp_c:       from BME280
// Returns correction to ADD to apparent altitude to get true altitude.
double astro_refraction_arcmin(double h_deg,
                                double pressure_hpa,
                                double temp_c);

// =============================================================================
//  Sidereal time utilities
// =============================================================================

// Current Earth Rotation Angle (ERA) — used for precise sidereal rate
double astro_era_deg(double jd_ut1);

// Convert degrees to hours-minutes-seconds string (for display/debug)
// buf must be at least 16 bytes
void astro_deg_to_hms(double deg, char *buf, int buf_len);

// Convert degrees to degrees-arcmin-arcsec string (for display/debug)
void astro_deg_to_dms(double deg, char *buf, int buf_len);
