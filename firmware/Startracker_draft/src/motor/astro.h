#pragma once

// =============================================================================
//  Given your GPS position and the current UTC time, this module
//  calculates the altitude and azimuth of the north celestial pole.
//
//  "Altitude" = how high above the horizon (degrees)
//  "Azimuth"  = compass bearing from true north, clockwise (degrees)
//
//  For a perfectly polar-aligned tracker:
//    altitude ≈ your latitude        (in Pécs: ~46°)
//    azimuth  ≈ 0° (due north)       (in practice: within ~1° of north)
// =============================================================================

typedef struct {
    double pole_alt_deg;      // altitude the platform altitude axis must reach (set manually)
    double pole_az_deg;       // azimuth the platform must rotate to (motor controlled)
    double alt_correction_deg; // how many degrees the user must tilt the platform manually
                               // positive = tilt up, negative = tilt down
} PoleDirection;

typedef struct {
    double ha_deg;            // hour angle — how far to rotate the polar axis from home
    double polar_dist_deg;    // polar distance — how far to tilt the camera from polar axis
                               // 0 = camera points at pole, 90 = camera points at equator
} CameraTarget;

// Calculate where to point the polar axis.
// lat, lon: from GPS (decimal degrees, north/east positive)
// unix_sec: current UTC time from GPS
PoleDirection astro_get_pole(double lat, double lon, long unix_sec);

// Convert a target RA/Dec into the motor angles needed to point the camera at it
// ra_hours: right ascension in hours (0–24)
// dec_deg:  declination in degrees (-90 to +90)
CameraTarget astro_radec_to_motors(double lat_deg, double lon_deg, long unix_sec,
                                    double ra_hours, double dec_deg);

// Current local sidereal time in degrees — useful for display
double astro_lst_deg(double lon_deg, long unix_sec);