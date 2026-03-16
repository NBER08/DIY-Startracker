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
    double altitude_deg;    // how high above horizon to point
    double azimuth_deg;     // compass bearing (0=north, 90=east, 180=south...)
} PoleDirection;

// Calculate where to point the polar axis.
// lat, lon: from GPS (decimal degrees, north/east positive)
// unix_sec: current UTC time from GPS
PoleDirection astro_get_pole(double lat, double lon, long unix_sec);

// How far is the current pointing from the target? (degrees, 0=perfect)
// Uses azimuth and altitude from mag/IMU vs the target from astro_get_pole()
float astro_total_error_deg(PoleDirection target,
                            float current_az, float current_alt);
