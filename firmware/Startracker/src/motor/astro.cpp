#include "astro.h"
#include <math.h>

// =============================================================================
//  Astronomical coordinate math
//
//  This file is pure math. No Wire, no Serial, no Arduino.h.
//  You could compile and test it on a PC with just gcc.
//
//  THE CHAIN OF CALCULATIONS:
//
//  UTC time
//    → Julian Date        (a single number counting days since 4713 BC)
//    → Sidereal Time      (which part of the sky is overhead right now)
//    → Hour Angle         (how far Polaris has rotated from your meridian)
//    → Altitude + Azimuth (finally: where to physically point)
//
//  Each step is one simple formula. Work through them in order.
// =============================================================================

// Where is Polaris? (J2000.0 epoch coordinates — accurate enough until ~2050)
#define POLARIS_RA_HRS   2.5303    // Right Ascension: 2 hours 31 minutes
#define POLARIS_DEC_DEG  89.2641   // Declination: 89° 15' — very close to pole

// Shorthand
#define DEG2RAD(d)  ((d) * M_PI / 180.0)
#define RAD2DEG(r)  ((r) * 180.0 / M_PI)

// Wrap an angle into [0, 360)
static double wrap360(double d) {
    d = fmod(d, 360.0);
    if (d < 0.0) d += 360.0;
    return d;
}

// Wrap an angle into [-180, 180]
static double wrap180(double d) {
    d = fmod(d, 360.0);
    if (d >  180.0) d -= 360.0;
    if (d < -180.0) d += 360.0;
    return d;
}

// =============================================================================
//  Step 1: Unix time → Julian Date
//
//  Julian Date is just a continuous day count since noon, 1 Jan 4713 BC.
//  Astronomers use it because it has no months, years, or time zones —
//  just a single increasing decimal number.
//
//  The Unix epoch (1 Jan 1970 00:00 UTC) = JD 2440587.5
//  So we just divide unix seconds by 86400 and add that offset.
// =============================================================================
static double unix_to_jd(long unix_sec) {
    return 2440587.5 + (double)unix_sec / 86400.0;
}

// =============================================================================
//  Step 2: Julian Date → Greenwich Mean Sidereal Time (GMST)
//
//  Sidereal time is like a clock that tracks Earth's rotation relative
//  to the stars (not the Sun). It tells you which part of the sky is
//  directly overhead at Greenwich right now.
//
//  The formula comes from the IAU standard. T is Julian centuries
//  since J2000.0 (1 Jan 2000, 12:00 UTC = JD 2451545.0).
// =============================================================================
static double jd_to_gmst_deg(double jd) {
    double T = (jd - 2451545.0) / 36525.0;   // centuries since J2000.0
    double gmst = 280.46061837
                + 360.98564736629 * (jd - 2451545.0)  // Earth rotates ~361°/day
                + T * T * 0.000387933
                - T * T * T / 38710000.0;
    return wrap360(gmst);
}

// =============================================================================
//  Step 3: GMST + your longitude → Local Sidereal Time (LST)
//
//  GMST is sidereal time at longitude 0° (Greenwich).
//  For Pécs at 18.23°E, the sky has rotated 18.23° further eastward,
//  so your local sidereal time is GMST + 18.23°.
// =============================================================================
static double gmst_to_lst(double gmst_deg, double lon_deg) {
    return wrap360(gmst_deg + lon_deg);
}

// =============================================================================
//  Step 4: LST + Polaris RA → Hour Angle
//
//  Right Ascension (RA) is a star's fixed east-west position in the sky,
//  measured in hours (0h to 24h). It doesn't change as Earth rotates.
//
//  Hour Angle = how far the star has "moved" west of your meridian today.
//  When HA = 0, the star is exactly due south at its highest point.
//  When HA = 6h (90°), it has moved 6 hours west of there.
//
//  HA = LST - RA (both in degrees)
// =============================================================================
static double get_hour_angle_deg(double lst_deg, double ra_hrs) {
    double ra_deg = ra_hrs * 15.0;   // hours → degrees (24h × 15 = 360°)
    return wrap180(lst_deg - ra_deg);
}

// =============================================================================
//  Step 5: Hour Angle + Declination + Latitude → Altitude + Azimuth
//
//  This is the standard "equatorial to horizontal" coordinate transform.
//  It converts from sky coordinates (RA/Dec) to local coordinates (Alt/Az).
//
//  Declination = star's fixed north-south position (like latitude in the sky)
//  Latitude    = your geographic latitude
//  Hour Angle  = how far the star is from your meridian (from Step 4)
// =============================================================================
static void equatorial_to_altaz(double ha_deg, double dec_deg, double lat_deg,
                                 double *alt_out, double *az_out) {
    double ha  = DEG2RAD(ha_deg);
    double dec = DEG2RAD(dec_deg);
    double lat = DEG2RAD(lat_deg);

    // Altitude: how high above the horizon
    double sin_alt = sin(dec) * sin(lat)
                   + cos(dec) * cos(lat) * cos(ha);
    *alt_out = RAD2DEG(asin(sin_alt));

    // Azimuth: compass bearing
    double cos_az = (sin(dec) - sin(*alt_out * M_PI / 180.0) * sin(lat))
                  / (cos(*alt_out * M_PI / 180.0) * cos(lat));

    // Clamp to avoid floating-point acos domain errors
    if (cos_az >  1.0) cos_az =  1.0;
    if (cos_az < -1.0) cos_az = -1.0;

    *az_out = RAD2DEG(acos(cos_az));

    // acos always returns 0–180°.
    // If the hour angle is positive (star is west of meridian),
    // the azimuth must be in the 180–360° range.
    if (sin(ha) > 0.0) *az_out = 360.0 - *az_out;
}

// =============================================================================
//  Public API
// =============================================================================

PoleDirection astro_get_pole(double lat, double lon, long unix_sec) {
    // Walk through the five steps
    double jd  = unix_to_jd(unix_sec);
    double gmst = jd_to_gmst_deg(jd);
    double lst  = gmst_to_lst(gmst, lon);
    double ha   = get_hour_angle_deg(lst, POLARIS_RA_HRS);

    PoleDirection result;
    equatorial_to_altaz(ha, POLARIS_DEC_DEG, lat,
                        &result.altitude_deg, &result.azimuth_deg);
    return result;
}

float astro_total_error_deg(PoleDirection target,
                            float current_az, float current_alt) {
    // Angular distance between two points on a sphere.
    // This is the straight-line angle between where we're pointing
    // and where we need to point — accounts for both az and alt error.
    double az_err  = DEG2RAD(target.azimuth_deg  - current_az);
    double alt_err = DEG2RAD(target.altitude_deg - current_alt);

    // Pythagorean approximation — valid for small angles (< 5°)
    return (float)RAD2DEG(sqrt(az_err * az_err + alt_err * alt_err));
}
