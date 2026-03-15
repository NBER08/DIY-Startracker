#include "astro_math.h"
#include <math.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
//  Internal helpers
// ---------------------------------------------------------------------------

static double deg_to_rad(double d) { return d * M_PI / 180.0; }
static double rad_to_deg(double r) { return r * 180.0 / M_PI; }

// Reduce angle to [0, 360)
static double wrap_360(double d) {
    d = fmod(d, 360.0);
    if (d < 0.0) d += 360.0;
    return d;
}

// Reduce angle to [-180, 180]
static double wrap_180(double d) {
    d = fmod(d, 360.0);
    if (d >  180.0) d -= 360.0;
    if (d < -180.0) d += 360.0;
    return d;
}

// ---------------------------------------------------------------------------
//  Julian Date
// ---------------------------------------------------------------------------

double astro_unix_ms_to_jd(int64_t unix_ms) {
    // Unix epoch (1970-01-01 00:00:00 UTC) = JD 2440587.5
    return 2440587.5 + (double)unix_ms / 86400000.0;
}

// ---------------------------------------------------------------------------
//  Greenwich Mean Sidereal Time
//  Formula: Astronomical Algorithms, Meeus, Ch. 12
// ---------------------------------------------------------------------------

double astro_jd_to_gmst_deg(double jd) {
    double T = (jd - 2451545.0) / 36525.0;   // Julian centuries from J2000.0
    double gmst = 280.46061837
                + 360.98564736629 * (jd - 2451545.0)
                + T * T * 0.000387933
                - T * T * T / 38710000.0;
    return wrap_360(gmst);
}

// ---------------------------------------------------------------------------
//  Local Sidereal Time
// ---------------------------------------------------------------------------

double astro_local_sidereal_time_deg(double gmst_deg, double lon_deg) {
    return wrap_360(gmst_deg + lon_deg);
}

// ---------------------------------------------------------------------------
//  Hour angle
// ---------------------------------------------------------------------------

double astro_hour_angle_deg(double lst_deg, double ra_hrs) {
    double ra_deg = ra_hrs * 15.0;   // convert hours to degrees
    return wrap_180(lst_deg - ra_deg);
}

// ---------------------------------------------------------------------------
//  Equatorial → Horizontal  (Alt/Az)
//  Standard spherical trig formulae
// ---------------------------------------------------------------------------

void astro_equatorial_to_horizontal(double ha_deg,  double dec_deg,
                                    double lat_deg,
                                    double *alt_out, double *az_out) {
    double ha  = deg_to_rad(ha_deg);
    double dec = deg_to_rad(dec_deg);
    double lat = deg_to_rad(lat_deg);

    double sin_alt = sin(dec) * sin(lat)
                   + cos(dec) * cos(lat) * cos(ha);
    double alt = asin(sin_alt);

    double cos_az = (sin(dec) - sin(alt) * sin(lat))
                  / (cos(alt) * cos(lat));

    // Clamp to [-1, 1] to avoid acos domain errors from floating point noise
    if (cos_az >  1.0) cos_az =  1.0;
    if (cos_az < -1.0) cos_az = -1.0;

    double az = acos(cos_az);

    // If hour angle is positive (west of meridian), az is in [180, 360)
    if (sin(ha) > 0.0) {
        az = 2.0 * M_PI - az;
    }

    *alt_out = rad_to_deg(alt);
    *az_out  = rad_to_deg(az);
}

// ---------------------------------------------------------------------------
//  Top-level convenience — what the state machine calls
// ---------------------------------------------------------------------------

void astro_get_pole_direction(double lat_deg,   double lon_deg,
                               int64_t unix_ms,
                               double *alt_out,  double *az_out) {
    double jd   = astro_unix_ms_to_jd(unix_ms);
    double gmst = astro_jd_to_gmst_deg(jd);
    double lst  = astro_local_sidereal_time_deg(gmst, lon_deg);
    double ha   = astro_hour_angle_deg(lst, POLARIS_RA_HRS);

    // Polaris is 0.74° from the true pole — we aim at Polaris's current
    // position and let the drift alignment (IMU feedback) do the fine tuning.
    astro_equatorial_to_horizontal(ha, POLARIS_DEC_DEG, lat_deg, alt_out, az_out);
}

// ---------------------------------------------------------------------------
//  Atmospheric refraction — Bennett's formula
// ---------------------------------------------------------------------------

double astro_refraction_arcmin(double h_deg,
                                double pressure_hpa,
                                double temp_c) {
    if (h_deg < -1.0) return 0.0;   // below horizon, skip
    double R = 1.02 / tan(deg_to_rad(h_deg + 10.3 / (h_deg + 5.11)));
    // Correct for pressure and temperature
    R *= (pressure_hpa / 1013.0) * (283.0 / (273.0 + temp_c));
    return R;  // arcminutes
}

// ---------------------------------------------------------------------------
//  Earth Rotation Angle (more precise than GMST for high-accuracy work)
// ---------------------------------------------------------------------------

double astro_era_deg(double jd_ut1) {
    double Du = jd_ut1 - 2451545.0;
    double era = 360.0 * (0.7790572732640 + 1.00273781191135448 * Du);
    return wrap_360(era);
}

// ---------------------------------------------------------------------------
//  Display helpers
// ---------------------------------------------------------------------------

void astro_deg_to_hms(double deg, char *buf, int buf_len) {
    deg = fabs(deg);
    double hrs = deg / 15.0;
    int h = (int)hrs;
    int m = (int)((hrs - h) * 60.0);
    double s = ((hrs - h) * 60.0 - m) * 60.0;
    snprintf(buf, buf_len, "%02dh %02dm %05.2fs", h, m, s);
}

void astro_deg_to_dms(double deg, char *buf, int buf_len) {
    char sign = (deg < 0) ? '-' : '+';
    deg = fabs(deg);
    int d = (int)deg;
    int m = (int)((deg - d) * 60.0);
    double s = ((deg - d) * 60.0 - m) * 60.0;
    snprintf(buf, buf_len, "%c%02d° %02d' %05.2f\"", sign, d, m, s);
}
