#include "astro.h"
#include <math.h>

// Polaris position (J2000)
#define POLARIS_RA_HRS  2.5303
#define POLARIS_DEC_DEG 89.2641

static double deg2rad(double d) { return d * M_PI / 180.0; }
static double rad2deg(double r) { return r * 180.0 / M_PI; }

static double wrap360(double d) {
    d = fmod(d, 360.0);
    return (d < 0) ? d + 360.0 : d;
}

// Julian date from unix timestamp
static double unix_to_jd(long unix_sec) {
    return 2440587.5 + unix_sec / 86400.0;
}

// Greenwich Mean Sidereal Time in degrees
static double gmst_deg(double jd) {
    double T = (jd - 2451545.0) / 36525.0;
    double g = 280.46061837
             + 360.98564736629 * (jd - 2451545.0)
             + T * T * 0.000387933
             - T * T * T / 38710000.0;
    return wrap360(g);
}

double astro_lst_deg(double lon_deg, long unix_sec) {
    return wrap360(gmst_deg(unix_to_jd(unix_sec)) + lon_deg);
}

// Hour angle of an object given LST and RA
static double hour_angle_deg(double lst_deg, double ra_hours) {
    double ha = lst_deg - ra_hours * 15.0;
    // Wrap to -180..+180
    ha = fmod(ha, 360.0);
    if (ha >  180.0) ha -= 360.0;
    if (ha < -180.0) ha += 360.0;
    return ha;
}

// Equatorial (HA, Dec) → Horizontal (Alt, Az)
static void eq_to_horiz(double ha_deg, double dec_deg, double lat_deg,
                         double *alt, double *az) {
    double ha  = deg2rad(ha_deg);
    double dec = deg2rad(dec_deg);
    double lat = deg2rad(lat_deg);

    double sin_alt = sin(dec)*sin(lat) + cos(dec)*cos(lat)*cos(ha);
    *alt = rad2deg(asin(sin_alt));

    double cos_az = (sin(dec) - sin(*alt * M_PI / 180.0) * sin(lat))
                  / (cos(*alt * M_PI / 180.0) * cos(lat));
    cos_az = fmax(-1.0, fmin(1.0, cos_az));   // clamp floating point noise
    *az = rad2deg(acos(cos_az));
    if (sin(ha) > 0.0) *az = 360.0 - *az;
}

PoleDirection astro_get_pole(double lat_deg, double lon_deg, long unix_sec) {
    double lst  = astro_lst_deg(lon_deg, unix_sec);
    double ha   = hour_angle_deg(lst, POLARIS_RA_HRS);
    double alt, az;
    eq_to_horiz(ha, POLARIS_DEC_DEG, lat_deg, &alt, &az);

    // The platform altitude should equal the observer's latitude.
    // alt_correction tells the user how much to tilt manually
    // to reach that from the current IMU-measured tilt.
    // (Current tilt is read in main.cpp — we just return the target here.)
    PoleDirection p;
    p.pole_alt_deg       = alt;
    p.pole_az_deg        = az;
    p.alt_correction_deg = 0.0;   // filled in by main.cpp using IMU reading
    return p;
}

CameraTarget astro_radec_to_motors(double lat_deg, double lon_deg, long unix_sec,
                                    double ra_hours, double dec_deg) {
    double lst = astro_lst_deg(lon_deg, unix_sec);
    double ha  = hour_angle_deg(lst, ra_hours);

    // Polar distance = angular separation from the celestial pole.
    // When polar_dist = 0 the camera points at the pole.
    // When polar_dist = 90 the camera points at the celestial equator.
    double polar_dist = 90.0 - dec_deg;

    CameraTarget t;
    t.ha_deg         = ha;
    t.polar_dist_deg = polar_dist;
    return t;
}
