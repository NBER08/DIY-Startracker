#include "gps.h"
#include "config.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <time.h>

static HardwareSerial GpsSerial(1); // UART 1
static TinyGPSPlus gps;

// -------------------------------------------------------------------------
// Build a Unix timestamp from the date/time that TinyGPSPlus decoded.
// Unix time = seconds since 1 Jan 1970 UTC.
// The astro math module needs this to calculate where Polaris is right now.
// -------------------------------------------------------------------------
static long build_unix_time() {
    if (!gps.date.isValid() || !gps.time.isValid()) return 0;

    struct tm t = {};
    t.tm_year = gps.date.year() - 1900;
    t.tm_mon = gps.date.month() - 1;
    t.tm_mday = gps.date.day();
    t.tm_hour = gps.time.hour();
    t.tm_min = gps.time.minute();
    t.tm_sec = gps.time.second();

    setenv("TZ", "UTC", 1);
    tzset();
    return (long)mktime(&t);
}

void gps_begin() {
    GpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

}

GpsFix gps_read() {
    while (GpsSerial.available()) {
        gps.encode(GpsSerial.read());
    }

    GpsFix fix = {};

    fix.sattelites = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
    fix.valid = gps.location.isValid();

    if (fix.valid) {
        fix.lat = gps.location.lat();
        fix.lon = gps.location.lng();
        fix.unix_sec = build_unix_time();
    }

    return fix;
}