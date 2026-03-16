#include "gps.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <time.h>

// ---- Pin and port configuration ----
#define GPS_RX_PIN   44    // ESP32 receives from M8N TX
#define GPS_TX_PIN   43    // ESP32 transmits to M8N RX
#define GPS_BAUD     9600

// HardwareSerial(1) = UART1 on the ESP32-S3
// The number in brackets is the UART port (0, 1, or 2).
// Port 0 is used by the USB serial monitor, so we use port 1 for GPS.
static HardwareSerial GpsSerial(1);

// TinyGPSPlus does all the NMEA parsing for us.
// We just feed it bytes and ask it for values.
static TinyGPSPlus gps;

// -------------------------------------------------------------------------
// Build a Unix timestamp from the date/time that TinyGPSPlus decoded.
// Unix time = seconds since 1 Jan 1970 UTC.
// The astro math module needs this to calculate where Polaris is right now.
// -------------------------------------------------------------------------
static long build_unix_time() {
    if (!gps.date.isValid() || !gps.time.isValid()) return 0;

    struct tm t = {};
    t.tm_year = gps.date.year() - 1900;  // struct tm counts from 1900
    t.tm_mon  = gps.date.month() - 1;    // struct tm months are 0-indexed
    t.tm_mday = gps.date.day();
    t.tm_hour = gps.time.hour();
    t.tm_min  = gps.time.minute();
    t.tm_sec  = gps.time.second();

    // Tell mktime to treat this as UTC, not local time
    setenv("TZ", "UTC0", 1);
    tzset();

    return (long)mktime(&t);
}

// -------------------------------------------------------------------------
// Public functions
// -------------------------------------------------------------------------

void gps_begin() {
    // begin(baud, config, RX_pin, TX_pin)
    GpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("GPS: UART started, waiting for fix...");
}

GpsFix gps_read() {
    // Feed every available byte to TinyGPSPlus.
    // It silently parses NMEA sentences in the background.
    // When a sentence is complete, it updates its internal values.
    while (GpsSerial.available()) {
        gps.encode(GpsSerial.read());
    }

    // Now just read the decoded values back out.
    GpsFix fix = {};

    fix.satellites = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;

    // isValid() returns true once TinyGPSPlus has received
    // and decoded at least one valid reading for that field
    fix.valid = gps.location.isValid();

    if (fix.valid) {
        fix.lat      = gps.location.lat();   // already in decimal degrees
        fix.lon      = gps.location.lng();
        fix.unix_sec = build_unix_time();
    }

    return fix;
}
