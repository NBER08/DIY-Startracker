#include "gps.h"
#include <Arduino.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

// The M8N is wired to UART1 on these pins (match your config.h)
#define GPS_TX_PIN  43
#define GPS_RX_PIN  44
#define GPS_BAUD    9600

// We use HardwareSerial port 1 on the ESP32-S3
static HardwareSerial GpsSerial(1);

// We keep one copy of the latest fix in memory.
// Only gps_parse_line() writes to it; gps_read() reads from it.
static GpsFix latest_fix = { 0 };

// -------------------------------------------------------------------------
// Parse a $GPRMC sentence.
// Example: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
//
// We only use NMEA here (not UBX binary) to keep the code simple.
// The downside is 1-second time resolution — fine for learning.
// -------------------------------------------------------------------------
static void parse_gprmc(const char *line) {
    // strtok splits the string at each comma
    char buf[100];
    strncpy(buf, line, sizeof(buf));

    char *token;
    int  field = 0;

    char time_str[10] = "";
    char status       = 'V';   // V = invalid, A = active
    char lat_str[12]  = "";
    char lat_dir      = 'N';
    char lon_str[12]  = "";
    char lon_dir      = 'E';
    char date_str[8]  = "";

    token = strtok(buf, ",");
    while (token != NULL) {
        switch (field) {
            case 1: strncpy(time_str, token, sizeof(time_str)); break;
            case 2: status = token[0]; break;
            case 3: strncpy(lat_str,  token, sizeof(lat_str));  break;
            case 4: lat_dir = token[0]; break;
            case 5: strncpy(lon_str,  token, sizeof(lon_str));  break;
            case 6: lon_dir = token[0]; break;
            case 9: strncpy(date_str, token, sizeof(date_str)); break;
        }
        field++;
        token = strtok(NULL, ",");
    }

    // Only update if status is 'A' (active = valid fix)
    if (status != 'A') {
        latest_fix.valid = false;
        return;
    }

    // Convert NMEA latitude "DDMM.MMMM" → decimal degrees
    // e.g. "4807.038" = 48 degrees + 07.038/60 minutes
    double lat_raw = atof(lat_str);
    int    lat_deg = (int)(lat_raw / 100);
    double lat_min = lat_raw - (lat_deg * 100);
    latest_fix.lat = lat_deg + lat_min / 60.0;
    if (lat_dir == 'S') latest_fix.lat = -latest_fix.lat;

    double lon_raw = atof(lon_str);
    int    lon_deg = (int)(lon_raw / 100);
    double lon_min = lon_raw - (lon_deg * 100);
    latest_fix.lon = lon_deg + lon_min / 60.0;
    if (lon_dir == 'W') latest_fix.lon = -latest_fix.lon;

    // Build a unix timestamp from date and time fields
    // time_str = "HHMMSS.ss", date_str = "DDMMYY"
    if (strlen(date_str) >= 6 && strlen(time_str) >= 6) {
        struct tm t = {};
        t.tm_hour = (time_str[0]-'0')*10 + (time_str[1]-'0');
        t.tm_min  = (time_str[2]-'0')*10 + (time_str[3]-'0');
        t.tm_sec  = (time_str[4]-'0')*10 + (time_str[5]-'0');
        t.tm_mday = (date_str[0]-'0')*10 + (date_str[1]-'0');
        t.tm_mon  = (date_str[2]-'0')*10 + (date_str[3]-'0') - 1;
        t.tm_year = (date_str[4]-'0')*10 + (date_str[5]-'0') + 100; // years since 1900
        // mktime assumes local time; set TZ=UTC so it treats this as UTC
        setenv("TZ", "UTC0", 1);
        tzset();
        latest_fix.unix_sec = (long)mktime(&t);
    }

    latest_fix.valid = true;
}

// -------------------------------------------------------------------------
// Parse a $GPGGA sentence to get satellite count.
// Example: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,...
// -------------------------------------------------------------------------
static void parse_gpgga(const char *line) {
    char buf[100];
    strncpy(buf, line, sizeof(buf));

    char *token = strtok(buf, ",");
    int   field = 0;
    while (token != NULL) {
        if (field == 7) {                          // field 7 = satellite count
            latest_fix.satellites = atoi(token);
            break;
        }
        field++;
        token = strtok(NULL, ",");
    }
}

// -------------------------------------------------------------------------
// Read bytes from GPS UART and process complete lines.
// Call this in a loop — it's non-blocking.
// -------------------------------------------------------------------------
static void gps_poll() {
    static char line_buf[100];
    static int  line_pos = 0;

    while (GpsSerial.available()) {
        char c = GpsSerial.read();

        if (c == '\n' || c == '\r') {
            // End of line — process it
            if (line_pos > 6) {
                line_buf[line_pos] = '\0';

                if (strncmp(line_buf, "$GPRMC", 6) == 0) {
                    parse_gprmc(line_buf);
                } else if (strncmp(line_buf, "$GPGGA", 6) == 0) {
                    parse_gpgga(line_buf);
                }
            }
            line_pos = 0;  // reset for next line
        } else if (line_pos < (int)sizeof(line_buf) - 1) {
            line_buf[line_pos++] = c;
        }
    }
}

// -------------------------------------------------------------------------
// Public functions
// -------------------------------------------------------------------------

void gps_begin() {
    GpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("GPS: waiting for fix...");
}

GpsFix gps_read() {
    gps_poll();          // process any new bytes that arrived
    return latest_fix;   // return a copy of the latest known fix
}
