#pragma once
#include <stdbool.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;

HardwareSerial GpsSerial(1);  // UART1 on ESP32-S3

void gps_init(uint16_t rx_pin, uint16_t tx_pin, uint32_t baud);

void gps_poll();
// This is everything the rest of the code needs to know about GPS.
// It doesn't need to know HOW we get it — just what we get.
/*
typedef struct {
    double lat;       // e.g. 46.07  (your latitude in degrees)
    double lon;       // e.g. 18.23  (your longitude in degrees)
    long   unix_sec;  // current UTC time as seconds since 1970
    int    satellites;
    bool   valid;     // false until we have a real fix
} GpsFix;

// Call once at startup
void gps_begin();

// Call anytime to get the latest data.
// Returns a GpsFix — check .valid before using it.
GpsFix gps_read();
*/