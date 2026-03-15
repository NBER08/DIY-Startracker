#pragma once

// =============================================================================
//  sensors/gps.h  —  M8N GPS driver
//
//  Protocol: UBX binary (not NMEA).
//  On init, sends UBX-CFG-PRT to disable NMEA and enable UBX-NAV-PVT at 1 Hz.
//
//  The 1PPS GPIO interrupt disciplins a software clock so that UTC time
//  can be interpolated to sub-millisecond accuracy between fix updates.
//
//  gps_get_fix() is safe to call from any task. It returns a snapshot
//  protected by a mutex — it never blocks for more than one I2C transaction.
// =============================================================================

#include <stdint.h>
#include <stdbool.h>

// Fix quality levels — mirrors UBX gnssFixOK flags
typedef enum {
    GPS_FIX_NONE    = 0,
    GPS_FIX_2D      = 2,
    GPS_FIX_3D      = 3,
    GPS_FIX_DGPS    = 4,
} gps_fix_type_t;

typedef struct {
    double          lat_deg;        // positive = North
    double          lon_deg;        // positive = East
    double          alt_m;          // altitude above MSL
    uint32_t        utc_ms;         // UTC time as ms since GPS epoch (week 0)
    int64_t         unix_time_ms;   // ms since Unix epoch — use for astro math
    uint8_t         satellites;     // number of satellites used in fix
    gps_fix_type_t  fix_type;
    bool            valid;          // true once first valid 3D fix received
} gps_fix_t;

// Initialise UART, configure M8N for UBX binary, start parser task.
// Must be called after uart_hal_init(GPS_UART_PORT).
void gps_init(void);

// Returns a copy of the most recent fix. valid=false until first fix acquired.
// Safe to call from any task.
gps_fix_t gps_get_fix(void);

// Returns current UTC time interpolated using the 1PPS signal.
// More accurate than gps_fix_t.unix_time_ms between fix updates.
// Returns 0 if no fix yet.
int64_t gps_get_utc_ms_interpolated(void);

// True if a valid 3D fix has been acquired and is fresher than 5 seconds.
bool gps_is_ready(void);
