#pragma once
#include <stdint.h>
#include <stdbool.h>

// =============================================================================
//  Commands: [cmd, param_hi, param_lo, 0x00]
//
//  CMD_SHUTTER:  param = exposure seconds (param_hi only, 1–255)
//  CMD_POINT_RA: param_hi = RA hours (0–23), param_lo = RA minutes (0–59)
//  CMD_POINT_DEC:param_hi = Dec degrees signed (+90 to -90, stored +128 offset)
//                param_lo = Dec arcminutes (0–59)
//  All others:   param ignored
// =============================================================================
typedef enum {
    CMD_NONE      = 0x00,
    CMD_START     = 0x01,   // start sidereal tracking
    CMD_STOP      = 0x02,   // stop tracking
    CMD_SHUTTER   = 0x03,   // open shutter for param_hi seconds
    CMD_PING      = 0x04,   // ping
    CMD_POINT_RA  = 0x05,   // set target RA (send before CMD_POINT_DEC)
    CMD_POINT_DEC = 0x06,   // set target Dec and execute pointing move
    CMD_SLEW_AZ   = 0x07,   // start azimuth slew to align with pole
} LoraCmd;


// Received command plus its parameter byte
typedef struct {
    LoraCmd cmd;
    uint8_t param_hi;
    uint8_t param_lo;
} LoraCmdPacket;


// Status packet sent by the tracker every 5 seconds
typedef struct {
    // Tracking
    bool    is_tracking;

    // GPS
    int     gps_satellites;
    bool    gps_valid;

    // Alignment
    float   pole_az_deg;           // computed pole azimuth
    float   current_az_deg;        // current platform azimuth (from magnetometer)
    float   alt_correction_deg;    // how many degrees to tilt platform manually
                                   // positive = tilt up, negative = tilt down

    // Camera pointing
    float   camera_tilt_deg;       // current camera polar distance angle

    // Environment
    float   temperature_c;
    float   humidity_pct;
    bool    humidity_warning;

    // Power
    float   battery_mv;
    float   current_ma;

    int16_t rssi;
} LoraStatus;

void     lora_begin();
LoraCmdPacket  lora_get_command();   // returns CMD_NONE if nothing received
void     lora_send_status(const LoraStatus &status);
