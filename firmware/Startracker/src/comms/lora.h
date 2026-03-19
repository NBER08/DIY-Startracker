#pragma once
#include <stdint.h>
#include <stdbool.h>

// Commands the tracker can receive from a controller
typedef enum {
    CMD_NONE           = 0x00,
    CMD_START          = 0x01,   // start tracking
    CMD_STOP           = 0x02,   // stop tracking
    CMD_SHUTTER        = 0x03,   // open shutter for param seconds, then close
    CMD_PING           = 0x04,   // controller checking if tracker is alive
} LoraCmd;


// Received command plus its parameter byte
typedef struct {
    LoraCmd cmd;
    uint8_t param;   // for CMD_SHUTTER: exposure duration in seconds (1–255)
} LoraCmdPacket;


// Status packet sent by the tracker every 5 seconds
typedef struct {
    // Tracking
    bool    is_tracking;

    // GPS
    int     gps_satellites;
    bool    gps_valid;

    // Environment (from BME280)
    float   temperature_c;
    float   humidity_pct;
    bool    humidity_warning;  // true when humidity > 80%

    // Power (from INA219)
    float   battery_mv;        // battery voltage in millivolts
    float   current_ma;        // current draw in milliamps

    // Link quality
    int16_t rssi;
} LoraStatus;

void     lora_begin();
LoraCmdPacket  lora_get_command();   // returns CMD_NONE if nothing received
void     lora_send_status(const LoraStatus &status);
