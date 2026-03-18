#pragma once
#include <stdint.h>
#include <stdbool.h>

// Commands the tracker can receive from a controller
typedef enum {
    CMD_NONE           = 0x00,
    CMD_START          = 0x01,   // start tracking
    CMD_STOP           = 0x02,   // stop tracking
    CMD_SHUTTER        = 0x03,   // trigger camera shutter once, need to hold it there for given time, need to resolve!!!!!!
    CMD_PING           = 0x04,   // controller checking if tracker is alive
} LoraCmd;

// Status packet sent by the tracker every 5 seconds
typedef struct { 
    int      gps_satellites;
    bool     gps_valid;
    bool     is_tracking;
    int16_t  rssi;             // signal strength of last received packet
} LoraStatus;

void     lora_begin();
LoraCmd  lora_get_command();   // returns CMD_NONE if nothing received
void     lora_send_status(const LoraStatus &status);
