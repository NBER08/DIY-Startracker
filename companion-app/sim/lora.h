#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <termios.h>   // struct termios, tcgetattr, etc.
#include <fcntl.h>     // open(), O_RDWR, O_NOCTTY
#include <unistd.h>    // write(), close()
#include <string.h>    // strlen()
#include <stdio.h> 

typedef enum {
    CMD_NONE             = 0x00,
    CMD_START            = 0x01,
    CMD_STOP             = 0x02,
    CMD_SHUTTER          = 0x03,
    CMD_PING             = 0x04,
    CMD_SLEW_ALT         = 0x05,
    CMD_SLEW_AZ          = 0x06,
    CMD_SLEW_START       = 0x07,
} LoraCmd;

typedef struct {
    LoraCmd cmd;
    uint16_t param_hi;
    uint16_t param_lo;
} LoraCmdPacket;

typedef struct {
    const char* frequency_hz;   // e.g. 868100000 
    const char* spreading_factor; // 7–12
    const char* bandwidth_hz;   // 125000
    const char* tx_power_dbm;   // up to +18 for WLR089U0, but 14 is the max legal on 868.1
    const char* coding_rate;    // 5–8
    const char* sync_word; 
} LoraConfig_t;

typedef struct {
    bool is_tracking;

    int gps_sattelites;

    int temp;
    int hum;
    bool hum_warning;

    float pole_az_deg;
    float current_az_deg;
    float alt_correction_deg;

    float current_camera_az;
    float current_camera_alt;
    bool is_on_target;

    float   battery_mv;
    float   current_ma;

    int16_t rssi;
} LoraStatus_t;

void  lora_init(const char* port, const LoraConfig_t* cfg, bool config);
void lora_send_status(LoraStatus_t* status);