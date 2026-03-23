#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <termios.h>   // struct termios, tcgetattr, etc.
#include <fcntl.h>     // open(), O_RDWR, O_NOCTTY
#include <unistd.h>    // write(), close()
#include <string.h>    // strlen()
#include <stdio.h>     // perror()

typedef struct {
    int8_t  rssi;
    int8_t  snr;
    uint8_t payload;
    uint8_t payload_len;
} LoraPacket_t;

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
} LoraStatus;

void  lora_init(const char* port, const LoraConfig_t* cfg, bool config);

void  lora_send(const char* payload);
int  lora_configure(int fd, const LoraConfig_t* cfg);

// Returns true if a new status packet arrived since the last call.
// Fills out_status with the decoded values.
bool lora_get_status(LoraStatus *out_status);

void serialCheck(int fd);

int8_t lora_last_rssi(void);
int8_t lora_last_snr(void);

