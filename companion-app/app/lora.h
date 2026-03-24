#pragma once
#include "protocol.h"
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
    uint8_t payload[255];
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



void  lora_init(const char* port, const LoraConfig_t* cfg, bool config);

void  lora_send(const char* payload);
int  lora_configure(int fd, const LoraConfig_t* cfg);

// Returns true if a new status packet arrived since the last call.
// Fills out_status with the decoded values.
int  lora_recv(LoraPacket_t* out, uint32_t timeout_ms);
int lora_recv_status(LoraStatus* out, uint32_t timeout_ms);

int8_t lora_last_rssi(void);
int8_t lora_last_snr(void);

// Send a command packet to the tracker (4 bytes on air)
void lora_send_cmd(LoraCmd cmd, uint8_t param_hi, uint8_t param_lo);

// Convenience wrappers
void lora_cmd_start(void);
void lora_cmd_stop(void);
void lora_cmd_ping(void);
void lora_cmd_shutter(uint16_t duration_tenths);   // duration in 0.1s steps
void lora_cmd_slew_az(uint16_t angle_tenths);     // degrees × 10