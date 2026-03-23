#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    int8_t  rssi;
    int8_t  snr;
    uint8_t payload;
    uint8_t payload_len;
} LoraPacket_t;

typedef struct {
    uint32_t frequency_hz;   // e.g. 868100000
    uint8_t  spreading_factor; // 7–12
    uint32_t bandwidth_hz;   // 125000
    uint8_t  tx_power_dbm;   // up to +18 for WLR089U0, but 14 is the max legal on 868.1
    uint8_t  coding_rate;    // 5–8
    uint8_t  sync_word;      // match tracker firmware, e.g. 0x34
} LoraConfig_t;

int  lora_init(const char* port, uint32_t baud, const LoraConfig_t* cfg);
void lora_deinit(void);

int  lora_recv(LoraPacket_t* out, uint32_t timeout_ms);
int  lora_send(const uint8_t* payload, uint8_t len);
int  lora_configure(const LoraConfig_t* cfg);

int8_t lora_last_rssi(void);
int8_t lora_last_snr(void);

