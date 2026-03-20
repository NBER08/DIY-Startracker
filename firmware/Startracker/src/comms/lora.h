#pragma once
#include <stdint.h>
#include <stdbool.h>

//=============================================================================

//=============================================================================

typedef enum {
    CMD_NONE             = 0x00,
    CMD_START            = 0x01,
    CMD_STOP             = 0x02,
    CMD_SHUTTER          = 0x03,
    CMD_PING             = 0x04,
    CMD_POINT_RA         = 0x05,
    CMD_POINT_DEC        = 0x06,
    CMD_SLEW_AZ          = 0X07,
} LoraCmd;

typedef struct {
    LoraCmd cmd;
    uint16_t param_hi;
    uint16_t param_lo;
} LoraCmdPacket;

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

void lora_begin();
LoraCmdPacket lora_get_command();
void lora_send_status(const LoraStatus &status);
void set_flag(void);