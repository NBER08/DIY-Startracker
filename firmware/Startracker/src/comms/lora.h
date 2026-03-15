#pragma once

// =============================================================================
//  comms/lora.h  —  SX1261 LoRa wireless link
//
//  Uses RadioLib. Two packet types:
//    COMMAND  — received from handheld controller / phone
//    TELEMETRY — transmitted by tracker every ~5 seconds while tracking
//
//  RF coexistence rule:
//    Never transmit LoRa while GPS is acquiring a fix (STATE_WAITING_FOR_GPS).
//    The SX1261 at +10 dBm will desensitise the M8N GPS receiver.
//    This is enforced in lora_send_telemetry() — it checks system state first.
// =============================================================================

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
//  Command packet  (received from controller, 8 bytes)
// =============================================================================

typedef enum {
    CMD_NONE           = 0x00,
    CMD_START_TRACKING = 0x01,
    CMD_STOP_TRACKING  = 0x02,
    CMD_REALIGN        = 0x03,   // re-run auto polar alignment
    CMD_RATE_SIDEREAL  = 0x10,
    CMD_RATE_LUNAR     = 0x11,
    CMD_RATE_SOLAR     = 0x12,
    CMD_SHUTTER_SINGLE = 0x20,   // trigger one exposure
    CMD_SHUTTER_BULB   = 0x21,   // open shutter (close with SHUTTER_CLOSE)
    CMD_SHUTTER_CLOSE  = 0x22,
    CMD_SHUTTER_BURST  = 0x23,   // n_frames exposures of duration_ms each
    CMD_MAG_RECAL      = 0x30,   // start magnetometer re-calibration
    CMD_PING           = 0xFE,   // controller sends, tracker echoes PONG
} lora_cmd_id_t;

typedef struct __attribute__((packed)) {
    uint8_t       magic;         // 0xAB — sanity check
    lora_cmd_id_t cmd;
    uint16_t      param_a;       // meaning depends on cmd (e.g. exposure ms)
    uint16_t      param_b;       // meaning depends on cmd (e.g. frame count)
    uint16_t      crc;           // CRC16 of bytes 0–5
} lora_cmd_packet_t;             // 8 bytes total

// =============================================================================
//  Telemetry packet  (sent from tracker, 24 bytes)
// =============================================================================

typedef struct __attribute__((packed)) {
    uint8_t  magic;              // 0xCD
    uint8_t  state;              // current system_state_t
    int16_t  alt_deg_x100;       // polar axis altitude × 100 (e.g. 4607 = 46.07°)
    int16_t  az_deg_x100;        // polar axis azimuth × 100
    int16_t  tracking_err_asec;  // IMU tracking error in arcseconds × 10
    int16_t  temperature_x10;    // BME280 temp × 10 (e.g. 152 = 15.2°C)
    uint16_t motor_mv;           // INA219 motor rail voltage in mV
    uint16_t motor_ma;           // INA219 motor current in mA
    uint8_t  gps_sats;           // satellites in fix
    uint8_t  flags;              // bit 0: dew warning, bit 1: low battery
    uint16_t crc;                // CRC16
} lora_telemetry_packet_t;       // 24 bytes total

// =============================================================================
//  API
// =============================================================================

// Initialise SX1261 via RadioLib. Must be called after spi_hal_init().
// Returns false if SX1261 does not respond.
bool lora_init(void);

// Returns the most recently received command, or CMD_NONE if nothing new.
// Clears the pending command flag after reading.
// Safe to call from any task.
lora_cmd_id_t lora_get_command(lora_cmd_packet_t *out_packet);

// Send a telemetry packet. Non-blocking (queues for transmission).
// Will silently drop if GPS is not yet fixed (RF coexistence rule).
void lora_send_telemetry(const lora_telemetry_packet_t *pkt);

// Send a PONG reply — called when CMD_PING is received.
// Includes RSSI of the received PING for link quality assessment.
void lora_send_pong(int16_t rssi_dbm);

// Returns RSSI of the last received packet in dBm.
int16_t lora_get_last_rssi(void);
