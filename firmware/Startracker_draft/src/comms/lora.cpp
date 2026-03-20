#include "lora.h"
#include "config.h"
#include <Arduino.h>
#include <RadioLib.h>

// ---- RadioLib SX1261 instance ----
static SX1261 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

// Flag set by the DIO1 interrupt when a packet arrives.
// volatile because it is written in an ISR and read in the main loop.
static volatile bool packet_received = false;

// Last received command and RSSI
static LoraCmd  pending_cmd  = CMD_NONE;
static int16_t  last_rssi    = 0;

// -------------------------------------------------------------------------
// DIO1 interrupt — called by the radio when a packet finishes arriving.
// Just sets a flag. All the actual reading happens in lora_get_command()
// in the main loop where it is safe to call RadioLib functions.
// -------------------------------------------------------------------------
void IRAM_ATTR radio_isr() {
    packet_received = true;
}

// -------------------------------------------------------------------------
// lora_begin — initialise the SX1261 and start listening
// -------------------------------------------------------------------------
void lora_begin() {
    // begin(frequency MHz, bandwidth kHz, spreading factor, coding rate,
    //       sync word, output power dBm, preamble length)
    int state = radio.begin(LORA_FREQ, 125.0, 7, 5,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                            LORA_TX_DBM, 8);

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("LoRa: init failed (error %d)\n", state);
        Serial.println("LoRa: check wiring and pin assignments in config.h");
        return;
    }

    // Tell RadioLib to call radio_isr when a packet arrives on DIO1
    radio.setDio1Action(radio_isr);
    radio.startReceive();
    Serial.printf("LoRa: ready at %.1f MHz  TX=%d dBm\n",
                  (double)LORA_FREQ, LORA_TX_DBM);
}

// -------------------------------------------------------------------------
// lora_get_command — call this in loop()
// Returns the received command, or CMD_NONE if nothing arrived.
// -------------------------------------------------------------------------
LoraCmdPacket lora_get_command() {
    LoraCmdPacket none = { CMD_NONE, 0, 0 };
    if (!packet_received) return none;
    packet_received = false;

    uint8_t buf[4] = {};
    int state = radio.readData(buf, sizeof(buf));
    radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) return none;

    last_rssi = (int16_t)radio.getRSSI();

    LoraCmd cmd = (LoraCmd)buf[0];
    if (cmd == CMD_NONE || cmd > CMD_SLEW_AZ) return none;

    Serial.printf("LoRa RX: cmd=0x%02X p_hi=%d p_lo=%d RSSI=%d\n",
                  cmd, buf[1], buf[2], last_rssi);

    LoraCmdPacket pkt = { cmd, buf[1], buf[2] };
    return pkt;
}

// -------------------------------------------------------------------------
// lora_send_status — transmit a status packet to the controller
// -------------------------------------------------------------------------
void lora_send_status(const LoraStatus &s) {
    uint8_t buf[22] = {};

    buf[0] = 0xAB;   // magic

    // Flags byte
    buf[1] = (s.is_tracking      ? 0x01 : 0)
           | (s.gps_valid        ? 0x02 : 0)
           | (s.humidity_warning ? 0x04 : 0);

    buf[2] = (uint8_t)s.gps_satellites;

    // Pole azimuth — degrees × 10, unsigned 16-bit
    uint16_t paz = (uint16_t)(s.pole_az_deg * 10.0f);
    buf[3] = paz >> 8;  buf[4] = paz & 0xFF;

    // Current platform azimuth — degrees × 10
    uint16_t caz = (uint16_t)(s.current_az_deg * 10.0f);
    buf[5] = caz >> 8;  buf[6] = caz & 0xFF;

    // Altitude correction — degrees × 10, signed 16-bit
    // positive = tilt up, negative = tilt down
    int16_t alt_corr = (int16_t)(s.alt_correction_deg * 10.0f);
    buf[7]  = alt_corr >> 8;  buf[8] = alt_corr & 0xFF;

    // Camera tilt angle — degrees × 10
    uint16_t ctilt = (uint16_t)(s.camera_tilt_deg * 10.0f);
    buf[9] = ctilt >> 8;  buf[10] = ctilt & 0xFF;

    // Temperature — degrees × 10, signed 16-bit
    int16_t temp = (int16_t)(s.temperature_c * 10.0f);
    buf[11] = temp >> 8;  buf[12] = temp & 0xFF;
    // Humidity — percent × 10
    uint16_t hum = (uint16_t)(s.humidity_pct * 10.0f);
    buf[13] = hum >> 8;  buf[14] = hum & 0xFF;

    // Battery voltage — millivolts
    uint16_t mv = (uint16_t)s.battery_mv;
    buf[15] = mv >> 8;  buf[16] = mv & 0xFF;
    // Current draw — milliamps
    uint16_t ma = (uint16_t)s.current_ma;
    buf[17] = ma >> 8;  buf[18] = ma & 0xFF;

    // RSSI of last received packet
    buf[19] = last_rssi >> 8;  buf[20] = last_rssi & 0xFF;

    // buf[21] reserved

    radio.standby();
    int state = radio.transmit(buf, sizeof(buf));
    radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("LoRa TX failed (%d)\n", state);
    }
}
