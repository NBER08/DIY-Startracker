#include "lora.h"
#include "config.h"
#include <Arduino.h>
#include <RadioLib.h>

// ---- RadioLib SX1261 instance ----
// Pins: CS, DIO1, RST, BUSY — all defined in config.h
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

    // Start listening for incoming packets
    radio.startReceive();

    Serial.printf("LoRa: ready at %.1f MHz  TX=%d dBm\n",
                  (double)LORA_FREQ, LORA_TX_DBM);
}

// -------------------------------------------------------------------------
// lora_get_command — call this in loop()
// Returns the received command, or CMD_NONE if nothing arrived.
// -------------------------------------------------------------------------
LoraCmdPacket lora_get_command() {
    LoraCmdPacket none = { CMD_NONE, 0 };
    if (!packet_received) return none;
    packet_received = false;

    // Packet format: [cmd, param, 0x00, 0x00]
    uint8_t buf[4] = {};
    int state = radio.readData(buf, sizeof(buf));
    radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) return none;

    last_rssi = (int16_t)radio.getRSSI();

    LoraCmd cmd = (LoraCmd)buf[0];
    if (cmd < CMD_START || cmd > CMD_PING) return none;

    Serial.printf("LoRa: cmd=0x%02X param=%d RSSI=%d dBm\n",
                  cmd, buf[1], last_rssi);

    LoraCmdPacket pkt = { cmd, buf[1] };
    return pkt;
}

// -------------------------------------------------------------------------
// lora_send_status — transmit a status packet to the controller
// -------------------------------------------------------------------------
void lora_send_status(const LoraStatus &s) {
    // Pack everything into 16 bytes
    uint8_t buf[16] = {};

    buf[0]  = 0xAB;                                      // magic

    buf[1]  = (s.is_tracking      ? 0x01 : 0x00)
            | (s.gps_valid        ? 0x02 : 0x00)
            | (s.humidity_warning ? 0x04 : 0x00);        // flags byte

    buf[2]  = (uint8_t)s.gps_satellites;

    // tracking error — store as degrees × 100, signed 16-bit
    int16_t err = (int16_t)(s.tracking_error * 100.0f);
    buf[3]  = (uint8_t)(err >> 8);
    buf[4]  = (uint8_t)(err & 0xFF);

    // temperature — degrees C × 10, signed 16-bit  (e.g. 15.3°C → 153)
    int16_t temp = (int16_t)(s.temperature_c * 10.0f);
    buf[5]  = (uint8_t)(temp >> 8);
    buf[6]  = (uint8_t)(temp & 0xFF);

    // humidity — percent × 10, unsigned 16-bit  (e.g. 65.4% → 654)
    uint16_t hum = (uint16_t)(s.humidity_pct * 10.0f);
    buf[7]  = (uint8_t)(hum >> 8);
    buf[8]  = (uint8_t)(hum & 0xFF);

    // battery voltage — millivolts, unsigned 16-bit  (e.g. 11800 mV)
    uint16_t mv = (uint16_t)s.battery_mv;
    buf[9]  = (uint8_t)(mv >> 8);
    buf[10] = (uint8_t)(mv & 0xFF);

    // current draw — milliamps, unsigned 16-bit
    uint16_t ma = (uint16_t)s.current_ma;
    buf[11] = (uint8_t)(ma >> 8);
    buf[12] = (uint8_t)(ma & 0xFF);

    // RSSI of last received packet
    buf[13] = (uint8_t)(last_rssi >> 8);
    buf[14] = (uint8_t)(last_rssi & 0xFF);

    // buf[15] reserved

    radio.standby();
    int state = radio.transmit(buf, sizeof(buf));
    radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("LoRa: transmit failed (error %d)\n", state);
    }
}