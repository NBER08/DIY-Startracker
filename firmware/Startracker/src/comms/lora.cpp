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
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 10, 8);

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("LoRa: init failed (error %d)\n", state);
        Serial.println("LoRa: check wiring and pin assignments in config.h");
        return;
    }

    // Tell RadioLib to call radio_isr when a packet arrives on DIO1
    radio.setDio1Action(radio_isr);

    // Start listening for incoming packets
    radio.startReceive();

    Serial.printf("LoRa: ready at %.1f MHz\n", (double)LORA_FREQ);
}

// -------------------------------------------------------------------------
// lora_get_command — call this in loop()
// Returns the received command, or CMD_NONE if nothing arrived.
// -------------------------------------------------------------------------
LoraCmd lora_get_command() {
    if (!packet_received) return CMD_NONE;
    packet_received = false;

    // Read the raw bytes out of the radio buffer
    uint8_t buf[4];
    int state = radio.readData(buf, sizeof(buf));

    if (state != RADIOLIB_ERR_NONE) {
        radio.startReceive();
        return CMD_NONE;
    }

    last_rssi = (int16_t)radio.getRSSI();

    // Re-enter receive mode immediately so the next packet isn't missed
    radio.startReceive();

    // First byte is the command ID — validate it
    LoraCmd cmd = (LoraCmd)buf[0];
    if (cmd < CMD_NONE || cmd > CMD_PING) return CMD_NONE;

    Serial.printf("LoRa: received command 0x%02X (RSSI=%d dBm)\n",
                  cmd, last_rssi);
    return cmd;
}

// -------------------------------------------------------------------------
// lora_send_status — transmit a status packet to the controller
// -------------------------------------------------------------------------
void lora_send_status(const LoraStatus &status) {
    // Pack status into a small byte buffer
    // Keep it simple — no struct packing, just the values we care about
    uint8_t buf[8];
    buf[0] = 0xAB;                                   // magic byte so receiver knows this is a status packet
    buf[1] = status.is_tracking   ? 0x01 : 0x00;
    buf[2] = status.gps_valid     ? 0x01 : 0x00;
    buf[3] = (uint8_t)status.gps_satellites;
    buf[4] = (uint8_t)("#");
    buf[5] = (uint8_t)("#");
    buf[6] = (uint8_t)(status.rssi >> 8);
    buf[7] = (uint8_t)(status.rssi & 0xFF);

    // Stop receiving while we transmit, then go back to receive
    radio.standby();
    int state = radio.transmit(buf, sizeof(buf));
    radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("LoRa: transmit failed (error %d)\n", state);
    }
}
