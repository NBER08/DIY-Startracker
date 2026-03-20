#include "lora.h"
#include "config.h"
#include <Arduino.h>
#include <RadioLib.h>

static SX1261 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

static volatile bool packet_recieved = false;
static int16_t  last_rssi    = 0;
static LoraCmd  pending_cmd  = CMD_NONE;

#if defined(ESP32)
    ICACHE_RAM_ATTR
#endif
void set_flag(void) {
    packet_recieved = true;
}

void lora_begin() {
    int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, LORA_PWR, LORA_PREL);
    if (state == RADIOLIB_ERR_NONE) {
        // Initialization successful
    } else {
        Serial.print("lora init failed: ");
        Serial.println(state);
        while (true) { delay(10); }
    }
    
    radio.setPacketReceivedAction(set_flag);

    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        //Success
    } else {
        Serial.print("lora failed: ");
        Serial.println(state);
        while (true) { delay(10); }
    }
}

LoraCmdPacket lora_get_command(){
    LoraCmdPacket none = {CMD_NONE, 0, 0};
    if (packet_recieved) return none;
    packet_recieved = false;

    uint8_t buf[4] = {};
    int state = radio.readData(buf, sizeof(buf));
    radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) return none;

    last_rssi = (int16_t)radio.getRSSI();

    LoraCmd cmd = (LoraCmd)buf[0];
    if (cmd == CMD_NONE || cmd > CMD_SLEW_AZ) return none;

    Serial.printf("LoRa RX: cmd=0x%02X p_hi=%d p_lo=%d rssi=%d\n",
                  cmd, buf[1], buf[2], last_rssi);

    LoraCmdPacket pkt = {cmd, buf[1], buf[2]};
    return pkt;
}

void lora_send_status(const LoraStatus &s) {
    uint8_t buf[24] = {};

    buf[0] = 0xAB; // magic

    // Flags:
    buf[1] = (s.is_tracking        ? 0x01 : 0)  //0000000x
            |(s.is_on_target       ? 0x02 : 0)  //000000x0
            |(s.hum_warning        ? 0x04 : 0); //00000x00
            
    buf[2] = (uint8_t)s.gps_sattelites;

    // Temperature — degrees × 10, signed 16-bit
    int16_t temp = (int16_t)(s.temp * 10.0f);
    buf[3] = temp >> 8;  buf[4] = temp & 0xFF;
    // Humidity — percent × 10
    uint16_t hum = (uint16_t)(s.hum * 10.0f);
    buf[5] = hum >> 8;  buf[6] = hum & 0xFF;

    // Pole azimuth — degrees × 10, unsigned 16-bit
    uint16_t paz = (uint16_t)(s.pole_az_deg * 10.0f);
    buf[7] = paz >>8; buf[8] = paz & 0xFF;

    // Current platform azimuth — degrees × 10
    uint16_t caz = (uint16_t)(s.current_az_deg * 10.0f);
    buf[9] = caz >> 8;  buf[10] = caz & 0xFF;

    // Altitude correction — degrees × 10, signed 16-bit
    // positive = tilt up, negative = tilt down
    int16_t alt_corr = (int16_t)(s.alt_correction_deg * 10.0f);
    buf[11]  = alt_corr >> 8;  buf[12] = alt_corr & 0xFF;

    // Current camera azimuth — degrees × 10
    uint16_t ccaz = (uint16_t)(s.current_camera_az * 10.0f);
    buf[13] = ccaz >> 8;  buf[14] = ccaz & 0xFF;

    // Current camera altitude — degrees × 10
    uint16_t ccal = (uint16_t)(s.current_camera_alt * 10.0f);
    buf[15] = ccal >> 8;  buf[16] = ccal & 0xFF;

    // Battery voltage
    uint16_t batt_v = (uint16_t)(s.battery_mv);
    buf[17] = batt_v >> 8;  buf[18] = batt_v & 0xFF;

    // Power usage in mA
    uint16_t current_ma = (uint16_t)(s.current_ma);
    buf[19] = current_ma >> 8;  buf[20] = current_ma & 0xFF;

    // RSSI of last received packet
    buf[21] = last_rssi >> 8;  buf[22] = last_rssi & 0xFF;

    // buf[23] reserved

    radio.standby();
    int state = radio.transmit(buf, sizeof(buf));
    radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("LoRa TX failed: ");
        Serial.println(state);
    }
}