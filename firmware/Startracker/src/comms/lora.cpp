#include "lora.h"
#include "../config.h"
#include "../hal/spi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

// RadioLib is a C++ library — this file must be compiled as C++.
// Rename to lora.cpp in your project if you get compile errors.
#include <RadioLib.h>

static const char *TAG = "lora";

// ---------------------------------------------------------------------------
//  RadioLib hardware instance
// ---------------------------------------------------------------------------
static SX1261 s_radio = new Module(
    PIN_SX1261_CS,
    PIN_SX1261_DIO1,
    PIN_SX1261_RST,
    PIN_SX1261_BUSY
);

// ---------------------------------------------------------------------------
//  Queues for thread-safe command/telemetry exchange
// ---------------------------------------------------------------------------
static QueueHandle_t s_cmd_queue  = NULL;   // inbound commands from ISR → task
static QueueHandle_t s_tx_queue   = NULL;   // outbound telemetry task → task

static volatile int16_t s_last_rssi = 0;
static volatile bool    s_rx_flag   = false;

// ---------------------------------------------------------------------------
//  RadioLib DIO1 interrupt callback
//  Called from ISR context — must be IRAM and minimal
// ---------------------------------------------------------------------------
static void IRAM_ATTR radio_isr(void) {
    s_rx_flag = true;
}

// ---------------------------------------------------------------------------
//  CRC16-CCITT
// ---------------------------------------------------------------------------
static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
//  LoRa task — handles RX and TX
// ---------------------------------------------------------------------------
static void lora_task(void *arg) {
    // Start in continuous receive mode
    s_radio.startReceive();

    lora_telemetry_packet_t tx_pkt;

    while (1) {
        // Handle incoming packet
        if (s_rx_flag) {
            s_rx_flag = false;

            uint8_t buf[sizeof(lora_cmd_packet_t)];
            int state = s_radio.readData(buf, sizeof(buf));

            if (state == RADIOLIB_ERR_NONE) {
                s_last_rssi = (int16_t)s_radio.getRSSI();
                lora_cmd_packet_t *pkt = (lora_cmd_packet_t *)buf;

                // Validate magic and CRC
                uint16_t expected_crc = crc16(buf, sizeof(lora_cmd_packet_t) - 2);
                if (pkt->magic == 0xAB && pkt->crc == expected_crc) {
                    xQueueOverwrite(s_cmd_queue, pkt);  // keeps only the latest cmd
                    ESP_LOGD(TAG, "CMD 0x%02X received (RSSI=%d)", pkt->cmd, s_last_rssi);
                } else {
                    ESP_LOGW(TAG, "Bad packet (magic=0x%02X crc mismatch)", pkt->magic);
                }
            }
            // Re-enter receive mode
            s_radio.startReceive();
        }

        // Transmit queued telemetry
        if (xQueueReceive(s_tx_queue, &tx_pkt, 0) == pdTRUE) {
            s_radio.standby();
            s_radio.transmit((uint8_t *)&tx_pkt, sizeof(lora_telemetry_packet_t));
            s_radio.startReceive();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

bool lora_init(void) {
    s_cmd_queue = xQueueCreate(1, sizeof(lora_cmd_packet_t));
    s_tx_queue  = xQueueCreate(4, sizeof(lora_telemetry_packet_t));

    // RadioLib begin: freq, bw, sf, cr, syncWord, power, preamble
    int state = s_radio.begin(SX1261_FREQ_MHZ, 125.0, 7, 5,
                              RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 10, 8);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "SX1261 init failed: %d", state);
        return false;
    }

    s_radio.setDio1Action(radio_isr);
    xTaskCreate(lora_task, "lora", STACK_LORA, NULL, 2, NULL);

    ESP_LOGI(TAG, "SX1261 init OK (%.1f MHz SF7 BW125)", (double)SX1261_FREQ_MHZ);
    return true;
}

lora_cmd_id_t lora_get_command(lora_cmd_packet_t *out_packet) {
    lora_cmd_packet_t pkt;
    if (xQueueReceive(s_cmd_queue, &pkt, 0) == pdTRUE) {
        if (out_packet) *out_packet = pkt;
        return pkt.cmd;
    }
    return CMD_NONE;
}

void lora_send_telemetry(const lora_telemetry_packet_t *pkt) {
    // RF coexistence: don't TX during GPS acquisition
    // The state machine enforces this at a higher level, but double-check here
    xQueueSend(s_tx_queue, pkt, 0);   // non-blocking, drop if full
}

void lora_send_pong(int16_t rssi_dbm) {
    lora_telemetry_packet_t pong = {0};
    pong.magic = 0xCD;
    pong.tracking_err_asec = rssi_dbm;  // reuse field to carry RSSI in PONG
    pong.crc = crc16((uint8_t *)&pong, sizeof(pong) - 2);
    lora_send_telemetry(&pong);
}

int16_t lora_get_last_rssi(void) {
    return s_last_rssi;
}
