#include "gps.h"
#include "../config.h"
#include "../hal/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "gps";

// ---------------------------------------------------------------------------
//  UBX framing constants
// ---------------------------------------------------------------------------
#define UBX_SYNC1       0xB5
#define UBX_SYNC2       0x62
#define UBX_CLASS_NAV   0x01
#define UBX_CLASS_CFG   0x06
#define UBX_ID_NAV_PVT  0x07
#define UBX_ID_CFG_PRT  0x00
#define UBX_ID_CFG_MSG  0x01

// UBX-NAV-PVT payload is exactly 92 bytes
#define NAV_PVT_LEN     92

// ---------------------------------------------------------------------------
//  UBX-NAV-PVT structure — matches M8N datasheet Table 10
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
    uint32_t iTOW;          // GPS time of week, ms
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;         // bit 0=validDate, 1=validTime, 2=fullyResolved
    uint32_t tAcc;          // time accuracy estimate, ns
    int32_t  nano;          // fraction of second, ns (-1e9..1e9)
    uint8_t  fixType;       // 0=none,2=2D,3=3D,4=GNSS+DR
    uint8_t  flags;         // bit 0=gnssFixOK
    uint8_t  flags2;
    uint8_t  numSV;         // satellites used
    int32_t  lon;           // degrees × 1e-7
    int32_t  lat;           // degrees × 1e-7
    int32_t  height;        // height above ellipsoid, mm
    int32_t  hMSL;          // height above MSL, mm
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t  velN;
    int32_t  velE;
    int32_t  velD;
    int32_t  gSpeed;
    int32_t  headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t  reserved[6];
    int32_t  headVeh;
    int16_t  magDec;
    uint16_t magAcc;
} ubx_nav_pvt_t;

// ---------------------------------------------------------------------------
//  Module state
// ---------------------------------------------------------------------------
static gps_fix_t       s_fix      = {0};
static SemaphoreHandle_t s_mutex  = NULL;

// 1PPS interpolation
static volatile int64_t DRAM_ATTR s_pps_esp_us  = 0;   // esp_timer value at last PPS edge
static volatile uint32_t DRAM_ATTR s_pps_itow_ms = 0;  // GPS iTOW at that edge

// ---------------------------------------------------------------------------
//  1PPS interrupt — captures ESP timer value at the exact PPS edge
// ---------------------------------------------------------------------------
static void IRAM_ATTR pps_isr_handler(void *arg) {
    s_pps_esp_us = esp_timer_get_time();     // microseconds since boot
    s_pps_itow_ms = s_fix.utc_ms;           // last known whole-second boundary
}

// ---------------------------------------------------------------------------
//  UBX checksum  (8-bit Fletcher, over class+id+length+payload)
// ---------------------------------------------------------------------------
static void ubx_checksum(const uint8_t *data, size_t len,
                          uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = 0; *ck_b = 0;
    for (size_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

// ---------------------------------------------------------------------------
//  Send a UBX packet to the M8N
// ---------------------------------------------------------------------------
static void ubx_send(uint8_t cls, uint8_t id,
                      const uint8_t *payload, uint16_t payload_len) {
    uint8_t hdr[6] = {
        UBX_SYNC1, UBX_SYNC2, cls, id,
        (uint8_t)(payload_len & 0xFF),
        (uint8_t)(payload_len >> 8)
    };
    uint8_t ck_a, ck_b;

    // Checksum covers cls, id, length bytes, and payload
    uint8_t ck_buf[payload_len + 4];
    ck_buf[0] = cls; ck_buf[1] = id;
    ck_buf[2] = hdr[4]; ck_buf[3] = hdr[5];
    if (payload_len > 0) memcpy(ck_buf + 4, payload, payload_len);
    ubx_checksum(ck_buf, payload_len + 4, &ck_a, &ck_b);

    uart_hal_write(GPS_UART_PORT, hdr, 6);
    if (payload_len > 0) uart_hal_write(GPS_UART_PORT, payload, payload_len);
    uint8_t footer[2] = {ck_a, ck_b};
    uart_hal_write(GPS_UART_PORT, footer, 2);
}

// ---------------------------------------------------------------------------
//  Configure M8N on startup:
//    1. Switch UART1 to UBX-only (disable NMEA output)
//    2. Enable UBX-NAV-PVT at 1 Hz
// ---------------------------------------------------------------------------
static void configure_m8n(void) {
    // UBX-CFG-PRT: set UART1 to UBX in+out only, 9600 baud
    uint8_t cfg_prt[20] = {
        0x01,           // portID = UART1
        0x00,           // reserved
        0x00, 0x00,     // txReady
        0xD0, 0x08, 0x00, 0x00, // mode: 8N1
        0x80, 0x25, 0x00, 0x00, // baudRate: 9600
        0x01, 0x00,     // inProtoMask: UBX only
        0x01, 0x00,     // outProtoMask: UBX only
        0x00, 0x00,     // flags
        0x00, 0x00,     // reserved
    };
    ubx_send(UBX_CLASS_CFG, UBX_ID_CFG_PRT, cfg_prt, sizeof(cfg_prt));
    vTaskDelay(pdMS_TO_TICKS(200));

    // UBX-CFG-MSG: enable NAV-PVT at 1 message per second on UART1
    uint8_t cfg_msg[8] = {
        UBX_CLASS_NAV, UBX_ID_NAV_PVT,
        0x00, 0x01, 0x00, 0x00, 0x00, 0x00  // rate=1 on UART1, 0 on others
    };
    ubx_send(UBX_CLASS_CFG, UBX_ID_CFG_MSG, cfg_msg, sizeof(cfg_msg));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "M8N configured for UBX-NAV-PVT @ 1 Hz");
}

// ---------------------------------------------------------------------------
//  Parse one UBX-NAV-PVT payload and update s_fix
// ---------------------------------------------------------------------------
static void parse_nav_pvt(const uint8_t *payload) {
    ubx_nav_pvt_t pvt;
    memcpy(&pvt, payload, sizeof(pvt));

    bool time_ok = (pvt.valid & 0x03) == 0x03;   // validDate + validTime
    bool fix_ok  = (pvt.flags & 0x01) && (pvt.fixType >= 3);

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return;

    s_fix.lat_deg = pvt.lat * 1e-7;
    s_fix.lon_deg = pvt.lon * 1e-7;
    s_fix.alt_m   = pvt.hMSL * 1e-3;
    s_fix.satellites = pvt.numSV;
    s_fix.fix_type   = (gps_fix_type_t)pvt.fixType;

    if (time_ok) {
        // Build unix time from year/month/day/hour/min/sec
        // Simple implementation — for full leap-second awareness use a proper library
        struct tm t = {
            .tm_year = pvt.year - 1900,
            .tm_mon  = pvt.month - 1,
            .tm_mday = pvt.day,
            .tm_hour = pvt.hour,
            .tm_min  = pvt.min,
            .tm_sec  = pvt.sec,
        };
        time_t unix_sec = mktime(&t);  // NOTE: mktime uses local time; set TZ=UTC
        s_fix.unix_time_ms = (int64_t)unix_sec * 1000 + pvt.nano / 1000000;
        s_fix.utc_ms = pvt.iTOW;
    }

    s_fix.valid = fix_ok && time_ok;
    xSemaphoreGive(s_mutex);

    if (fix_ok) {
        ESP_LOGD(TAG, "Fix: lat=%.6f lon=%.6f alt=%.1fm sats=%d",
                 s_fix.lat_deg, s_fix.lon_deg, s_fix.alt_m, pvt.numSV);
    }
}

// ---------------------------------------------------------------------------
//  GPS parser task — runs on Core 0
// ---------------------------------------------------------------------------
static void gps_task(void *arg) {
    configure_m8n();

    // UBX frame parser state machine
    enum { S_SYNC1, S_SYNC2, S_CLASS, S_ID, S_LEN1, S_LEN2,
           S_PAYLOAD, S_CK_A, S_CK_B } state = S_SYNC1;

    uint8_t  cls = 0, id = 0;
    uint16_t payload_len = 0, payload_idx = 0;
    uint8_t  payload[NAV_PVT_LEN + 8];  // max we care about
    uint8_t  ck_a_rx = 0, ck_b_rx = 0;
    uint8_t  byte;

    while (1) {
        int n = uart_hal_read(GPS_UART_PORT, &byte, 1, 100);
        if (n != 1) continue;

        switch (state) {
            case S_SYNC1:  if (byte == UBX_SYNC1) state = S_SYNC2; break;
            case S_SYNC2:  if (byte == UBX_SYNC2) state = S_CLASS; else state = S_SYNC1; break;
            case S_CLASS:  cls = byte; state = S_ID; break;
            case S_ID:     id  = byte; state = S_LEN1; break;
            case S_LEN1:   payload_len  = byte; state = S_LEN2; break;
            case S_LEN2:
                payload_len |= ((uint16_t)byte << 8);
                payload_idx  = 0;
                state = (payload_len > 0) ? S_PAYLOAD : S_CK_A;
                break;
            case S_PAYLOAD:
                if (payload_idx < sizeof(payload)) payload[payload_idx] = byte;
                payload_idx++;
                if (payload_idx >= payload_len) state = S_CK_A;
                break;
            case S_CK_A:  ck_a_rx = byte; state = S_CK_B; break;
            case S_CK_B: {
                ck_b_rx = byte;
                // Verify checksum
                uint8_t ck_a_calc, ck_b_calc;
                uint8_t ck_buf[payload_len + 4];
                ck_buf[0] = cls; ck_buf[1] = id;
                ck_buf[2] = (uint8_t)(payload_len & 0xFF);
                ck_buf[3] = (uint8_t)(payload_len >> 8);
                memcpy(ck_buf + 4, payload, payload_len);
                ubx_checksum(ck_buf, payload_len + 4, &ck_a_calc, &ck_b_calc);

                if (ck_a_rx == ck_a_calc && ck_b_rx == ck_b_calc) {
                    if (cls == UBX_CLASS_NAV && id == UBX_ID_NAV_PVT
                            && payload_len == NAV_PVT_LEN) {
                        parse_nav_pvt(payload);
                    }
                } else {
                    ESP_LOGW(TAG, "UBX checksum error on %02X/%02X", cls, id);
                }
                state = S_SYNC1;
                break;
            }
        }
    }
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

void gps_init(void) {
    s_mutex = xSemaphoreCreateMutex();

    // Configure UART
    uart_cfg_t uart_cfg = {
        .port        = GPS_UART_PORT,
        .tx_pin      = PIN_GPS_TX,
        .rx_pin      = PIN_GPS_RX,
        .baud        = GPS_BAUD,
        .rx_buf_size = 512,
    };
    uart_hal_init(&uart_cfg);

    // Configure 1PPS GPIO interrupt
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_GPS_1PPS),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_GPS_1PPS, pps_isr_handler, NULL);

    xTaskCreate(gps_task, "gps", STACK_GPS, NULL, 4, NULL);
    ESP_LOGI(TAG, "GPS init done");
}

gps_fix_t gps_get_fix(void) {
    gps_fix_t copy;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        copy = s_fix;
        xSemaphoreGive(s_mutex);
    } else {
        memset(&copy, 0, sizeof(copy));
    }
    return copy;
}

int64_t gps_get_utc_ms_interpolated(void) {
    if (!s_fix.valid) return 0;
    int64_t since_pps_us = esp_timer_get_time() - s_pps_esp_us;
    return s_fix.unix_time_ms + (since_pps_us / 1000);
}

bool gps_is_ready(void) {
    return s_fix.valid;
}
