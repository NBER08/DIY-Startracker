#include "tracking.h"
#include "../config.h"
#include "../hal/uart.h"
#include "../hal/gptimer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "tracking";

// ---------------------------------------------------------------------------
//  TMC2209 UART protocol
//  Single-wire half-duplex.  Sync byte 0x05 always starts a datagram.
// ---------------------------------------------------------------------------
#define TMC_SYNC        0x05
#define TMC_SLAVE_ADDR  TMC_UART_ADDR

// Register addresses
#define REG_GCONF       0x00
#define REG_IHOLD_IRUN  0x10
#define REG_CHOPCONF    0x6C
#define REG_PWMCONF     0x70
#define REG_DRV_STATUS  0x6F

static uint8_t s_seq = 0;  // write datagram sequence counter

// ---------------------------------------------------------------------------
//  CRC8 for TMC2209 UART datagrams
// ---------------------------------------------------------------------------
static uint8_t tmc_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc ^ b) & 0x80) crc = (crc << 1) ^ 0x07;
            else                   crc <<= 1;
            b <<= 1;
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
//  Write a 32-bit register via UART
// ---------------------------------------------------------------------------
static void tmc_write_reg(uint8_t reg, uint32_t val) {
    uint8_t pkt[8] = {
        TMC_SYNC,
        TMC_SLAVE_ADDR,
        (uint8_t)(reg | 0x80),          // bit 7 = write
        (uint8_t)((val >> 24) & 0xFF),
        (uint8_t)((val >> 16) & 0xFF),
        (uint8_t)((val >>  8) & 0xFF),
        (uint8_t)( val        & 0xFF),
        0,                              // CRC placeholder
    };
    pkt[7] = tmc_crc8(pkt, 7);
    uart_hal_write(TMC_UART_PORT, pkt, 8);
    s_seq++;
    vTaskDelay(pdMS_TO_TICKS(2));   // brief pause between commands
}

// ---------------------------------------------------------------------------
//  Read a 32-bit register via UART
//  Send 4-byte read request, receive 8-byte reply datagram.
// ---------------------------------------------------------------------------
static uint32_t tmc_read_reg(uint8_t reg) {
    uart_hal_flush_rx(TMC_UART_PORT);

    uint8_t req[4] = {
        TMC_SYNC, TMC_SLAVE_ADDR, reg & 0x7F, 0
    };
    req[3] = tmc_crc8(req, 3);
    uart_hal_write(TMC_UART_PORT, req, 4);

    uint8_t reply[8] = {0};
    int n = uart_hal_read(TMC_UART_PORT, reply, 8, 20);
    if (n != 8) return 0xFFFFFFFF;

    uint8_t expected_crc = tmc_crc8(reply, 7);
    if (reply[7] != expected_crc) {
        ESP_LOGW(TAG, "TMC2209 CRC mismatch on reg 0x%02X", reg);
        return 0xFFFFFFFF;
    }

    return ((uint32_t)reply[3] << 24) | ((uint32_t)reply[4] << 16)
         | ((uint32_t)reply[5] <<  8) | reply[6];
}

// ---------------------------------------------------------------------------
//  Configure the TMC2209 on startup
// ---------------------------------------------------------------------------
static bool configure_tmc2209(void) {
    // GCONF: enable PDN_DISABLE (UART mode), I_scale_analog=0
    tmc_write_reg(REG_GCONF, 0x00000040);   // bit 6 = pdn_disable

    // IHOLD_IRUN: IHOLD=8 (hold current), IRUN=20 (~1.2A RMS), IHOLDDELAY=6
    // irun formula: IRUN = (RMS_current / VREF × 32) - 1
    // At VREF=~0.9V internal ref: 1200mA → IRUN ≈ 20
    tmc_write_reg(REG_IHOLD_IRUN, (6 << 16) | (20 << 8) | 8);

    // CHOPCONF: microstep = 1/16 (mres=3), TBL=1, TOFF=3
    // mres encoding: 0=256, 1=128, 2=64, 3=32, 4=16, 5=8, 6=4, 7=2, 8=full
    // We want 1/16 → mres=4
    tmc_write_reg(REG_CHOPCONF, 0x10000053 | (4 << 24));

    // PWMCONF: enable stealthChop (default values are fine)
    tmc_write_reg(REG_PWMCONF, 0xC10D0024);

    // Readback GCONF to verify UART communication is working
    uint32_t gconf = tmc_read_reg(REG_GCONF);
    if (gconf == 0xFFFFFFFF) {
        ESP_LOGE(TAG, "TMC2209 not responding — check wiring");
        return false;
    }
    ESP_LOGI(TAG, "TMC2209 OK (GCONF=0x%08lX)", gconf);
    return true;
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

bool tracking_init(void) {
    // Initialise UART for TMC2209
    uart_cfg_t uart_cfg = {
        .port        = TMC_UART_PORT,
        .tx_pin      = PIN_TMC_TX,
        .rx_pin      = PIN_TMC_RX,
        .baud        = 115200,
        .rx_buf_size = 256,
    };
    uart_hal_init(&uart_cfg);

    // Initialise the GPTimer step ISR
    ESP_ERROR_CHECK(sidereal_timer_init());

    // Configure chip over UART
    return configure_tmc2209();
}

void tracking_start(void) {
    // Set direction pin for normal (eastward) tracking
    gpio_set_level(PIN_TMC_DIR, 0);
    sidereal_timer_reset_count();
    sidereal_timer_start();
    ESP_LOGI(TAG, "Tracking started");
}

void tracking_stop(void) {
    sidereal_timer_stop();
    // Reduce to hold current to save power and reduce heat
    tmc_write_reg(REG_IHOLD_IRUN, (6 << 16) | (8 << 8) | 4);
    ESP_LOGI(TAG, "Tracking stopped");
}

void tracking_set_rate(track_rate_t rate) {
    sidereal_timer_set_rate(rate);
}

void tracking_apply_imu_correction(double error_arcsec) {
    // Convert arcsecond error to PPM correction
    // 1 arcsec error = need to adjust rate by 1/86164 of full revolution ≈ 11.6 ppm
    // We apply a fraction of this to avoid over-correction (proportional controller)
    const double Kp = 5.0;  // gain — tune this during field testing
    int32_t ppm = (int32_t)(error_arcsec * Kp);

    // Clamp to ±500 ppm (about ±43 arcseconds/hour correction)
    if (ppm >  500) ppm =  500;
    if (ppm < -500) ppm = -500;

    sidereal_timer_apply_ppm_correction(ppm);
}

void tracking_set_direction_reversed(bool reversed) {
    gpio_set_level(PIN_TMC_DIR, reversed ? 1 : 0);
}

tmc2209_status_t tracking_get_motor_status(void) {
    tmc2209_status_t status = {0};
    uint32_t drv = tmc_read_reg(REG_DRV_STATUS);
    status.driver_status    = (uint16_t)(drv & 0xFFFF);
    status.overtemp_warning = (drv >> 26) & 1;
    status.overtemp_shutdown= (drv >> 25) & 1;
    status.open_load_a      = (drv >> 18) & 1;
    status.open_load_b      = (drv >> 19) & 1;
    status.stall_detected   = (drv >> 24) & 1;
    status.estimated_load   = (float)((drv >> 10) & 0x1FF) / 511.0f;
    return status;
}
