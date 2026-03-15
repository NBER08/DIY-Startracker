#include "slew.h"
#include "../config.h"
#include "../hal/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "slew";

// ---------------------------------------------------------------------------
//  PCA9535 registers
// ---------------------------------------------------------------------------
#define PCA_REG_OUT0    0x02    // Output port 0 (pins P00-P07)
#define PCA_REG_OUT1    0x03    // Output port 1 (pins P10-P17)
#define PCA_REG_CFG0    0x06    // Direction port 0 (0=output, 1=input)
#define PCA_REG_CFG1    0x07    // Direction port 1

// ---------------------------------------------------------------------------
//  TB6612FNG pin mapping via PCA9535
//  AZ axis uses Port 0, ALT axis uses Port 1
//  Each TB6612FNG channel needs: AIN1, AIN2, BIN1, BIN2, STBY
//
//  Port 0 (AZ):  P00=AIN1, P01=AIN2, P02=BIN1, P03=BIN2, P04=STBY
//  Port 1 (ALT): P10=AIN1, P11=AIN2, P12=BIN1, P13=BIN2, P14=STBY
// ---------------------------------------------------------------------------

// Half-step sequence for one motor winding pair (8 states)
// Each row: {AIN1, AIN2, BIN1, BIN2}
static const uint8_t HALF_STEP[8][4] = {
    {1, 0, 0, 0},
    {1, 0, 1, 0},
    {0, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 0},
    {0, 1, 0, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
};

static int s_step_idx[2] = {0, 0};  // current half-step position per axis

// ---------------------------------------------------------------------------
//  PCA9535 helpers
// ---------------------------------------------------------------------------

static void pca_write_port(uint8_t reg, uint8_t val) {
    i2c_hal_write(I2C_BUS_EXPANDER, ADDR_IO_EXPANDER, reg, &val, 1);
}

static void set_axis_pins(slew_axis_t axis, const uint8_t pins[4], bool stby_on) {
    // Build byte: bits [0..3] = AIN1,AIN2,BIN1,BIN2, bit 4 = STBY
    uint8_t port_val = (stby_on ? (1 << 4) : 0);
    for (int i = 0; i < 4; i++) {
        if (pins[i]) port_val |= (1 << i);
    }
    uint8_t reg = (axis == SLEW_AZ) ? PCA_REG_OUT0 : PCA_REG_OUT1;
    pca_write_port(reg, port_val);
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

void slew_init(void) {
    // Set all PCA9535 pins as outputs
    pca_write_port(PCA_REG_CFG0, 0x00);
    pca_write_port(PCA_REG_CFG1, 0x00);
    // All pins low (motors off, STBY low = disabled)
    pca_write_port(PCA_REG_OUT0, 0x00);
    pca_write_port(PCA_REG_OUT1, 0x00);
    ESP_LOGI(TAG, "Slew init OK");
}

void slew_move_steps(slew_axis_t axis, slew_dir_t dir,
                      uint32_t steps, uint32_t step_delay_ms) {
    int delta = (dir == SLEW_DIR_POSITIVE) ? 1 : -1;

    for (uint32_t i = 0; i < steps; i++) {
        s_step_idx[axis] = (s_step_idx[axis] + delta + 8) % 8;
        set_axis_pins(axis, HALF_STEP[s_step_idx[axis]], true);
        vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    }
}

bool slew_move_to_angle(slew_axis_t axis,
                         double target_deg,
                         double current_deg,
                         double tolerance_deg) {
    double diff = target_deg - current_deg;
    // Normalise diff to [-180, 180]
    while (diff >  180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;

    if (fabs(diff) <= tolerance_deg) return true;

    slew_dir_t dir = (diff > 0) ? SLEW_DIR_POSITIVE : SLEW_DIR_NEGATIVE;

    // Rough step estimate: 200 steps/rev motor, 1/8 gear reduction for slew
    // Tune STEPS_PER_DEG for your specific slew mechanism
    const double STEPS_PER_DEG = 10.0;
    uint32_t steps = (uint32_t)(fabs(diff) * STEPS_PER_DEG);
    if (steps < 1) steps = 1;

    slew_move_steps(axis, dir, steps, 8);
    return false;   // caller should re-check current_deg and call again
}

void slew_stop_all(void) {
    pca_write_port(PCA_REG_OUT0, 0x00);
    pca_write_port(PCA_REG_OUT1, 0x00);
}

void slew_release(slew_axis_t axis) {
    uint8_t reg = (axis == SLEW_AZ) ? PCA_REG_OUT0 : PCA_REG_OUT1;
    pca_write_port(reg, 0x00);  // all low, STBY=0 = coast
}
