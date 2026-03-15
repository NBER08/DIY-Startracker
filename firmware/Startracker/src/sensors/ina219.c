#include "ina219.h"
#include "../config.h"
#include "../hal/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "ina219";

// INA219 register addresses
#define INA_REG_CONFIG    0x00
#define INA_REG_SHUNT     0x01
#define INA_REG_BUS       0x02
#define INA_REG_POWER     0x03
#define INA_REG_CURRENT   0x04
#define INA_REG_CALIB     0x05

static ina219_data_t     s_data[2]       = {0};
static float             s_shunt_mohm[2] = {10.0f, 100.0f};  // defaults
static SemaphoreHandle_t s_mutex         = NULL;

static uint8_t ch_addr(ina219_channel_t ch) {
    return (ch == INA219_MOTOR) ? ADDR_INA219_MOTOR : ADDR_INA219_LOGIC;
}

static esp_err_t ina_write16(uint8_t addr, uint8_t reg, uint16_t val) {
    uint8_t buf[2] = {(uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    return i2c_hal_write(I2C_BUS_SENSORS, addr, reg, buf, 2);
}

static uint16_t ina_read16(uint8_t addr, uint8_t reg) {
    uint8_t buf[2] = {0};
    i2c_hal_read(I2C_BUS_SENSORS, addr, reg, buf, 2);
    return (uint16_t)((buf[0] << 8) | buf[1]);
}

static void ina219_task(void *arg) {
    while (1) {
        for (int ch = 0; ch < 2; ch++) {
            uint8_t addr = ch_addr((ina219_channel_t)ch);

            // Bus voltage: bits [15:3], LSB = 4 mV
            uint16_t bus_raw   = ina_read16(addr, INA_REG_BUS);
            // Shunt voltage: signed, LSB = 10 µV
            int16_t  shunt_raw = (int16_t)ina_read16(addr, INA_REG_SHUNT);

            float bus_mv    = (float)(bus_raw >> 3) * 4.0f;
            float shunt_uv  = (float)shunt_raw * 10.0f;
            float current_ma = shunt_uv / s_shunt_mohm[ch];

            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                s_data[ch].bus_voltage_mv = bus_mv;
                s_data[ch].current_ma     = current_ma;
                s_data[ch].power_mw       = bus_mv * current_ma / 1000.0f;
                s_data[ch].valid          = true;
                xSemaphoreGive(s_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ina219_init(ina219_channel_t ch, float shunt_mohm) {
    if (!s_mutex) {
        s_mutex = xSemaphoreCreateMutex();
        xTaskCreate(ina219_task, "ina219", STACK_ENV / 2, NULL, 1, NULL);
    }
    s_shunt_mohm[ch] = shunt_mohm;
    uint8_t addr = ch_addr(ch);
    // Reset and configure: 32V range, gain /8 (320 mV), 12-bit, continuous
    ina_write16(addr, INA_REG_CONFIG, 0x399F);

    // Calibration register: Cal = 0.04096 / (current_lsb × shunt_ohm)
    // current_lsb = 1 mA, shunt_ohm = shunt_mohm / 1000
    float shunt_ohm = shunt_mohm / 1000.0f;
    uint16_t cal = (uint16_t)(0.04096f / (0.001f * shunt_ohm));
    ina_write16(addr, INA_REG_CALIB, cal);

    ESP_LOGI(TAG, "INA219 ch%d init (addr=0x%02X shunt=%.0f mΩ)", ch, addr, shunt_mohm);
}

ina219_data_t ina219_get(ina219_channel_t ch) {
    ina219_data_t copy;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    copy = s_data[ch];
    xSemaphoreGive(s_mutex);
    return copy;
}

bool ina219_battery_low(void) {
    return s_data[INA219_MOTOR].valid &&
           s_data[INA219_MOTOR].bus_voltage_mv < (float)BATTERY_UNDERVOLT_MV;
}

bool ina219_motor_jammed(void) {
    return s_data[INA219_MOTOR].valid &&
           s_data[INA219_MOTOR].current_ma > (float)MOTOR_OVERCURRENT_MA;
}
