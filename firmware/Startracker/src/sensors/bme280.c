#include "bme280.h"
#include "../config.h"
#include "../hal/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>

static const char *TAG = "bme280";

// ---------------------------------------------------------------------------
//  BME280 register addresses
// ---------------------------------------------------------------------------
#define BME_REG_CALIB0   0x88
#define BME_REG_ID       0xD0
#define BME_REG_RESET    0xE0
#define BME_REG_CALIB26  0xE1
#define BME_REG_CTRL_HUM 0xF2
#define BME_REG_STATUS   0xF3
#define BME_REG_CTRL_MEAS 0xF4
#define BME_REG_CONFIG   0xF5
#define BME_REG_PRESS_MSB 0xF7

// ---------------------------------------------------------------------------
//  Calibration data (loaded once on init)
// ---------------------------------------------------------------------------
static struct {
    uint16_t T1; int16_t T2, T3;
    uint16_t P1; int16_t P2,P3,P4,P5,P6,P7,P8,P9;
    uint8_t  H1; int16_t H2; uint8_t H3; int16_t H4,H5; int8_t H6;
    int32_t  t_fine;
} cal;

static bme280_data_t     s_data  = {0};
static SemaphoreHandle_t s_mutex = NULL;

static void load_calibration(void) {
    uint8_t buf[24];
    i2c_hal_read(I2C_BUS_SENSORS, ADDR_BME280, BME_REG_CALIB0, buf, 24);
    cal.T1 = (uint16_t)(buf[1]<<8|buf[0]);
    cal.T2 = (int16_t) (buf[3]<<8|buf[2]);
    cal.T3 = (int16_t) (buf[5]<<8|buf[4]);
    cal.P1 = (uint16_t)(buf[7]<<8|buf[6]);
    cal.P2 = (int16_t) (buf[9]<<8|buf[8]);
    cal.P3 = (int16_t) (buf[11]<<8|buf[10]);
    cal.P4 = (int16_t) (buf[13]<<8|buf[12]);
    cal.P5 = (int16_t) (buf[15]<<8|buf[14]);
    cal.P6 = (int16_t) (buf[17]<<8|buf[16]);
    cal.P7 = (int16_t) (buf[19]<<8|buf[18]);
    cal.P8 = (int16_t) (buf[21]<<8|buf[20]);
    cal.P9 = (int16_t) (buf[23]<<8|buf[22]);
    i2c_hal_read(I2C_BUS_SENSORS, ADDR_BME280, 0xA1, &cal.H1, 1);
    uint8_t h[7];
    i2c_hal_read(I2C_BUS_SENSORS, ADDR_BME280, BME_REG_CALIB26, h, 7);
    cal.H2 = (int16_t)(h[1]<<8|h[0]);
    cal.H3 = h[2];
    cal.H4 = (int16_t)(((int8_t)h[3]<<4) | (h[4]&0x0F));
    cal.H5 = (int16_t)((h[5]<<4) | (h[4]>>4));
    cal.H6 = (int8_t)h[6];
}

// Compensation formulae from BME280 datasheet §4.2.3
static float compensate_temp(int32_t adc_T) {
    int32_t v1 = ((adc_T>>3) - ((int32_t)cal.T1<<1));
    int32_t a  = (v1 * cal.T2) >> 11;
    int32_t v2 = ((adc_T>>4) - (int32_t)cal.T1);
    int32_t b  = (((v2*v2)>>12) * cal.T3) >> 14;
    cal.t_fine = a + b;
    return (float)((cal.t_fine*5+128)>>8) / 100.0f;
}

static float compensate_press(int32_t adc_P) {
    int64_t v1 = (int64_t)cal.t_fine - 128000;
    int64_t v2 = v1*v1*(int64_t)cal.P6 + ((v1*(int64_t)cal.P5)<<17) + ((int64_t)cal.P4<<35);
    v1 = ((v1*v1*(int64_t)cal.P3)>>8) + ((v1*(int64_t)cal.P2)<<12);
    v1 = (((int64_t)1<<47)+v1)*cal.P1>>33;
    if (v1 == 0) return 0;
    int64_t p = 1048576 - adc_P;
    p = (((p<<31)-v2)*3125)/v1;
    v1 = ((int64_t)cal.P9*(p>>13)*(p>>13))>>25;
    v2 = ((int64_t)cal.P8*p)>>19;
    p  = ((p+v1+v2)>>8) + ((int64_t)cal.P7<<4);
    return (float)p / 25600.0f;   // hPa
}

static float compensate_hum(int32_t adc_H) {
    int32_t v = cal.t_fine - 76800;
    v = (((adc_H<<14) - ((int32_t)cal.H4<<20) - ((int32_t)cal.H5*v) + 16384)>>15)
      * (((((((v*(int32_t)cal.H6)>>10)*(((v*(int32_t)cal.H3)>>11)+32768))>>10)+2097152)
         *(int32_t)cal.H2+8192)>>14);
    v = v - (((((v>>15)*(v>>15))>>7)*(int32_t)cal.H1)>>4);
    v = (v < 0) ? 0 : (v > 419430400) ? 419430400 : v;
    return (float)(v>>12) / 1024.0f;
}

static void bme280_task(void *arg) {
    while (1) {
        uint8_t buf[8];
        i2c_hal_read(I2C_BUS_SENSORS, ADDR_BME280, BME_REG_PRESS_MSB, buf, 8);

        int32_t adc_P = ((int32_t)buf[0]<<12)|((int32_t)buf[1]<<4)|(buf[2]>>4);
        int32_t adc_T = ((int32_t)buf[3]<<12)|((int32_t)buf[4]<<4)|(buf[5]>>4);
        int32_t adc_H = ((int32_t)buf[6]<<8)|buf[7];

        float temp  = compensate_temp(adc_T);
        float press = compensate_press(adc_P);
        float hum   = compensate_hum(adc_H);
        // Magnus formula dew point
        float gamma = (17.67f*temp/(243.5f+temp)) + logf(hum/100.0f);
        float dew   = 243.5f*gamma / (17.67f-gamma);

        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            s_data.temperature_c = temp;
            s_data.pressure_hpa  = press;
            s_data.humidity_rh   = hum;
            s_data.dewpoint_c    = dew;
            s_data.valid         = true;
            xSemaphoreGive(s_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 Hz
    }
}

void bme280_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    // Soft reset
    uint8_t v = 0xB6;
    i2c_hal_write(I2C_BUS_SENSORS, ADDR_BME280, BME_REG_RESET, &v, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    load_calibration();
    // Humidity oversampling ×1 (must set before ctrl_meas)
    v = 0x01; i2c_hal_write(I2C_BUS_SENSORS, ADDR_BME280, BME_REG_CTRL_HUM, &v, 1);
    // Temp ×1, Pressure ×1, Normal mode
    v = 0x27; i2c_hal_write(I2C_BUS_SENSORS, ADDR_BME280, BME_REG_CTRL_MEAS, &v, 1);
    xTaskCreate(bme280_task, "bme280", STACK_ENV, NULL, 1, NULL);
    ESP_LOGI(TAG, "BME280 init OK");
}

bme280_data_t bme280_get(void) {
    bme280_data_t copy;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    copy = s_data;
    xSemaphoreGive(s_mutex);
    return copy;
}

bool bme280_dew_warning(void) {
    return s_data.valid && (s_data.temperature_c < s_data.dewpoint_c + 2.0f);
}
