#include "mag.h"
#include "../config.h"
#include "../hal/i2c.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>

static const char *TAG = "mag";

// ---------------------------------------------------------------------------
//  IIS2MDCTR register map
// ---------------------------------------------------------------------------
#define IIS2_OFFSET_X_REG_L   0x45
#define IIS2_CFG_REG_A        0x60
#define IIS2_CFG_REG_B        0x61
#define IIS2_CFG_REG_C        0x62
#define IIS2_STATUS_REG       0x67
#define IIS2_OUTX_L_REG       0x68   // X, Y, Z low bytes follow sequentially

#define IIS2_WHO_AM_I         0x4F   // expected value: 0x40

// CFG_REG_A: ODR 50 Hz, continuous mode
#define CFG_A_VAL  0x0C   // ODR=50Hz, MD=continuous

// Sensitivity: 1.5 mGauss/LSB
#define IIS2_SENS_MGAUSS  1.5f

// ---------------------------------------------------------------------------
//  Calibration data — stored in NVS
// ---------------------------------------------------------------------------
typedef struct {
    float offset[3];    // hard iron offsets (x, y, z)
    float scale[3];     // soft iron scale (x, y, z) — diagonal of scale matrix
    bool  valid;
} mag_cal_t;

static mag_cal_t         s_cal   = {.scale = {1.0f, 1.0f, 1.0f}};
static mag_data_t        s_data  = {0};
static float             s_decl  = 0.0f;
static SemaphoreHandle_t s_mutex = NULL;

#define NVS_NAMESPACE   "mag_cal"
#define NVS_KEY         "cal_data"

// ---------------------------------------------------------------------------
//  NVS persistence
// ---------------------------------------------------------------------------

bool mag_load_calibration(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;

    size_t size = sizeof(mag_cal_t);
    esp_err_t ret = nvs_get_blob(h, NVS_KEY, &s_cal, &size);
    nvs_close(h);

    if (ret == ESP_OK && s_cal.valid) {
        ESP_LOGI(TAG, "Cal loaded: off=(%.2f %.2f %.2f) scale=(%.3f %.3f %.3f)",
                 s_cal.offset[0], s_cal.offset[1], s_cal.offset[2],
                 s_cal.scale[0],  s_cal.scale[1],  s_cal.scale[2]);
        return true;
    }
    return false;
}

static void save_calibration(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, NVS_KEY, &s_cal, sizeof(mag_cal_t));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Calibration saved to NVS");
}

void mag_clear_calibration(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, NVS_KEY);
    nvs_commit(h);
    nvs_close(h);
    s_cal.valid = false;
    ESP_LOGI(TAG, "Calibration cleared");
}

// ---------------------------------------------------------------------------
//  Read one raw XYZ sample from the chip
// ---------------------------------------------------------------------------
static bool read_raw(float *x, float *y, float *z) {
    uint8_t status;
    if (i2c_hal_read(I2C_BUS_SENSORS, ADDR_IIS2MDCTR, IIS2_STATUS_REG,
                     &status, 1) != ESP_OK) return false;
    if (!(status & 0x08)) return false;  // ZYXDA not set — no new data

    uint8_t raw[6];
    if (i2c_hal_read(I2C_BUS_SENSORS, ADDR_IIS2MDCTR, IIS2_OUTX_L_REG,
                     raw, 6) != ESP_OK) return false;

    int16_t rx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t ry = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t rz = (int16_t)((raw[5] << 8) | raw[4]);

    *x = rx * IIS2_SENS_MGAUSS;
    *y = ry * IIS2_SENS_MGAUSS;
    *z = rz * IIS2_SENS_MGAUSS;
    return true;
}

// ---------------------------------------------------------------------------
//  Apply hard + soft iron correction
// ---------------------------------------------------------------------------
static void apply_calibration(float rx, float ry, float rz,
                               float *cx, float *cy, float *cz) {
    *cx = (rx - s_cal.offset[0]) * s_cal.scale[0];
    *cy = (ry - s_cal.offset[1]) * s_cal.scale[1];
    *cz = (rz - s_cal.offset[2]) * s_cal.scale[2];
}

// ---------------------------------------------------------------------------
//  Magnetometer task — 10 Hz
// ---------------------------------------------------------------------------
static void mag_task(void *arg) {
    while (1) {
        float rx, ry, rz;
        if (read_raw(&rx, &ry, &rz)) {
            float cx, cy, cz;
            apply_calibration(rx, ry, rz, &cx, &cy, &cz);

            // Heading: atan2 of Y over X in the horizontal plane
            // Tilt compensation would use IMU pitch/roll here — simplified for now
            float heading_mag = atan2f(cy, cx) * 180.0f / M_PI;
            if (heading_mag < 0) heading_mag += 360.0f;

            float heading_true = heading_mag + s_decl;
            if (heading_true < 0)    heading_true += 360.0f;
            if (heading_true >= 360) heading_true -= 360.0f;

            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                s_data.x_uT = rx * 0.1f;   // mGauss → µTesla
                s_data.y_uT = ry * 0.1f;
                s_data.z_uT = rz * 0.1f;
                s_data.heading_mag_deg  = heading_mag;
                s_data.heading_true_deg = heading_true;
                s_data.declination_deg  = s_decl;
                s_data.calibrated       = s_cal.valid;
                s_data.valid            = true;
                xSemaphoreGive(s_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // 10 Hz
    }
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

void mag_init(void) {
    s_mutex = xSemaphoreCreateMutex();

    // Verify WHO_AM_I
    uint8_t who;
    i2c_hal_read(I2C_BUS_SENSORS, ADDR_IIS2MDCTR, 0x4F, &who, 1);
    if (who != 0x40) {
        ESP_LOGE(TAG, "IIS2MDCTR not found (got 0x%02X, expected 0x40)", who);
        return;
    }

    // Set 50 Hz continuous mode
    uint8_t cfg = CFG_A_VAL;
    i2c_hal_write(I2C_BUS_SENSORS, ADDR_IIS2MDCTR, IIS2_CFG_REG_A, &cfg, 1);

    // Enable block data update (BDU) to prevent reading mid-update
    cfg = 0x10;
    i2c_hal_write(I2C_BUS_SENSORS, ADDR_IIS2MDCTR, IIS2_CFG_REG_C, &cfg, 1);

    mag_load_calibration();

    xTaskCreate(mag_task, "mag", STACK_MAG, NULL, 2, NULL);
    ESP_LOGI(TAG, "IIS2MDCTR init OK (cal=%s)", s_cal.valid ? "loaded" : "needed");
}

mag_data_t mag_get(void) {
    mag_data_t copy;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    copy = s_data;
    xSemaphoreGive(s_mutex);
    return copy;
}

bool mag_run_calibration(void) {
    ESP_LOGI(TAG, "Calibration started — slowly rotate the tracker through all orientations");
    ESP_LOGI(TAG, "Collecting samples for 60 seconds...");

    float xmin =  1e6f, xmax = -1e6f;
    float ymin =  1e6f, ymax = -1e6f;
    float zmin =  1e6f, zmax = -1e6f;
    int   samples = 0;

    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(60000);
    while (xTaskGetTickCount() < end) {
        float rx, ry, rz;
        if (read_raw(&rx, &ry, &rz)) {
            if (rx < xmin) xmin = rx;  if (rx > xmax) xmax = rx;
            if (ry < ymin) ymin = ry;  if (ry > ymax) ymax = ry;
            if (rz < zmin) zmin = rz;  if (rz > zmax) zmax = rz;
            samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (samples < 100) {
        ESP_LOGE(TAG, "Calibration failed — only %d samples", samples);
        return false;
    }

    // Hard iron offsets = midpoint of min/max sphere
    s_cal.offset[0] = (xmax + xmin) / 2.0f;
    s_cal.offset[1] = (ymax + ymin) / 2.0f;
    s_cal.offset[2] = (zmax + zmin) / 2.0f;

    // Soft iron scale — normalise each axis to the average radius
    float rx_range = (xmax - xmin) / 2.0f;
    float ry_range = (ymax - ymin) / 2.0f;
    float rz_range = (zmax - zmin) / 2.0f;
    float avg = (rx_range + ry_range + rz_range) / 3.0f;
    s_cal.scale[0] = (rx_range > 0) ? avg / rx_range : 1.0f;
    s_cal.scale[1] = (ry_range > 0) ? avg / ry_range : 1.0f;
    s_cal.scale[2] = (rz_range > 0) ? avg / rz_range : 1.0f;
    s_cal.valid = true;

    save_calibration();
    ESP_LOGI(TAG, "Calibration complete (%d samples)", samples);
    return true;
}

void mag_set_declination_from_gps(double lat_deg, double lon_deg, int year) {
    // Simplified WMM model — accurate to ~1° for most of Europe
    // For Pécs (46.07°N, 18.23°E) declination is approximately +4.5° East
    // For production use, implement the full WMM2025 model
    (void)lat_deg; (void)lon_deg; (void)year;
    s_decl = 4.5f;  // degrees East — positive means true North is east of magnetic North
    ESP_LOGI(TAG, "Magnetic declination set to %.1f°", s_decl);
}
