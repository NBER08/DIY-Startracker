#include "imu.h"
#include "../config.h"
#include "../hal/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>

static const char *TAG = "imu";

// ---------------------------------------------------------------------------
//  BNO085 SHTP (Sensor Hub Transport Protocol) constants
// ---------------------------------------------------------------------------
#define SHTP_CHANNEL_COMMAND    0
#define SHTP_CHANNEL_EXECUTABLE 1
#define SHTP_CHANNEL_CONTROL    2
#define SHTP_CHANNEL_REPORTS    3

#define SENSOR_REPORTID_ROTATION_VECTOR    0x05
#define SENSOR_REPORTID_GAME_ROTATION_VEC  0x08

// Report IDs for feature configuration
#define SHTP_BASE_TIMESTAMP     0xFB

// ---------------------------------------------------------------------------
//  Module state
// ---------------------------------------------------------------------------
static imu_data_t         s_mount  = {0};
static imu_data_t         s_camera = {0};
static SemaphoreHandle_t  s_mutex  = NULL;

// ---------------------------------------------------------------------------
//  SHTP helpers
// ---------------------------------------------------------------------------

// Read SHTP packet header (4 bytes) and then payload
static esp_err_t shtp_read(uint8_t i2c_addr, uint8_t *buf, uint16_t *out_len) {
    uint8_t hdr[4];
    esp_err_t ret = i2c_hal_read(I2C_BUS_SENSORS, i2c_addr, 0, hdr, 4);
    if (ret != ESP_OK) return ret;

    uint16_t cargo_len = ((uint16_t)hdr[1] << 8 | hdr[0]) & 0x7FFF;
    if (cargo_len < 4 || cargo_len > 256) {
        *out_len = 0;
        return ESP_OK;
    }

    // Re-read including the 4 header bytes as part of full packet
    ret = i2c_hal_read(I2C_BUS_SENSORS, i2c_addr, 0, buf, cargo_len);
    *out_len = cargo_len;
    return ret;
}

// Send a SHTP set-feature-command to enable a rotation vector report
static void enable_rotation_vector(uint8_t addr, uint8_t report_id,
                                    uint32_t interval_us) {
    uint8_t pkt[17] = {0};
    // Header: length (little-endian), channel, sequence
    pkt[0] = 17;
    pkt[1] = 0;
    pkt[2] = SHTP_CHANNEL_CONTROL;
    pkt[3] = 0;     // sequence number (don't care for first send)

    // Payload: Set Feature Command
    pkt[4]  = 0xFD;  // Set Feature Command
    pkt[5]  = report_id;
    pkt[6]  = 0;     // feature flags
    pkt[7]  = 0;     // change sensitivity LSB
    pkt[8]  = 0;     // change sensitivity MSB
    pkt[9]  = (uint8_t)(interval_us & 0xFF);       // report interval LSB
    pkt[10] = (uint8_t)((interval_us >> 8)  & 0xFF);
    pkt[11] = (uint8_t)((interval_us >> 16) & 0xFF);
    pkt[12] = (uint8_t)((interval_us >> 24) & 0xFF);
    // remaining bytes = 0

    // Write directly to device (no register — SHTP uses raw I2C)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, pkt, 17, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);
}

// ---------------------------------------------------------------------------
//  Parse a rotation vector report from the BNO085 payload
// ---------------------------------------------------------------------------
static void parse_rotation_vector(const uint8_t *payload, imu_data_t *out) {
    // Payload bytes after SHTP header:
    // [0] report ID, [1] seq, [2] status, [3] delay, [4..11] quaternion components
    if (payload[4] != SENSOR_REPORTID_ROTATION_VECTOR) return;

    uint8_t status = payload[7] & 0x03;   // accuracy: 0=unreliable 3=high
    out->cal_status = status;

    // Q-point for rotation vector is 14 — divide raw int16 by 2^14
    float scale = 1.0f / (1 << 14);
    int16_t qi = (int16_t)((payload[10] << 8) | payload[9]);
    int16_t qj = (int16_t)((payload[12] << 8) | payload[11]);
    int16_t qk = (int16_t)((payload[14] << 8) | payload[13]);
    int16_t qr = (int16_t)((payload[16] << 8) | payload[15]);

    out->x = qi * scale;
    out->y = qj * scale;
    out->z = qk * scale;
    out->w = qr * scale;

    // Derived Euler angles (for display only)
    // Yaw (Z-axis rotation)
    out->yaw_deg   = atan2f(2.0f*(out->w*out->z + out->x*out->y),
                            1.0f - 2.0f*(out->y*out->y + out->z*out->z))
                   * 180.0f / M_PI;
    // Pitch (Y-axis)
    float sinp = 2.0f*(out->w*out->y - out->z*out->x);
    if (fabsf(sinp) >= 1.0f)
        out->pitch_deg = copysignf(90.0f, sinp);
    else
        out->pitch_deg = asinf(sinp) * 180.0f / M_PI;
    // Roll (X-axis)
    out->roll_deg  = atan2f(2.0f*(out->w*out->x + out->y*out->z),
                            1.0f - 2.0f*(out->x*out->x + out->y*out->y))
                   * 180.0f / M_PI;

    out->valid = true;
}

// ---------------------------------------------------------------------------
//  IMU task — reads both sensors at 100 Hz
// ---------------------------------------------------------------------------
static void imu_task(void *arg) {
    // Enable rotation vector on both sensors at 10 ms interval (100 Hz)
    enable_rotation_vector(ADDR_BNO085_MOUNT,  SENSOR_REPORTID_ROTATION_VECTOR, 10000);
    vTaskDelay(pdMS_TO_TICKS(50));
    enable_rotation_vector(ADDR_BNO085_CAMERA, SENSOR_REPORTID_ROTATION_VECTOR, 10000);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "Both BNO085 sensors enabled");

    uint8_t  buf[64];
    uint16_t pkt_len;

    while (1) {
        // Poll mount sensor
        if (shtp_read(ADDR_BNO085_MOUNT, buf, &pkt_len) == ESP_OK && pkt_len >= 17) {
            if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
                parse_rotation_vector(buf, &s_mount);
                xSemaphoreGive(s_mutex);
            }
        }

        // Poll camera sensor
        if (shtp_read(ADDR_BNO085_CAMERA, buf, &pkt_len) == ESP_OK && pkt_len >= 17) {
            if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
                parse_rotation_vector(buf, &s_camera);
                xSemaphoreGive(s_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz
    }
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

void imu_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    xTaskCreate(imu_task, "imu", STACK_IMU, NULL, 5, NULL);
    ESP_LOGI(TAG, "IMU init done");
}

imu_data_t imu_get_mount(void) {
    imu_data_t copy;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    copy = s_mount;
    xSemaphoreGive(s_mutex);
    return copy;
}

imu_data_t imu_get_camera(void) {
    imu_data_t copy;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    copy = s_camera;
    xSemaphoreGive(s_mutex);
    return copy;
}

double imu_get_tracking_error_arcsec(void) {
    if (!s_mount.valid || !s_camera.valid) return 0.0;
    if (s_mount.cal_status < 2 || s_camera.cal_status < 2) return 0.0;

    // Compute differential quaternion: q_err = q_mount_conjugate ⊗ q_camera
    // q_mount conjugate = (w, -x, -y, -z)
    float mw = s_mount.w, mx = -s_mount.x, my = -s_mount.y, mz = -s_mount.z;
    float cw = s_camera.w, cx = s_camera.x, cy = s_camera.y, cz = s_camera.z;

    float ew = mw*cw - mx*cx - my*cy - mz*cz;
    float ex = mw*cx + mx*cw + my*cz - mz*cy;
    float ey = mw*cy - mx*cz + my*cw + mz*cx;
    float ez = mw*cz + mx*cy - my*cx + mz*cw;

    // The rotation angle of q_err (in radians) is the tracking error
    float angle_rad = 2.0f * acosf(fabsf(ew));

    // Convert to arcseconds (1 rad = 206265 arcsec)
    // Sign: use the z-component of the error axis
    float sign = (ez >= 0.0f) ? 1.0f : -1.0f;
    return (double)(sign * angle_rad * 206265.0f);
}

double imu_get_tilt_deg(void) {
    imu_data_t m = imu_get_mount();
    if (!m.valid) return 0.0;
    // Tilt from horizontal = magnitude of pitch² + roll²
    return sqrt(m.pitch_deg * m.pitch_deg + m.roll_deg * m.roll_deg);
}

bool imu_is_calibrated(void) {
    return (s_mount.valid  && s_mount.cal_status  >= 2 &&
            s_camera.valid && s_camera.cal_status >= 2);
}
