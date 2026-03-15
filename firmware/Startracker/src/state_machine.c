#include "state_machine.h"
#include "config.h"
#include "hal/i2c.h"
#include "hal/uart.h"
#include "hal/spi.h"
#include "sensors/gps.h"
#include "sensors/imu.h"
#include "sensors/mag.h"
#include "sensors/bme280.h"
#include "sensors/ina219.h"
#include "astro/astro_math.h"
#include "motor/tracking.h"
#include "motor/slew.h"
#include "comms/lora.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "state_machine";

// ---------------------------------------------------------------------------
//  Module state
// ---------------------------------------------------------------------------
static system_status_t   s_status   = {0};
static SemaphoreHandle_t s_mutex    = NULL;
static bool              s_skip_cal = false;
static camera_params_t   s_cam_params = {
    .focus_delay_ms = 300,
    .exposure_ms    = 0,
    .burst_count    = 1,
    .interval_ms    = 500,
};

// ---------------------------------------------------------------------------
//  LED helpers
// ---------------------------------------------------------------------------
static void led_set(bool on) {
    gpio_set_level(PIN_DEBUG_LED, on ? 1 : 0);
}

static void led_blink(int times, int period_ms) {
    for (int i = 0; i < times; i++) {
        led_set(true);
        vTaskDelay(pdMS_TO_TICKS(period_ms / 2));
        led_set(false);
        vTaskDelay(pdMS_TO_TICKS(period_ms / 2));
    }
}

// ---------------------------------------------------------------------------
//  Build telemetry packet from current status
// ---------------------------------------------------------------------------
static lora_telemetry_packet_t build_telemetry(void) {
    lora_telemetry_packet_t pkt = {0};
    pkt.magic             = 0xCD;
    pkt.state             = (uint8_t)s_status.state;
    pkt.alt_deg_x100      = (int16_t)(s_status.pole_alt_deg  * 100.0);
    pkt.az_deg_x100       = (int16_t)(s_status.pole_az_deg   * 100.0);
    pkt.tracking_err_asec = (int16_t)(s_status.tracking_error_arcsec * 10.0);
    pkt.temperature_x10   = (int16_t)(s_status.temperature_c * 10.0f);
    pkt.motor_mv          = (uint16_t)s_status.motor_rail_mv;
    pkt.motor_ma          = (uint16_t)s_status.motor_current_ma;
    pkt.gps_sats          = s_status.gps_satellites;
    pkt.flags             = (s_status.dew_warning   ? 0x01 : 0)
                          | (s_status.battery_low   ? 0x02 : 0)
                          | (s_status.motor_jammed  ? 0x04 : 0);
    // CRC over all bytes except the last two
    uint16_t crc = 0xFFFF;
    const uint8_t *b = (const uint8_t *)&pkt;
    for (size_t i = 0; i < sizeof(pkt) - 2; i++) {
        crc ^= (uint16_t)b[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    pkt.crc = crc;
    return pkt;
}

// ---------------------------------------------------------------------------
//  Camera shutter control
// ---------------------------------------------------------------------------
static void trigger_camera(const camera_params_t *p) {
    for (uint16_t frame = 0; frame < p->burst_count; frame++) {
        // Focus
        gpio_set_level(PIN_CAMERA_FOCUS, 1);
        vTaskDelay(pdMS_TO_TICKS(p->focus_delay_ms));

        // Shutter
        gpio_set_level(PIN_CAMERA_SHUTTER, 1);
        if (p->exposure_ms == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));   // single-shot: brief pulse
        } else {
            vTaskDelay(pdMS_TO_TICKS(p->exposure_ms));
        }
        gpio_set_level(PIN_CAMERA_SHUTTER, 0);
        gpio_set_level(PIN_CAMERA_FOCUS,   0);

        if (frame < (uint16_t)(p->burst_count - 1)) {
            vTaskDelay(pdMS_TO_TICKS(p->interval_ms));
        }
    }
}

// ---------------------------------------------------------------------------
//  State handlers — each returns the next state
// ---------------------------------------------------------------------------

static system_state_t handle_boot(void) {
    ESP_LOGI(TAG, "STATE: BOOT");
    led_blink(3, 200);

    // Initialise HAL
    ESP_ERROR_CHECK(i2c_hal_init(I2C_BUS_SENSORS));
    ESP_ERROR_CHECK(i2c_hal_init(I2C_BUS_EXPANDER));
    ESP_ERROR_CHECK(spi_hal_init());

    // Scan buses during bring-up — remove in production
    i2c_hal_scan(I2C_BUS_SENSORS);

    // Initialise sensors
    gps_init();
    imu_init();
    mag_init();
    bme280_init();
    ina219_init(INA219_MOTOR, 10.0f);   // 10 mΩ shunt on motor rail
    ina219_init(INA219_LOGIC, 100.0f);  // 100 mΩ shunt on logic rail

    // Initialise motors
    slew_init();
    if (!tracking_init()) {
        state_machine_fault("TMC2209 not found");
        return STATE_ERROR;
    }

    // Initialise LoRa
    lora_init();   // non-fatal if it fails — continue without wireless

    // Configure camera pins as outputs
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<PIN_CAMERA_FOCUS)|(1ULL<<PIN_CAMERA_SHUTTER)|(1ULL<<PIN_DEBUG_LED),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_CAMERA_FOCUS,   0);
    gpio_set_level(PIN_CAMERA_SHUTTER, 0);

    ESP_LOGI(TAG, "BOOT complete");
    return STATE_WAITING_FOR_GPS;
}

static system_state_t handle_waiting_for_gps(void) {
    ESP_LOGI(TAG, "STATE: WAITING_FOR_GPS");
    static int log_counter = 0;

    // Slow LED blink while waiting
    led_blink(1, 1000);

    gps_fix_t fix = gps_get_fix();

    if (++log_counter % 5 == 0) {
        ESP_LOGI(TAG, "Waiting for GPS... sats=%d valid=%d",
                 fix.satellites, fix.valid);
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        s_status.gps_satellites = fix.satellites;
        s_status.gps_valid = fix.valid;
        xSemaphoreGive(s_mutex);
    }

    if (!gps_is_ready()) return STATE_WAITING_FOR_GPS;

    // First fix — compute pole direction and set magnetic declination
    s_status.lat_deg = fix.lat_deg;
    s_status.lon_deg = fix.lon_deg;
    s_status.utc_ms  = fix.unix_time_ms;

    astro_get_pole_direction(fix.lat_deg, fix.lon_deg, fix.unix_time_ms,
                             &s_status.pole_alt_deg, &s_status.pole_az_deg);

    mag_set_declination_from_gps(fix.lat_deg, fix.lon_deg, 2025);

    char alt_str[20], az_str[20];
    astro_deg_to_dms(s_status.pole_alt_deg, alt_str, sizeof(alt_str));
    astro_deg_to_dms(s_status.pole_az_deg,  az_str,  sizeof(az_str));
    ESP_LOGI(TAG, "GPS fix! Pole direction: Alt=%s  Az=%s", alt_str, az_str);

    return s_skip_cal ? STATE_SLEWING_TO_POLE : STATE_MAG_CALIBRATE;
}

static system_state_t handle_mag_calibrate(void) {
    ESP_LOGI(TAG, "STATE: MAG_CALIBRATE");

    // If valid calibration already in NVS, offer to skip
    if (mag_load_calibration()) {
        ESP_LOGI(TAG, "Existing mag cal found — skipping calibration");
        return STATE_SLEWING_TO_POLE;
    }

    // Run calibration — user must rotate the tracker
    led_blink(5, 200);
    if (!mag_run_calibration()) {
        // Calibration failed or insufficient motion — try to continue anyway
        ESP_LOGW(TAG, "Mag calibration failed, proceeding without cal");
    }
    return STATE_SLEWING_TO_POLE;
}

static system_state_t handle_slewing(void) {
    ESP_LOGI(TAG, "STATE: SLEWING_TO_POLE");

    // Refresh GPS time before computing pole direction
    gps_fix_t fix = gps_get_fix();
    int64_t   utc = gps_get_utc_ms_interpolated();
    double target_alt, target_az;
    astro_get_pole_direction(fix.lat_deg, fix.lon_deg, utc,
                             &target_alt, &target_az);

    const double TOLERANCE_DEG = 0.5;
    bool az_done  = false;
    bool alt_done = false;
    int  iterations = 0;

    while ((!az_done || !alt_done) && iterations < 200) {
        mag_data_t  mag = mag_get();
        double      tilt = imu_get_tilt_deg();

        // Check for faults mid-slew
        if (ina219_battery_low() || ina219_motor_jammed()) {
            slew_stop_all();
            state_machine_fault(ina219_battery_low() ? "Battery low" : "Motor jammed");
            return STATE_ERROR;
        }

        // Move azimuth toward target
        if (!az_done) {
            az_done = slew_move_to_angle(SLEW_AZ, target_az,
                                          mag.heading_true_deg, TOLERANCE_DEG);
        }
        // Move altitude toward target
        if (!alt_done) {
            alt_done = slew_move_to_angle(SLEW_ALT, target_alt,
                                           90.0 - tilt, TOLERANCE_DEG);
        }

        led_set(iterations % 4 < 2);   // fast blink during slew
        iterations++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    slew_release(SLEW_AZ);
    slew_release(SLEW_ALT);

    ESP_LOGI(TAG, "Slew complete (iterations=%d)", iterations);
    return STATE_TRACKING;
}

static system_state_t handle_tracking(void) {
    static uint32_t telem_counter = 0;

    tracking_start();
    led_set(true);   // solid LED = tracking
    ESP_LOGI(TAG, "STATE: TRACKING");

    while (1) {
        // Check for LoRa commands
        lora_cmd_packet_t cmd_pkt;
        lora_cmd_id_t cmd = lora_get_command(&cmd_pkt);
        if (cmd == CMD_STOP_TRACKING || cmd == CMD_REALIGN) {
            tracking_stop();
            return (cmd == CMD_REALIGN) ? STATE_SLEWING_TO_POLE : STATE_ERROR;
        }
        if (cmd == CMD_RATE_SIDEREAL)  tracking_set_rate(TRACK_RATE_SIDEREAL);
        if (cmd == CMD_RATE_LUNAR)     tracking_set_rate(TRACK_RATE_LUNAR);
        if (cmd == CMD_RATE_SOLAR)     tracking_set_rate(TRACK_RATE_SOLAR);
        if (cmd == CMD_SHUTTER_SINGLE || cmd == CMD_SHUTTER_BURST) {
            if (cmd == CMD_SHUTTER_BURST) {
                s_cam_params.burst_count    = cmd_pkt.param_a;
                s_cam_params.exposure_ms    = cmd_pkt.param_b;
            }
            trigger_camera(&s_cam_params);
        }
        if (cmd == CMD_PING) lora_send_pong(lora_get_last_rssi());

        // Closed-loop IMU correction
        double err = imu_get_tracking_error_arcsec();
        tracking_apply_imu_correction(err);

        // Fault detection
        if (ina219_battery_low()) { state_machine_fault("Battery low"); return STATE_ERROR; }
        if (ina219_motor_jammed()) { state_machine_fault("Motor jammed"); return STATE_ERROR; }

        // Update status struct
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            bme280_data_t env = bme280_get();
            ina219_data_t pwr = ina219_get(INA219_MOTOR);
            gps_fix_t     fix = gps_get_fix();

            s_status.tracking_error_arcsec = err;
            s_status.steps_since_start     = (uint32_t)sidereal_timer_get_step_count();
            s_status.temperature_c         = env.temperature_c;
            s_status.pressure_hpa          = env.pressure_hpa;
            s_status.dew_warning           = bme280_dew_warning();
            s_status.motor_rail_mv         = pwr.bus_voltage_mv;
            s_status.motor_current_ma      = pwr.current_ma;
            s_status.battery_low           = ina219_battery_low();
            s_status.motor_jammed          = ina219_motor_jammed();
            s_status.gps_satellites        = fix.satellites;
            s_status.last_rssi_dbm         = lora_get_last_rssi();
            xSemaphoreGive(s_mutex);
        }

        // Send telemetry every 5 seconds
        if (++telem_counter >= 50) {
            telem_counter = 0;
            lora_telemetry_packet_t pkt = build_telemetry();
            lora_send_telemetry(&pkt);
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // 10 Hz tick
    }
    return STATE_TRACKING;  // unreachable, but satisfies compiler
}

static system_state_t handle_error(void) {
    tracking_stop();
    slew_stop_all();
    led_set(false);

    ESP_LOGE(TAG, "STATE: ERROR — %s",
             s_status.error_reason ? s_status.error_reason : "unknown");

    // Wait for a REALIGN command to recover
    while (1) {
        led_blink(2, 300);
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (lora_get_command(NULL) == CMD_REALIGN) {
            s_status.error_reason = NULL;
            return STATE_BOOT;
        }
    }
}

// ---------------------------------------------------------------------------
//  State machine task
// ---------------------------------------------------------------------------
static void state_machine_task(void *arg) {
    s_status.state = STATE_BOOT;

    while (1) {
        system_state_t next;
        switch (s_status.state) {
            case STATE_BOOT:            next = handle_boot();           break;
            case STATE_WAITING_FOR_GPS: next = handle_waiting_for_gps(); break;
            case STATE_MAG_CALIBRATE:   next = handle_mag_calibrate();  break;
            case STATE_SLEWING_TO_POLE: next = handle_slewing();        break;
            case STATE_TRACKING:        next = handle_tracking();       break;
            case STATE_ERROR:           next = handle_error();          break;
            default:                    next = STATE_ERROR;             break;
        }

        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            s_status.state = next;
            xSemaphoreGive(s_mutex);
        }
    }
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

void state_machine_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    // Pin the state machine task to Core 0; tracking ISR runs on Core 1
    xTaskCreatePinnedToCore(state_machine_task, "state_machine",
                            STACK_STATE_MACHINE, NULL, 3, NULL, 0);
    ESP_LOGI(TAG, "State machine task started on Core 0");
}

system_status_t state_machine_get_status(void) {
    system_status_t copy;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    copy = s_status;
    xSemaphoreGive(s_mutex);
    return copy;
}

system_state_t state_machine_get_state(void) {
    return s_status.state;
}

void state_machine_fault(const char *reason) {
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_status.state        = STATE_ERROR;
        s_status.error_reason = reason;
        xSemaphoreGive(s_mutex);
    }
    ESP_LOGE(TAG, "FAULT: %s", reason ? reason : "unknown");
}

void state_machine_trigger_camera(const camera_params_t *params) {
    s_cam_params = *params;
    trigger_camera(&s_cam_params);
}

void state_machine_skip_mag_cal(void) {
    s_skip_cal = true;
}
