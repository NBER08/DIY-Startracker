#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"

// The only header main.cpp needs
#include "state_machine.h"

static const char *TAG = "main";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Star tracker firmware starting");

    // NVS is required for magnetometer calibration persistence
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition full — erasing");
        nvs_flash_erase();
        nvs_flash_init();
    }

    // This single call boots the entire system.
    // All hardware init, sensor tasks, and the state machine
    // live inside state_machine_init().
    state_machine_init();

    // app_main() can return — FreeRTOS keeps running.
    // Or loop here to print a watchdog heartbeat:
    while (1) {
        system_status_t s = state_machine_get_status();
        const char *state_names[] = {
            "BOOT", "WAITING_GPS", "MAG_CAL", "SLEWING", "TRACKING", "ERROR"
        };
        ESP_LOGI(TAG, "[%s] sats=%d err=%.1f\" steps=%lu bat=%.2fV",
                 state_names[s.state],
                 s.gps_satellites,
                 s.tracking_error_arcsec,
                 (unsigned long)s.steps_since_start,
                 s.motor_rail_mv / 1000.0f);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
