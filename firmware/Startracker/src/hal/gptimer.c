#include "gptimer.h"
#include "../config.h"
#include "esp_log.h"
#include "esp_attr.h"       // IRAM_ATTR, DRAM_ATTR
#include "driver/gpio.h"
#include "driver/gptimer.h"

static const char *TAG = "gptimer";

// ---------------------------------------------------------------------------
//  Step period calculation
//  total_steps = MOTOR_STEPS_PER_REV × MICROSTEP_DIVISOR × GEAR_RATIO
//  period_us   = sidereal_day_us / total_steps
// ---------------------------------------------------------------------------

#define TOTAL_STEPS_PER_REV \
    ((uint64_t)MOTOR_STEPS_PER_REV * MICROSTEP_DIVISOR * GEAR_RATIO)

static const uint64_t RATE_PERIOD_US[3] = {
    // SIDEREAL: 86164.0905 s
    (uint64_t)(SIDEREAL_DAY_S * 1e6) / TOTAL_STEPS_PER_REV,
    // LUNAR: 89309.4 s (sidereal day + lunar drift correction)
    (uint64_t)(LUNAR_DAY_S   * 1e6) / TOTAL_STEPS_PER_REV,
    // SOLAR: 86400.0 s
    (uint64_t)(SOLAR_DAY_S   * 1e6) / TOTAL_STEPS_PER_REV,
};

// ---------------------------------------------------------------------------
//  State — DRAM_ATTR so the ISR can access them without flash cache
// ---------------------------------------------------------------------------
static gptimer_handle_t  s_timer    = NULL;
static volatile uint64_t DRAM_ATTR  s_step_count  = 0;
static volatile uint64_t DRAM_ATTR  s_period_us   = 0;
static volatile bool     DRAM_ATTR  s_running      = false;

// ---------------------------------------------------------------------------
//  The ISR — every attribute here is mandatory
// ---------------------------------------------------------------------------
static bool IRAM_ATTR on_sidereal_alarm(
        gptimer_handle_t timer,
        const gptimer_alarm_event_data_t *edata,
        void *user_ctx)
{
    (void)timer; (void)edata; (void)user_ctx;

    // Pulse STEP pin — TMC2209 needs ≥1 µs high time
    gpio_set_level(PIN_TMC_STEP, 1);
    esp_rom_delay_us(2);
    gpio_set_level(PIN_TMC_STEP, 0);

    s_step_count++;

    // false = no need to yield to a higher-priority FreeRTOS task
    return false;
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

esp_err_t sidereal_timer_init(void) {
    // Configure STEP and DIR pins as outputs
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_TMC_STEP) | (1ULL << PIN_TMC_DIR),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(PIN_TMC_STEP, 0);
    gpio_set_level(PIN_TMC_DIR,  0);  // 0 = tracking direction (east)

    // Set default rate
    s_period_us = RATE_PERIOD_US[TRACK_RATE_SIDEREAL];

    // Create GPTimer at 1 MHz (1 µs resolution)
    gptimer_config_t timer_cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,   // 1 MHz → 1 µs per tick
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &s_timer));

    // Register alarm callback (our ISR)
    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_sidereal_alarm,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_timer, &cbs, NULL));

    // Set alarm to fire at s_period_us and auto-reload
    gptimer_alarm_config_t alarm = {
        .alarm_count               = s_period_us,
        .reload_count              = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_timer, &alarm));
    ESP_ERROR_CHECK(gptimer_enable(s_timer));

    ESP_LOGI(TAG, "Sidereal timer ready — period=%" PRIu64 " µs (%.4f Hz)",
             s_period_us, 1e6 / (double)s_period_us);
    return ESP_OK;
}

void sidereal_timer_start(void) {
    if (!s_running) {
        s_step_count = 0;
        s_running = true;
        ESP_ERROR_CHECK(gptimer_start(s_timer));
        ESP_LOGI(TAG, "Tracking started");
    }
}

void sidereal_timer_stop(void) {
    if (s_running) {
        ESP_ERROR_CHECK(gptimer_stop(s_timer));
        s_running = false;
        ESP_LOGI(TAG, "Tracking stopped at step %" PRIu64, s_step_count);
    }
}

void sidereal_timer_set_rate(track_rate_t rate) {
    if (rate == TRACK_RATE_STOPPED) {
        sidereal_timer_stop();
        return;
    }
    s_period_us = RATE_PERIOD_US[rate];
    gptimer_alarm_config_t alarm = {
        .alarm_count               = s_period_us,
        .reload_count              = 0,
        .flags.auto_reload_on_alarm = true,
    };
    // Safe to call while timer is running
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_timer, &alarm));
    ESP_LOGI(TAG, "Rate changed to %d (period=%" PRIu64 " µs)", rate, s_period_us);
}

void sidereal_timer_apply_ppm_correction(int32_t ppm) {
    // Positive ppm → shorter period → faster stepping
    // Negative ppm → longer period → slower stepping
    int64_t base = (int64_t)RATE_PERIOD_US[TRACK_RATE_SIDEREAL];
    int64_t corrected = base - (base * ppm / 1000000LL);
    if (corrected < 100) corrected = 100;  // safety clamp: never < 100 µs

    s_period_us = (uint64_t)corrected;
    gptimer_alarm_config_t alarm = {
        .alarm_count               = s_period_us,
        .reload_count              = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(s_timer, &alarm);
}

uint64_t sidereal_timer_get_step_count(void) {
    return s_step_count;
}

void sidereal_timer_reset_count(void) {
    s_step_count = 0;
}
