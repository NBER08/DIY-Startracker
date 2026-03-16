#include "motor.h"
#include <Arduino.h>
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_attr.h"   // for IRAM_ATTR

// ---- Pin assignments ----
#define STEP_PIN  GPIO_NUM_6
#define DIR_PIN   GPIO_NUM_7

// ---- Motor geometry ----
// Change these to match your hardware.
// period_us = (sidereal day in microseconds) / (total steps per revolution)
// total_steps = motor_steps × microstep × gear_ratio
#define MOTOR_STEPS   200     // steps per revolution of the motor shaft
#define MICROSTEP     32      // set on TMC2209 via UART
#define GEAR_RATIO    144     // how many motor turns per one output turn
#define SIDEREAL_US   86164090500ULL   // one sidereal day in microseconds

// Calculated period between steps
#define TOTAL_STEPS   ((uint64_t)MOTOR_STEPS * MICROSTEP * GEAR_RATIO)
#define STEP_PERIOD_US (SIDEREAL_US / TOTAL_STEPS)

// ---- State ----
static gptimer_handle_t timer = NULL;
static volatile uint64_t step_count = 0;

// This is the period the timer is currently using.
// We adjust it slightly when applying IMU correction.
static volatile uint64_t current_period_us = STEP_PERIOD_US;

// -------------------------------------------------------------------------
// THE ISR — this runs every ~93.5ms, triggered by the hardware timer.
//
// IRAM_ATTR puts this function in fast internal RAM so it never
// has to wait for the flash cache. This is what makes timing precise.
//
// Rule: do as little as possible here. Just pulse the pin and leave.
// -------------------------------------------------------------------------
static bool IRAM_ATTR step_isr(gptimer_handle_t t,
                                const gptimer_alarm_event_data_t *e,
                                void *arg) {
    // Pull STEP pin high → TMC2209 registers a step
    gpio_set_level(STEP_PIN, 1);

    // Wait 2 µs — TMC2209 needs at least 1 µs pulse width
    esp_rom_delay_us(2);

    // Pull STEP pin low — step is complete
    gpio_set_level(STEP_PIN, 0);

    step_count++;
    return false;
}

// -------------------------------------------------------------------------
// Public functions
// -------------------------------------------------------------------------

void motor_begin() {
    // Set up STEP and DIR as outputs
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << STEP_PIN) | (1ULL << DIR_PIN);
    io.mode         = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    gpio_set_level(STEP_PIN, 0);
    gpio_set_level(DIR_PIN,  0);   // 0 = forward (eastward tracking)

    // Create a hardware timer at 1 MHz resolution (1 tick = 1 µs)
    gptimer_config_t timer_cfg = {};
    timer_cfg.clk_src       = GPTIMER_CLK_SRC_DEFAULT;
    timer_cfg.direction     = GPTIMER_COUNT_UP;
    timer_cfg.resolution_hz = 1000000;   // 1 MHz
    gptimer_new_timer(&timer_cfg, &timer);

    // Tell the timer which function to call when it fires
    gptimer_event_callbacks_t cbs = {};
    cbs.on_alarm = step_isr;
    gptimer_register_event_callbacks(timer, &cbs, NULL);

    // Set the alarm to fire every STEP_PERIOD_US and automatically reset
    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count                = STEP_PERIOD_US;
    alarm.reload_count               = 0;
    alarm.flags.auto_reload_on_alarm = true;
    gptimer_set_alarm_action(timer, &alarm);

    gptimer_enable(timer);

    Serial.printf("Motor: step period = %llu µs (%.4f Hz)\n",
                  STEP_PERIOD_US,
                  1000000.0 / (double)STEP_PERIOD_US);
}

void motor_start_tracking() {
    step_count = 0;
    gptimer_start(timer);
    Serial.println("Motor: tracking started");
}

void motor_stop() {
    gptimer_stop(timer);
    Serial.println("Motor: stopped");
}

void motor_apply_correction(float correction_deg) {
    // Convert the angular error to a change in step period.
    //
    // If the camera is 0.01° ahead, we need to slow down slightly.
    // We do this by making the timer period a tiny bit longer.
    //
    // This is a simple proportional controller:
    //   error_deg × gain = how many microseconds to add/subtract
    //
    const float gain = 100.0f;  // tune this during field testing
    int64_t adjust = (int64_t)(correction_deg * gain);

    // New period = base period + small adjustment
    int64_t new_period = (int64_t)STEP_PERIOD_US + adjust;

    // Safety clamp — never let it go crazy
    if (new_period < 100000LL)   new_period = 100000LL;   // max 10 Hz
    if (new_period > 300000000LL) new_period = 300000000LL; // min 0.003 Hz

    current_period_us = (uint64_t)new_period;

    // Update the timer — this takes effect on the next alarm cycle
    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count                = current_period_us;
    alarm.reload_count               = 0;
    alarm.flags.auto_reload_on_alarm = true;
    gptimer_set_alarm_action(timer, &alarm);
}
