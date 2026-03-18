#include "motor.h"
#include "config.h"
#include <Arduino.h>
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_attr.h"   // for IRAM_ATTR

#define SIDEREAL_US   86164090500ULL   // one sidereal day in microseconds

// Calculated period between steps
#define TOTAL_STEPS   ((uint64_t)MOTOR_STEPS * MICROSTEP * GEAR_RATIO)
#define STEP_PERIOD_US (SIDEREAL_US / TOTAL_STEPS)

// ---- State ----
static hw_timer_t       *step_timer = NULL;
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
void IRAM_ATTR step_isr() {
    // Pulse STEP pin HIGH → TMC2209 registers one microstep
    digitalWrite(STEP_PIN, HIGH);

    // TMC2209 needs the pulse to be at least 1 µs wide.
    // ets_delay_us() is a busy-wait that is safe to call from an ISR.
    // (Don't use Arduino's delayMicroseconds() inside an ISR — it can
    //  behave unpredictably because it relies on interrupts itself.)
    ets_delay_us(200);

    // Pull LOW — step is complete
    digitalWrite(STEP_PIN, LOW);

    step_count++;
}

// -------------------------------------------------------------------------
// Public functions
// -------------------------------------------------------------------------

void motor_begin() {
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN,  OUTPUT);
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN,  LOW);   // LOW = forward (eastward tracking)

    // Create a timer running at 1 MHz (1 tick = 1 µs).
    // timerBegin(frequency_hz) is the Arduino ESP32 core 3.x API.
    // If you are on core 2.x the call is: timerBegin(0, 80, true)
    // where 0=timer number, 80=divider (80MHz/80=1MHz), true=count up.
    step_timer = timerBegin(1000000);

    // Connect the ISR function to this timer
    timerAttachInterrupt(step_timer, &step_isr);

    // Set the alarm: fire every STEP_PERIOD_US ticks, repeat forever.
    // timerAlarm(timer, ticks_until_alarm, repeat, ticks_to_reload_to)
    timerAlarm(step_timer, STEP_PERIOD_US, true, 0);

    // timerAlarm() starts the timer — stop it immediately.
    // We only want it running when the state machine says TRACKING.
    timerStop(step_timer);

    Serial.printf("Motor ready: period=%llu µs  (%.4f Hz)\n",
                  STEP_PERIOD_US, 1000000.0 / (double)STEP_PERIOD_US);
}

void motor_start_tracking() {
    step_count = 0;
    timerStart(step_timer);
    Serial.println("Motor: tracking started");
}

void motor_stop() {
    timerStop(step_timer);
    Serial.println("Motor: stopped");
}