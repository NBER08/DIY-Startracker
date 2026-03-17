// =============================================================================
//  timer_test.cpp — sidereal ISR accuracy test
//
//  What this does:
//    - Toggles PIN 6 at the exact sidereal stepping frequency (~5.35 Hz)
//    - Runs fake "load" tasks on the main loop (string ops, math, serial prints)
//    - Every 5 seconds, reports how many steps fired vs how many should have
//
//  How to verify it's working:
//    - Connect a logic analyser or oscilloscope to PIN 6
//    - Period should be ~186,941 µs (± a few µs)
//    - The step count in serial should stay in sync even while load is running
//
//  Flash to your ESP32-S3 devkit, open serial monitor at 115200 baud.
// =============================================================================

#include <Arduino.h>
#include <math.h>

// ---- The pin we toggle ----
// Connect scope/logic analyser here to measure the actual frequency
#define STEP_PIN   6

// ---- Sidereal math ----
#define MOTOR_STEPS    200
#define MICROSTEP      64
#define GEAR_RATIO     144
#define SIDEREAL_US    86164090500ULL

#define TOTAL_STEPS    ((uint64_t)MOTOR_STEPS * MICROSTEP * GEAR_RATIO)
#define STEP_PERIOD_US (SIDEREAL_US / TOTAL_STEPS)   // 186,941 µs

// ---- ISR state ----
static hw_timer_t       *step_timer  = NULL;
static volatile uint64_t step_count  = 0;
static volatile uint64_t isr_late_us = 0;   // rough jitter tracking

// =============================================================================
//  The ISR — toggles the pin and counts steps
// =============================================================================
void IRAM_ATTR step_isr() {
    digitalWrite(STEP_PIN, HIGH);
    ets_delay_us(200);
    digitalWrite(STEP_PIN, LOW);
    step_count++;
}

// =============================================================================
//  Simulated load functions
//  These run on the main loop to stress the system while the ISR fires.
//  The point: does the step count stay accurate even when the CPU is busy?
// =============================================================================

// Load 1: heavy floating point math (similar to astro calculations)
static void load_math() {
    volatile double result = 0;
    for (int i = 0; i < 500; i++) {
        result += sin(i * 0.01) * cos(i * 0.02) * sqrt(i + 1.0);
    }
    (void)result;
}

//  Load 2: string formatting (similar to building serial/LoRa messages)
static void load_string() {
    char buf[128];
    for (int i = 0; i < 20; i++) {
        snprintf(buf, sizeof(buf),
                 "lat=46.072700 lon=18.232300 step=%llu err=0.0023",
                 (unsigned long long)step_count);
    }
    (void)buf;
}

// Load 3: fake I2C-style delay loop (simulates sensor polling)
static void load_fake_i2c() {
    for (int i = 0; i < 200; i++) {
        delayMicroseconds(100);   // 50 × 50µs = 2.5ms total blocking delay
    }
}

// =============================================================================
//  setup()
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== Sidereal timer accuracy test ===");
    Serial.printf("Step period:  %llu µs\n",    STEP_PERIOD_US);
    Serial.printf("Step freq:    %.6f Hz\n",    1000000.0 / (double)STEP_PERIOD_US);
    Serial.printf("Total steps/rev: %llu\n",    TOTAL_STEPS);
    Serial.printf("Sidereal day: %.4f seconds\n\n", (double)SIDEREAL_US / 1e6);

    pinMode(STEP_PIN, OUTPUT);
    digitalWrite(STEP_PIN, LOW);

    // Create timer at 1 MHz (1 tick = 1 µs)
    step_timer = timerBegin(1000000);
    timerAttachInterrupt(step_timer, &step_isr);
    timerAlarm(step_timer, STEP_PERIOD_US, true, 0);

    Serial.println("Timer started. Loads running...");
    Serial.println("ELAPSED(s) | STEPS ACTUAL | STEPS EXPECTED | ERROR");
    Serial.println("-----------|--------------|----------------|------");
}

// =============================================================================
//  loop()
// =============================================================================
void loop() {
    static uint32_t last_report_ms = 0;
    static uint32_t start_ms       = millis();
    static bool     led_state       = false;

    // --- Run the simulated loads ---
    // These are the kinds of things the real firmware does while tracking.
    // If any of them were blocking the ISR, step_count would fall behind.
    load_math();
    load_string();
    load_fake_i2c();

    // --- Heartbeat LED: toggle every 500ms ---
    uint32_t now = millis();
    // --- Report every 5 seconds ---
    if (now - last_report_ms >= 5000) {
        last_report_ms = now;

        uint64_t actual   = step_count;
        double   elapsed_s = (now - start_ms) / 1000.0;

        // How many steps should have fired by now?
        double   expected = elapsed_s * (1000000.0 / (double)STEP_PERIOD_US);
        double   error    = (double)actual - expected;

        Serial.printf("%10.1f | %12llu | %14.2f | %+.2f\n",
                      elapsed_s,
                      (unsigned long long)actual,
                      expected,
                      error);

        // Warn if we're more than 1 step behind
        if (error < -1.0) {
            Serial.println("  *** WARNING: steps are late — ISR may be getting blocked ***");
        }
    }
}