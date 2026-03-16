#include "motor/slew.h"
#include "sensors/mag.h"
#include "sensors/imu.h"
#include <Arduino.h>
#include <Wire.h>

// =============================================================================
//  Hardware layout
//
//  PCA9535 I2C I/O expander sits between the ESP32 and the TB6612FNG chips.
//  It gives us 16 extra GPIO pins over just 2 wires (SDA/SCL).
//  We write a byte to it and it sets 8 physical output pins.
//
//  PCA9535 has two 8-bit output ports:
//    Port 0 (register 0x02) → controls azimuth  motor (TB6612FNG #1)
//    Port 1 (register 0x03) → controls altitude motor (TB6612FNG #2)
//
//  Each TB6612FNG channel needs 4 control pins:
//    AIN1, AIN2 → direction of coil A
//    BIN1, BIN2 → direction of coil B
//    (STBY pin is tied high on both — always enabled)
//
//  Bit layout of each port byte:
//    bit 0 = AIN1
//    bit 1 = AIN2
//    bit 2 = BIN1
//    bit 3 = BIN2
//    bits 4-7 unused
// =============================================================================

#define PCA9535_ADDR     0x20   // I2C address (all address pins low)
#define PCA_OUT_PORT0    0x02   // output register for port 0 (azimuth)
#define PCA_OUT_PORT1    0x03   // output register for port 1 (altitude)
#define PCA_CFG_PORT0    0x06   // direction register port 0 (0=output, 1=input)
#define PCA_CFG_PORT1    0x07   // direction register port 1

// =============================================================================
//  Stepper motor half-step sequence
//
//  A stepper motor has two coils (A and B). We energise them in a rotating
//  pattern to make the shaft turn. "Half-step" uses 8 states instead of 4,
//  giving smoother motion and twice the angular resolution.
//
//  Each row is one step: {AIN1, AIN2, BIN1, BIN2}
//  Reading down the table = one full revolution of the motor.
//
//  To reverse direction: read the table upward instead.
// =============================================================================
static const uint8_t HALF_STEP[8][4] = {
    {1, 0, 0, 0},   // coil A only, forward
    {1, 0, 1, 0},   // both coils
    {0, 0, 1, 0},   // coil B only, forward
    {0, 1, 1, 0},   // both coils
    {0, 1, 0, 0},   // coil A only, reverse
    {0, 1, 0, 1},   // both coils
    {0, 0, 0, 1},   // coil B only, reverse
    {1, 0, 0, 1},   // both coils
};

// Current step index for each axis — tracks where in the sequence we are
static int az_step_idx  = 0;
static int alt_step_idx = 0;

// How long to wait between steps (milliseconds).
// Shorter = faster but motor may skip steps.
// 8ms = safe speed for most small stepper motors.
#define STEP_DELAY_MS   8

// =============================================================================
//  PCA9535 write helper
//
//  Writing to the PCA9535 is just a standard I2C write:
//    start → device address → register address → data byte → stop
// =============================================================================
static void pca_write(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(PCA9535_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// =============================================================================
//  Convert a step's {AIN1, AIN2, BIN1, BIN2} into one byte for the PCA9535.
//
//  The four pin values become the four lowest bits of the byte:
//    byte = AIN1<<0 | AIN2<<1 | BIN1<<2 | BIN2<<3
// =============================================================================
static uint8_t step_to_byte(const uint8_t pins[4]) {
    return (pins[0]     )
         | (pins[1] << 1)
         | (pins[2] << 2)
         | (pins[3] << 3);
}

// =============================================================================
//  Execute one step on one axis
//
//  "direction" is +1 (forward) or -1 (reverse).
//  We advance the step index in that direction and write the new
//  pin pattern to the PCA9535 output port for that axis.
// =============================================================================
static void step_az(int direction) {
    az_step_idx = (az_step_idx + direction + 8) % 8;  // wrap 0-7
    pca_write(PCA_OUT_PORT0, step_to_byte(HALF_STEP[az_step_idx]));
}

static void step_alt(int direction) {
    alt_step_idx = (alt_step_idx + direction + 8) % 8;
    pca_write(PCA_OUT_PORT1, step_to_byte(HALF_STEP[alt_step_idx]));
}

// De-energise a motor to save power after a move.
// The mechanical friction of the worm gear holds position without power.
static void release_az()  { pca_write(PCA_OUT_PORT0, 0x00); }
static void release_alt() { pca_write(PCA_OUT_PORT1, 0x00); }

// =============================================================================
//  Public API
// =============================================================================

void slew_begin() {
    // Set all 16 PCA9535 pins as outputs (0 = output in the direction register)
    pca_write(PCA_CFG_PORT0, 0x00);
    pca_write(PCA_CFG_PORT1, 0x00);

    // All pins low — motors de-energised at start
    pca_write(PCA_OUT_PORT0, 0x00);
    pca_write(PCA_OUT_PORT1, 0x00);

    Serial.println("Slew: PCA9535 configured, motors ready");
}

void slew_stop() {
    release_az();
    release_alt();
}

void slew_az_nudge(bool clockwise) {
    step_az(clockwise ? +1 : -1);
    delay(STEP_DELAY_MS);
    // Don't release — hold position for nudging
}

void slew_alt_nudge(bool upward) {
    step_alt(upward ? +1 : -1);
    delay(STEP_DELAY_MS);
}

// =============================================================================
//  slew_to_pole — the main alignment routine
//
//  This function runs a feedback loop:
//    1. Read where we're currently pointing (mag → azimuth, IMU → altitude)
//    2. Compare to where we need to point (from astro_get_pole)
//    3. Step whichever motor reduces the error
//    4. Repeat until we're within SLEW_TOLERANCE_DEG on both axes
//
//  It's like a thermostat — continuously comparing actual vs target
//  and correcting in the right direction.
// =============================================================================
bool slew_to_pole(PoleDirection target, int timeout_sec) {
    Serial.printf("Slew: target alt=%.2f°  az=%.2f°\n",
                  target.altitude_deg, target.azimuth_deg);

    unsigned long deadline = millis() + (unsigned long)timeout_sec * 1000;
    bool az_done  = false;
    bool alt_done = false;

    while (millis() < deadline) {

        // --- Read current pointing ---
        MagData mag = mag_read();
        ImuData imu = imu_get_mount();

        float current_az  = mag.heading_true;
        float current_alt = 90.0f - imu.pitch;  // IMU pitch 0° = vertical

        // --- Calculate errors ---
        float az_err  = (float)(target.azimuth_deg  - current_az);
        float alt_err = (float)(target.altitude_deg - current_alt);

        // Normalise azimuth error to [-180, 180]
        // (so we always take the short way around)
        if (az_err >  180.0f) az_err -= 360.0f;
        if (az_err < -180.0f) az_err += 360.0f;

        // --- Check if each axis is done ---
        az_done  = (fabs(az_err)  < SLEW_TOLERANCE_DEG);
        alt_done = (fabs(alt_err) < SLEW_TOLERANCE_DEG);

        if (az_done && alt_done) {
            // We've arrived!
            release_az();
            release_alt();
            float total = astro_total_error_deg(target, current_az, current_alt);
            Serial.printf("Slew: aligned! total error=%.3f°\n", total);
            return true;
        }

        // --- Step toward target ---
        // Only move one axis at a time to avoid I2C timing issues.
        // Azimuth first — it has larger error usually.
        if (!az_done) {
            step_az(az_err > 0 ? +1 : -1);
        } else if (!alt_done) {
            step_alt(alt_err > 0 ? +1 : -1);
        }

        delay(STEP_DELAY_MS);

        // Print progress every 2 seconds
        static unsigned long last_print = 0;
        if (millis() - last_print > 2000) {
            last_print = millis();
            Serial.printf("Slew: az_err=%.2f°  alt_err=%.2f°\n",
                          az_err, alt_err);
        }
    }

    // Timed out
    slew_stop();
    Serial.println("Slew: TIMEOUT — check motor wiring and sensor readings");
    return false;
}
