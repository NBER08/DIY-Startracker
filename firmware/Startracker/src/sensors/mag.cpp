#include "mag.h"
#include <Arduino.h>
#include <Wire.h>

// =============================================================================
//  IIS2MDCTR Magnetometer
//
//  This chip talks over I2C. We don't need a library for it — the chip
//  is simple enough to drive with just Wire.h reads and writes directly
//  to its registers.
//
//  A "register" on an I2C sensor is just a memory location inside the chip.
//  You write to a register to configure the chip.
//  You read from a register to get data from it.
//  Every register has an address (a single byte, like 0x60).
// =============================================================================

#define MAG_ADDR        0x1E    // I2C address (SA0 pin tied low on your board)

// Register addresses (from IIS2MDCTR datasheet, Table 15)
#define REG_CFG_A       0x60    // configuration: sets sample rate and mode
#define REG_CFG_C       0x62    // configuration: enables block data update
#define REG_STATUS      0x67    // tells us when new data is ready
#define REG_OUTX_L      0x68    // X axis output, low byte
                                // Y and Z follow at 0x69, 0x6A, 0x6B, 0x6C, 0x6D

// How sensitive is the chip? 1.5 milligauss per raw unit (from datasheet)
#define SENSITIVITY     1.5f    // mGauss per LSB

// Magnetic declination for Pécs, Hungary ≈ +4.5° East
// Positive = true north is EAST of magnetic north
// Look up your exact value at: https://www.magnetic-declination.com
#define DECLINATION_DEG 4.5f

// Hard-iron calibration offsets.
// These correct for the magnetic distortion caused by nearby metal
// (your motors, PCB copper pours, etc).
// After running mag_calibrate() once, copy the printed values here.
static float cal_offset_x = 0.0f;
static float cal_offset_y = 0.0f;

// =============================================================================
//  Low-level I2C helpers
//  These are tiny wrappers so the rest of the code is easier to read.
// =============================================================================

static void write_reg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

static uint8_t read_reg(uint8_t reg) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);   // false = don't release bus (repeated start)
    Wire.requestFrom(MAG_ADDR, 1);
    return Wire.read();
}

// Read multiple consecutive registers in one I2C transaction.
// The IIS2MDCTR auto-increments its register pointer, so we can
// grab X, Y, Z (6 bytes total) in a single read.
static void read_regs(uint8_t start_reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(start_reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MAG_ADDR, (int)len);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
}

// =============================================================================
//  Public functions
// =============================================================================

void mag_begin() {
    // CFG_REG_A = 0x0C:
    //   bits [3:2] = 11 → ODR = 50 Hz (output data rate)
    //   bits [1:0] = 00 → continuous measurement mode
    write_reg(REG_CFG_A, 0x0C);

    // CFG_REG_C = 0x10:
    //   bit 4 = 1 → Block Data Update enabled
    //   This prevents reading the high byte of X while the chip is
    //   writing the low byte of X — avoids corrupted readings.
    write_reg(REG_CFG_C, 0x10);

    // Verify the chip responded by checking WHO_AM_I (register 0x4F)
    // The IIS2MDCTR always returns 0x40 from this register.
    uint8_t who = read_reg(0x4F);
    if (who == 0x40) {
        Serial.println("Mag: IIS2MDCTR found OK");
    } else {
        Serial.printf("Mag: wrong WHO_AM_I (got 0x%02X, expected 0x40)\n", who);
        Serial.println("Mag: check wiring and I2C address");
    }
}

MagData mag_read() {
    MagData out = {};

    // Check the STATUS register — bit 3 (ZYXDA) goes high when
    // all three axes have fresh data ready to read
    uint8_t status = read_reg(REG_STATUS);
    if (!(status & 0x08)) {
        // No new data yet — return the previous reading unchanged
        return out;
    }

    // Read all 6 output bytes starting at OUTX_L (auto-increment reads X, Y, Z)
    uint8_t raw[6];
    read_regs(REG_OUTX_L, raw, 6);

    // Each axis is a signed 16-bit integer stored little-endian (low byte first)
    int16_t rx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t ry = (int16_t)((raw[3] << 8) | raw[2]);

    // Convert raw counts to microtesla (÷10 converts mGauss to µT)
    out.x_uT = rx * SENSITIVITY / 10.0f;
    out.y_uT = ry * SENSITIVITY / 10.0f;

    // Apply hard-iron calibration offsets
    float cx = (rx * SENSITIVITY) - cal_offset_x;
    float cy = (ry * SENSITIVITY) - cal_offset_y;

    // atan2 gives the angle of the magnetic field vector in the horizontal plane
    // Result is in radians → convert to degrees → shift to 0–360 range
    float heading = atan2(cy, cx) * 180.0f / M_PI;
    if (heading < 0) heading += 360.0f;

    out.heading_mag  = heading;
    out.heading_true = heading + DECLINATION_DEG;
    if (out.heading_true >= 360.0f) out.heading_true -= 360.0f;
    
    return out;
}

// =============================================================================
//  Calibration routine
//
//  The problem: the raw magnetometer output is distorted into an ellipse
//  by nearby ferrous metal (motors, screws, PCB copper). We need to find
//  the centre of that ellipse (the "hard iron offset") and subtract it.
//
//  The method: slowly rotate the tracker through all orientations for 15
//  seconds while we record the min and max on each axis. The centre of
//  the min-max range on each axis IS the offset we need to subtract.
//
//  After running this, update cal_offset_x and cal_offset_y above.
// =============================================================================
void mag_calibrate() {
    Serial.println("Mag calibration: slowly rotate the tracker in all directions");
    Serial.println("for 15 seconds. Keep it away from large metal objects.");

    float xmin =  99999, xmax = -99999;
    float ymin =  99999, ymax = -99999;
    int   samples = 0;

    unsigned long end_ms = millis() + 15000;
    while (millis() < end_ms) {
        uint8_t raw[6];
        read_regs(REG_OUTX_L, raw, 6);
        float rx = (int16_t)((raw[1] << 8) | raw[0]) * SENSITIVITY;
        float ry = (int16_t)((raw[3] << 8) | raw[2]) * SENSITIVITY;

        if (rx < xmin) xmin = rx;
        if (rx > xmax) xmax = rx;
        if (ry < ymin) ymin = ry;
        if (ry > ymax) ymax = ry;
        samples++;
        delay(50);
    }

    float offset_x = (xmax + xmin) / 2.0f;
    float offset_y = (ymax + ymin) / 2.0f;

    Serial.println("Calibration done! Copy these values into mag.cpp:");
    Serial.printf("  cal_offset_x = %.2f;\n", offset_x);
    Serial.printf("  cal_offset_y = %.2f;\n", offset_y);
    Serial.printf("  (%d samples collected)\n", samples);

    // Apply immediately without requiring a restart
    cal_offset_x = offset_x;
    cal_offset_y = offset_y;
}
