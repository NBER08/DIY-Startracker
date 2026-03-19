#include "slew.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// =============================================================================
//  PCA9535 I2C expander — controls both motors with one byte write
//
//  Register map:
//    0x06 = Configuration port 0 (0 = output, 1 = input — set all to 0)
//    0x02 = Output port 0
//
//  Bit layout of the output byte:
//    bits 0-3 = azimuth motor  (D2): A2, A1, B1, B2
//    bits 4-7 = camera motor   (D3): B2, B1, A1, A2
// =============================================================================

#define PCA_REG_CFG_0   0x06
#define PCA_REG_OUT_0   0x02

// Half-step sequence — {AIN1, AIN2, BIN1, BIN2} for each of 8 states
static const uint8_t HALF_STEP[8][4] = {
    {1,0,0,0}, {1,0,1,0}, {0,0,1,0}, {0,1,1,0},
    {0,1,0,0}, {0,1,0,1}, {0,0,0,1}, {1,0,0,1},
};

// Pre-computed output bytes for each half-step state.
// az_lut  : bits 0-3 only (azimuth motor, lower nibble)
// cam_lut : bits 4-7 only (camera motor, upper nibble)
//
// Azimuth motor pin mapping (from schematic):
//   AIN1=P1(bit1)  AIN2=P0(bit0)  BIN1=P2(bit2)  BIN2=P3(bit3)
//
// Camera motor pin mapping:
//   AIN1=P6(bit6)  AIN2=P7(bit7)  BIN1=P5(bit5)  BIN2=P4(bit4)

static uint8_t az_lut[8];
static uint8_t cam_lut[8];

static void build_lut() {
    for (int i = 0; i < 8; i++) {
        uint8_t ain1 = HALF_STEP[i][0];
        uint8_t ain2 = HALF_STEP[i][1];
        uint8_t bin1 = HALF_STEP[i][2];
        uint8_t bin2 = HALF_STEP[i][3];

        // Azimuth: AIN1→bit1, AIN2→bit0, BIN1→bit2, BIN2→bit3
        az_lut[i] = (ain1 << 1) | (ain2 << 0) | (bin1 << 2) | (bin2 << 3);

        // Camera: AIN1→bit6, AIN2→bit7, BIN1→bit5, BIN2→bit4
        cam_lut[i] = (ain1 << 6) | (ain2 << 7) | (bin1 << 5) | (bin2 << 4);
    }
}

// Write a byte to the expander output port
static void expander_write(uint8_t value) {
    Wire.beginTransmission(EXPANDER_ADDR);
    Wire.write(PCA_REG_OUT_0);
    Wire.write(value);
    Wire.endTransmission();
}

// Current step indices and camera position
static int   az_idx  = 0;
static int   cam_idx = 0;
static float cam_deg = 0.0f;

// Combined output — upper nibble = camera, lower nibble = azimuth
static uint8_t current_out = 0x00;

static void apply_motors() {
    int ai = ((az_idx  % 8) + 8) % 8;
    int ci = ((cam_idx % 8) + 8) % 8;
    current_out = az_lut[ai] | cam_lut[ci];
    expander_write(current_out);
}

static void motors_off() {
    expander_write(0x00);
}

// =============================================================================
//  Public API
// =============================================================================

void slew_begin() {
    build_lut();

    // Set all Port 0 pins as outputs
    Wire.beginTransmission(EXPANDER_ADDR);
    Wire.write(PCA_REG_CFG_0);
    Wire.write(0x00);   // 0 = output
    Wire.endTransmission();

    motors_off();
    Serial.println("Slew: expander ready (0x20)");
}

// Azimuth — call from loop(), returns true when within tolerance
bool slew_az_toward(float target_az_deg, float current_az_deg,
                     float tolerance_deg) {
    float diff = target_az_deg - current_az_deg;
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    if (fabsf(diff) <= tolerance_deg) {
        // De-energise azimuth coils — keep camera nibble as-is
        current_out &= 0xF0;   // clear lower nibble (azimuth)
        expander_write(current_out);
        return true;
    }

    az_idx += (diff > 0) ? 1 : -1;
    apply_motors();
    delay(8);
    return false;
}

void slew_az_stop() {
    current_out &= 0xF0;
    expander_write(current_out);
}

// Camera tilt — blocking move to an absolute polar-distance angle
void camera_tilt_to(float target_deg) {
    float diff      = target_deg - cam_deg;
    int   steps     = (int)(fabsf(diff) * CAM_STEPS_PER_DEG);
    int   dir       = (diff > 0) ? 1 : -1;
    if (steps == 0) return;

    Serial.printf("Camera: %.1f° → %.1f° (%d steps)\n", cam_deg, target_deg, steps);

    for (int i = 0; i < steps; i++) {
        cam_idx += dir;
        apply_motors();
        delay(8);
    }

    // De-energise camera coils — keep azimuth nibble as-is
    cam_deg = target_deg;
    current_out &= 0x0F;   // clear upper nibble (camera)
    expander_write(current_out);
    Serial.printf("Camera: done (%.1f°)\n", cam_deg);
}

void camera_tilt_stop() {
    current_out &= 0x0F;
    expander_write(current_out);
}

float camera_get_tilt_deg() {
    return cam_deg;
}