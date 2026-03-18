#pragma once

// =============================================================================
//  config.h — all pin numbers and constants in one place
//  Change values here. Do not hardcode pins in any other file.
// =============================================================================

// ---- GPS (UART1) ----
#define GPS_RX_PIN   44
#define GPS_TX_PIN   43
#define GPS_BAUD     9600

// ---- I2C bus (sensors) ----
#define I2C_SDA      8
#define I2C_SCL      9

// ---- BNO085 IMU addresses ----
#define IMU_ADDR_MOUNT   0x4A   // PS1 = LOW
#define IMU_ADDR_CAMERA  0x4B   // PS1 = HIGH

// ---- Magnetometer (I2C) ----
#define MAG_ADDR      0x1E    // SA0 pin tied LOW on your board
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

// ---- power monitors (I2C) ----
// I2C addresses — set by A0/A1 pins on the INA219
#define ADDR_MOTOR  0x40    // A1=0, A0=0
#define ADDR_LOGIC  0x41    // A1=0, A0=1

// Fault thresholds
#define BATTERY_LOW_V    5.5f    // 6V battery is "low" below this
#define MOTOR_JAM_MA     3500.0f  // normal hold current ~1.2A; jam = much higher

// ---- Motor (TMC2209) ----
#define STEP_PIN     6
#define DIR_PIN      7

#define MOTOR_STEPS  200    // steps per motor revolution
#define MICROSTEP    16     // TMC2209 microstepping setting
#define GEAR_RATIO   144    // worm gear output ratio

// ---- Astro ----
#define POLARIS_RA_HRS   2.5303    // Right Ascension: 2 hours 31 minutes
#define POLARIS_DEC_DEG  89.2641   // Declination: 89° 15'

// ---- LoRa (SX1261, SPI) ----
#define LORA_MOSI    21
#define LORA_MISO    20
#define LORA_SCK     19
#define LORA_CS      18
#define LORA_DIO1    17
#define LORA_RST     15
#define LORA_BUSY    16
#define LORA_FREQ    868.0  // MHz — use 915.0 for US/AU

// ---- Misc ----
#define LED_PIN      2      // onboard LED
