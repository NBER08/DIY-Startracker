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

// ---- BNO085 — camera only (one sensor now) ----
#define IMU_ADDR_CAMERA  0x4A

// ---- VL53L4CD ToF distance sensor ----
// Mounted on the base platform, beam pointing toward the tilting platform.
// Used to calculate platform altitude angle.
#define TOF_ADDR         0x29   // default I2C address, not changeable

// Calibration — measure these physically on your build:
//   TOF_FLAT_MM   : reading (mm) when the platform is perfectly horizontal (0°)
//   TOF_ARM_MM    : horizontal distance (mm) from the sensor beam
//                   to the platform hinge/pivot point
//
// Geometry:  altitude_deg = atan( (reading - TOF_FLAT_MM) / TOF_ARM_MM )
// Set both to 1.0 as placeholders until you can measure them.
#define TOF_FLAT_MM      120.0f   // replace with your measured value
#define TOF_ARM_MM       80.0f    // replace with your measured value

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

// ---- Azimuth + camera tilt motors via PCA9535 I2C expander ----
// A0=A1=A2=GND → address 0x20
// All motor pins on Port 0 (single register write moves both motors)
//
//  Port 0 bit → expander pin → TB6612FNG signal
//  bit 0 → P0 → D2_A2   (azimuth motor)
//  bit 1 → P1 → D2_A1
//  bit 2 → P2 → D2_B1
//  bit 3 → P3 → D2_B2
//  bit 4 → P4 → D3_B2   (camera tilt motor)
//  bit 5 → P5 → D3_B1
//  bit 6 → P6 → D3_A1
//  bit 7 → P7 → D3_A2
#define EXPANDER_ADDR      0x20
#define CAM_STEPS_PER_DEG  20.0f

// ---- Astro ----
#define POLARIS_RA_HRS   2.5303    // Right Ascension: 2 hours 31 minutes
#define POLARIS_DEC_DEG  89.2641   // Declination: 89° 15'

// ---- Camera ----
#define SHUTTER_PIN  38     // drives NPN transistor to camera shutter jack
#define FOCUS_PIN    39     // drives NPN transistor to camera focus jack

// ---- LoRa (SX1261, SPI) ----
#define LORA_MOSI    21
#define LORA_MISO    20
#define LORA_SCK     19
#define LORA_CS      18
#define LORA_DIO1    17
#define LORA_RST     15
#define LORA_BUSY    16
#define LORA_FREQ    868.0  // MHz — use 915.0 for US/AU
#define LORA_TX_DBM  10     // LoRa transmit power in dBm (-17– +15)

// ---- Misc ----
#define LED_PIN      2      // onboard LED
