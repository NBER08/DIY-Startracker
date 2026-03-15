#pragma once

// =============================================================================
//  config.h  —  pin assignments and compile-time constants
//  Edit this file to match your PCB. Nothing else should contain raw GPIO numbers.
// =============================================================================

// -----------------------------------------------------------------------------
//  GPS  (M8N via UART1)
// -----------------------------------------------------------------------------
#define PIN_GPS_TX          43      // ESP TX → GPS RX  (TXD0 on schematic)
#define PIN_GPS_RX          44      // GPS TX → ESP RX  (RXD0 on schematic)
#define PIN_GPS_1PPS        10      // 1-pulse-per-second input, rising edge
#define GPS_UART_PORT       UART_NUM_1
#define GPS_BAUD            9600

// -----------------------------------------------------------------------------
//  I2C bus 0  —  sensors (BNO085 ×2, IIS2MDCTR, BME280, INA219 ×2)
// -----------------------------------------------------------------------------
#define PIN_I2C0_SDA        8
#define PIN_I2C0_SCL        9
#define I2C0_FREQ_HZ        400000  // 400 kHz fast-mode

// I2C addresses
#define ADDR_BNO085_MOUNT   0x4A    // PS1 = 0
#define ADDR_BNO085_CAMERA  0x4B    // PS1 = 1
#define ADDR_IIS2MDCTR      0x1E    // SA0 tied low
#define ADDR_BME280         0x76    // SDO tied low
#define ADDR_INA219_MOTOR   0x40    // A1=0, A0=0  — 12 V motor rail
#define ADDR_INA219_LOGIC   0x41    // A1=0, A0=1  — 3.3 V logic rail

// -----------------------------------------------------------------------------
//  I2C bus 1  —  TB6612FNG expander (separate bus, can't block sensors)
// -----------------------------------------------------------------------------
#define PIN_I2C1_SDA        35      // TB_SDAD on schematic
#define PIN_I2C1_SCL        36      // TB_SCLD on schematic
#define I2C1_FREQ_HZ        100000
#define ADDR_IO_EXPANDER    0x20    // PCA9535 default address

// -----------------------------------------------------------------------------
//  SPI bus  —  SX1261 LoRa
// -----------------------------------------------------------------------------
#define PIN_SPI_MOSI        21      // SX_MOSI
#define PIN_SPI_MISO        20      // SX_MISO  (IO20 on schematic — verify not USB_D+)
#define PIN_SPI_SCK         19      // SX_SCK
#define PIN_SX1261_CS       18      // SX_NS5
#define PIN_SX1261_DIO1     17      // SX_DIO1
#define PIN_SX1261_RST      15      // SX_RST
#define PIN_SX1261_BUSY     16      // SX_BUSY
#define SX1261_FREQ_MHZ     868.0   // EU 868 MHz — change to 915.0 for US/AU

// -----------------------------------------------------------------------------
//  TMC2209  —  polar tracking axis
// -----------------------------------------------------------------------------
#define PIN_TMC_STEP        6       // STEP1D
#define PIN_TMC_DIR         7       // DIR1D
#define PIN_TMC_TX          4       // TMC_TXD  (UART2)
#define PIN_TMC_RX          5       // TMC_RXD
#define TMC_UART_PORT       UART_NUM_2
#define TMC_UART_ADDR       0       // MS1/MS2 both low → address 0
#define TMC_RMS_CURRENT_MA  1200    // Motor RMS current — tune to your motor

// Motor geometry — used in step period calculation
#define MOTOR_STEPS_PER_REV 200
#define MICROSTEP_DIVISOR   16      // set via TMC2209 UART on startup
#define GEAR_RATIO          144     // worm gear output ratio — VERIFY before use

// -----------------------------------------------------------------------------
//  Camera control jack
// -----------------------------------------------------------------------------
#define PIN_CAMERA_FOCUS    38      // FocusD  — drives NPN transistor
#define PIN_CAMERA_SHUTTER  39      // ShutterD

// -----------------------------------------------------------------------------
//  Misc
// -----------------------------------------------------------------------------
#define PIN_DEBUG_LED       46      // DEBUG_LED1 on schematic — active high

// -----------------------------------------------------------------------------
//  Tracking rates  (seconds per sidereal-equivalent output revolution)
// -----------------------------------------------------------------------------
#define SIDEREAL_DAY_S      86164.0905
#define LUNAR_DAY_S         89309.4
#define SOLAR_DAY_S         86400.0

// -----------------------------------------------------------------------------
//  FreeRTOS task stack sizes (words, not bytes — 1 word = 4 bytes on Xtensa)
// -----------------------------------------------------------------------------
#define STACK_GPS           4096
#define STACK_IMU           4096
#define STACK_MAG           2048
#define STACK_ENV           2048
#define STACK_LORA          4096
#define STACK_STATE_MACHINE 6144

// -----------------------------------------------------------------------------
//  Fault thresholds
// -----------------------------------------------------------------------------
#define BATTERY_UNDERVOLT_MV    10500   // stop tracking below this on 12 V rail
#define MOTOR_OVERCURRENT_MA    3500    // jam detection on motor INA219
