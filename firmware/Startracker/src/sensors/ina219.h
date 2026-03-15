#pragma once

// =============================================================================
//  sensors/ina219.h  —  INA219 current / voltage monitor  (×2)
//
//  Two monitors:
//    INA219_MOTOR  (0x40) — 12 V motor supply rail
//    INA219_LOGIC  (0x41) — 3.3 V logic rail
//
//  Used for:
//    - Low battery detection (motor rail < BATTERY_UNDERVOLT_MV)
//    - Motor jam detection (motor current >> expected holding current)
//    - Field session power logging via LoRa telemetry
// =============================================================================

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    INA219_MOTOR = 0,
    INA219_LOGIC = 1,
} ina219_channel_t;

typedef struct {
    float   bus_voltage_mv;     // bus voltage in millivolts
    float   current_ma;         // current in milliamps (positive = load)
    float   power_mw;           // bus_voltage × current
    bool    valid;
} ina219_data_t;

// Initialise both INA219 sensors.
// Must be called after i2c_hal_init(I2C_BUS_SENSORS).
// shunt_mohm: shunt resistor value in milliohms — check your PCB BOM.
void ina219_init(ina219_channel_t ch, float shunt_mohm);

// Get latest reading for one channel.
ina219_data_t ina219_get(ina219_channel_t ch);

// Fault checks — these are polled by the env+power task and fed to
// the state machine as fault flags.
bool ina219_battery_low(void);      // motor rail below BATTERY_UNDERVOLT_MV
bool ina219_motor_jammed(void);     // motor current above MOTOR_OVERCURRENT_MA
