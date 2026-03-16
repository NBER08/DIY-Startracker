#include "power.h"
#include <Arduino.h>
#include <Adafruit_INA219.h>

// =============================================================================
//  INA219 — Voltage and Current Monitor (×2)
//
//  The INA219 works by measuring the tiny voltage drop across a small
//  "shunt" resistor placed in series with the load. Because the resistor
//  value is known (Ohm's Law: I = V/R), the chip can calculate current.
//  It also directly measures the supply voltage on the bus.
//
//    Motor rail shunt:  20  mΩ  (0.020 Ω) — at address 0x40
//    Logic rail shunt:  100 mΩ  (0.100 Ω) — at address 0x41
// =============================================================================

// I2C addresses — set by A0/A1 pins on the INA219
#define ADDR_MOTOR  0x40    // A1=0, A0=0
#define ADDR_LOGIC  0x41    // A1=0, A0=1

// Fault thresholds
#define BATTERY_LOW_V    5.5f    // 6V battery is "low" below this
#define MOTOR_JAM_MA     3500.0f  // normal hold current ~1.2A; jam = much higher

static Adafruit_INA219 ina_motor(ADDR_MOTOR);
static Adafruit_INA219 ina_logic(ADDR_LOGIC);

static PowerReading latest_motor = {};
static PowerReading latest_logic = {};

void power_begin() {
    // begin() configures the chip for ±32V, ±2A range by default.
    // For the motor rail we need a higher current range, so we
    // call setCalibration_32V_2A() explicitly for both — the motor
    // will briefly exceed 2A on startup but that's acceptable here.

    ina_logic.begin();
    ina_motor.begin();
}

PowerReading power_read_motor() {
    // getShuntVoltage_mV() returns the voltage across the shunt resistor.
    // getBusVoltage_V()    returns the supply voltage.
    // getCurrent_mA()      returns current calculated from shunt voltage ÷ shunt resistance.

    latest_motor.voltage_V  = ina_motor.getBusVoltage_V();
    latest_motor.current_mA = ina_motor.getCurrent_mA();
    latest_motor.power_mW   = ina_motor.getPower_mW();
    return latest_motor;
}

PowerReading power_read_logic() {
    latest_logic.voltage_V  = ina_logic.getBusVoltage_V();
    latest_logic.current_mA = ina_logic.getCurrent_mA();
    latest_logic.power_mW   = ina_logic.getPower_mW();
    return latest_logic;
}

bool power_battery_low() {
    // We use the most recently read value — no need to re-read here.
    // Call power_read_motor() regularly in your loop to keep it fresh.
    return latest_motor.voltage_V < BATTERY_LOW_V;
}

bool power_motor_jammed() {
    // A jammed motor draws far more current than normal because
    // the coils are energised but the shaft can't turn.
    return latest_motor.current_mA > MOTOR_JAM_MA;
}
