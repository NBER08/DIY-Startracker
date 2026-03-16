#pragma once
#include <stdbool.h>

// Two INA219 chips monitor power on two rails:
//   MOTOR rail (6V) — powers the TMC2209 and stepper motor
//   LOGIC rail (3.3V) — powers the ESP32 and sensors
//
// Why monitor power?
//   - Battery voltage tells you when to stop before a mid-exposure cutoff
//   - Motor current spikes tell you if something is mechanically jammed
//   - Logging consumption helps you plan session length

typedef struct {
    float voltage_V;    // rail voltage in volts
    float current_mA;   // current draw in milliamps
    float power_mW;     // voltage × current in milliwatts
} PowerReading;

// Call once at startup for each channel
void power_begin();

// Get readings from each rail
PowerReading power_read_motor();   // 6V motor rail
PowerReading power_read_logic();   // 3.3V logic rail

// Fault checks — call these in your main loop
bool power_battery_low();    // motor rail below 5.5V (safe shutdown threshold)
bool power_motor_jammed();   // motor current above 3.5A (mechanical blockage)
