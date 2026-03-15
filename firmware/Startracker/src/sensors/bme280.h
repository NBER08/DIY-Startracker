#pragma once

// =============================================================================
//  sensors/bme280.h  —  BME280 temperature / pressure / humidity
//
//  Data is used for:
//    - Atmospheric refraction correction in astro_math
//    - Temperature reporting in LoRa telemetry
//    - Optional dew point calculation (camera lens fogging warning)
// =============================================================================

#include <stdbool.h>

typedef struct {
    float temperature_c;    // ambient temperature, degrees Celsius
    float pressure_hpa;     // atmospheric pressure, hPa (used for refraction)
    float humidity_rh;      // relative humidity, %
    float dewpoint_c;       // calculated dew point — warn if temp < dewpoint+2
    bool  valid;
} bme280_data_t;

// Initialise the BME280 in normal mode (1 Hz updates).
// Must be called after i2c_hal_init(I2C_BUS_SENSORS).
void bme280_init(void);

// Get latest reading. Valid immediately after init succeeds.
bme280_data_t bme280_get(void);

// True if ambient temperature is within 2°C of dew point.
// The state machine can send a LoRa warning when this is true.
bool bme280_dew_warning(void);
