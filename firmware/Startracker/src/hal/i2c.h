#pragma once

// =============================================================================
//  hal/i2c.h  —  thin I2C bus wrappers
//
//  Two buses:
//    I2C_BUS_SENSORS  (bus 0) — BNO085 ×2, IIS2MDCTR, BME280, INA219 ×2
//    I2C_BUS_EXPANDER (bus 1) — PCA9535 I/O expander driving TB6612FNG
//
//  All functions are safe to call from FreeRTOS tasks.
//  None are safe to call from an ISR.
// =============================================================================

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

typedef enum {
    I2C_BUS_SENSORS  = 0,
    I2C_BUS_EXPANDER = 1,
} i2c_bus_t;

// Call once at startup for each bus before any sensor init
esp_err_t i2c_hal_init(i2c_bus_t bus);

// Write len bytes from buf to device at addr, starting at reg
esp_err_t i2c_hal_write(i2c_bus_t bus, uint8_t addr,
                         uint8_t reg, const uint8_t *buf, size_t len);

// Read len bytes into buf from device at addr, starting at reg
esp_err_t i2c_hal_read(i2c_bus_t bus, uint8_t addr,
                        uint8_t reg, uint8_t *buf, size_t len);

// Convenience: write then read in one transaction (for sensors that require it)
esp_err_t i2c_hal_write_read(i2c_bus_t bus, uint8_t addr,
                              const uint8_t *wbuf, size_t wlen,
                              uint8_t *rbuf,       size_t rlen);

// Scan bus and print found addresses to ESP log — useful during bring-up
void i2c_hal_scan(i2c_bus_t bus);
