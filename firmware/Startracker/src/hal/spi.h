#pragma once

// =============================================================================
//  hal/spi.h  —  thin SPI bus wrapper
//
//  Used by:
//    SX1261 LoRa module
//
//  RadioLib handles the SX1261 SPI transactions directly using its own
//  ESP32 HAL. This wrapper exists for any future SPI devices and to
//  initialise the bus before RadioLib takes over.
// =============================================================================

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// Initialise the VSPI/SPI2 bus (MOSI/MISO/SCK).
// Call once before constructing any RadioLib objects.
// CS pins are managed per-device, not here.
esp_err_t spi_hal_init(void);
