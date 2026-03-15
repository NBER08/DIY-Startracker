#pragma once

// =============================================================================
//  hal/uart.h  —  thin UART wrappers
//
//  Used by:
//    UART1  — GPS (M8N), UBX binary protocol
//    UART2  — TMC2209 single-wire UART config
// =============================================================================

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/uart.h"

typedef struct {
    uart_port_t port;
    int         tx_pin;
    int         rx_pin;
    uint32_t    baud;
    size_t      rx_buf_size;   // ring buffer size in bytes — use 512+ for GPS
} uart_cfg_t;

// Install driver and configure pins
esp_err_t uart_hal_init(const uart_cfg_t *cfg);

// Blocking read with timeout_ms. Returns number of bytes actually read.
int uart_hal_read(uart_port_t port, uint8_t *buf, size_t len, uint32_t timeout_ms);

// Non-blocking write. Returns ESP_OK if all bytes queued.
esp_err_t uart_hal_write(uart_port_t port, const uint8_t *buf, size_t len);

// Flush receive buffer — call before starting a fresh UBX transaction
void uart_hal_flush_rx(uart_port_t port);
