#include "uart.h"
#include "esp_log.h"

static const char *TAG = "hal_uart";

esp_err_t uart_hal_init(const uart_cfg_t *cfg) {
    uart_config_t ucfg = {
        .baud_rate  = (int)cfg->baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret;
    ret = uart_param_config(cfg->port, &ucfg);
    if (ret != ESP_OK) return ret;

    ret = uart_set_pin(cfg->port, cfg->tx_pin, cfg->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) return ret;

    ret = uart_driver_install(cfg->port, cfg->rx_buf_size, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART%d install failed: %s", cfg->port, esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "UART%d init OK (TX=%d RX=%d baud=%lu)",
                 cfg->port, cfg->tx_pin, cfg->rx_pin, cfg->baud);
    }
    return ret;
}

int uart_hal_read(uart_port_t port, uint8_t *buf, size_t len, uint32_t timeout_ms) {
    return uart_read_bytes(port, buf, len, pdMS_TO_TICKS(timeout_ms));
}

esp_err_t uart_hal_write(uart_port_t port, const uint8_t *buf, size_t len) {
    int written = uart_write_bytes(port, buf, len);
    return (written == (int)len) ? ESP_OK : ESP_FAIL;
}

void uart_hal_flush_rx(uart_port_t port) {
    uart_flush_input(port);
}
