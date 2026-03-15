#include "spi.h"
#include "../config.h"
#include "esp_log.h"
#include "driver/spi_master.h"

static const char *TAG = "hal_spi";

esp_err_t spi_hal_init(void) {
    spi_bus_config_t cfg = {
        .mosi_io_num     = PIN_SPI_MOSI,
        .miso_io_num     = PIN_SPI_MISO,
        .sclk_io_num     = PIN_SPI_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 256,
    };

    // SPI2_HOST is the general-purpose SPI bus on ESP32-S3
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPI2 init OK (MOSI=%d MISO=%d SCK=%d)",
                 PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK);
    }
    return ret;
}
