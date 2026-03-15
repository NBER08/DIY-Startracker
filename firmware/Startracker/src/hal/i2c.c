#include "i2c.h"
#include "../config.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "hal_i2c";

// Map our bus enum to ESP-IDF i2c_port_t
static i2c_port_t bus_to_port(i2c_bus_t bus) {
    return (bus == I2C_BUS_SENSORS) ? I2C_NUM_0 : I2C_NUM_1;
}

esp_err_t i2c_hal_init(i2c_bus_t bus) {
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
    };

    if (bus == I2C_BUS_SENSORS) {
        cfg.sda_io_num  = PIN_I2C0_SDA;
        cfg.scl_io_num  = PIN_I2C0_SCL;
        cfg.master.clk_speed = I2C0_FREQ_HZ;
    } else {
        cfg.sda_io_num  = PIN_I2C1_SDA;
        cfg.scl_io_num  = PIN_I2C1_SCL;
        cfg.master.clk_speed = I2C1_FREQ_HZ;
    }

    i2c_port_t port = bus_to_port(bus);
    esp_err_t ret = i2c_param_config(port, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d param_config failed: %s", bus, esp_err_to_name(ret));
        return ret;
    }

    // 0 = no dedicated TX/RX buffer in master mode
    ret = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d driver_install failed: %s", bus, esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Bus %d initialised (SDA=%d SCL=%d)",
                 bus,
                 (bus == I2C_BUS_SENSORS) ? PIN_I2C0_SDA : PIN_I2C1_SDA,
                 (bus == I2C_BUS_SENSORS) ? PIN_I2C0_SCL : PIN_I2C1_SCL);
    }
    return ret;
}

esp_err_t i2c_hal_write(i2c_bus_t bus, uint8_t addr,
                          uint8_t reg, const uint8_t *buf, size_t len) {
    i2c_port_t port = bus_to_port(bus);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    if (len > 0) {
        i2c_master_write(cmd, buf, len, true);
    }
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_hal_read(i2c_bus_t bus, uint8_t addr,
                        uint8_t reg, uint8_t *buf, size_t len) {
    i2c_port_t port = bus_to_port(bus);

    // Write register address, then repeated start and read
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);  // repeated start
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_hal_write_read(i2c_bus_t bus, uint8_t addr,
                               const uint8_t *wbuf, size_t wlen,
                               uint8_t *rbuf,       size_t rlen) {
    i2c_port_t port = bus_to_port(bus);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, wbuf, wlen, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (rlen > 1) {
        i2c_master_read(cmd, rbuf, rlen - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, rbuf + rlen - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void i2c_hal_scan(i2c_bus_t bus) {
    ESP_LOGI(TAG, "Scanning I2C bus %d...", bus);
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        // A zero-byte write probes the address
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(bus_to_port(bus), cmd, pdMS_TO_TICKS(5));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            found++;
        }
    }
    ESP_LOGI(TAG, "Scan complete — %d device(s) found", found);
}
