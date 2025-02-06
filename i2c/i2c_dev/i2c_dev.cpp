#include "i2c_dev.h"

I2Cdev::I2Cdev(uint8_t address) {
    if (!bus_initialized) {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = -1,
            .sda_io_num = GPIO_NUM_21,
            .scl_io_num = GPIO_NUM_22,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = true,
                .allow_pd = false
            }
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
        bus_initialized = true;
    }

    this->address = address;

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false
        }
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
    sensor_count++;
}

I2Cdev::~I2Cdev() {
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    sensor_count--;

    if (!sensor_count) {
        ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
        bus_initialized = false;
    }
}

void I2Cdev::write8(const uint8_t reg, const uint8_t buffer) {
    uint8_t data[] = {reg, buffer};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 2, -1));
}

void I2Cdev::write16(const uint8_t reg, const uint16_t buffer) {
    const uint8_t* buffer_arr = reinterpret_cast<const uint8_t*>(&buffer);
    uint8_t data[] = {reg, buffer_arr[1], buffer_arr[0]};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 3, -1));
}

uint8_t I2Cdev::read8(const uint8_t reg) {
    uint8_t buffer;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, &buffer, 1, -1));

   return buffer;
}

uint16_t I2Cdev::read16(const uint8_t reg) {
    uint8_t buffer[2];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, 2, -1));

    return (buffer[0] << 8) + buffer[1];
}

void I2Cdev::read_data(const uint8_t reg, uint8_t* buffer, size_t length) {
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, length, -1));
}