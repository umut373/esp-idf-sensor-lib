#include "i2c_dev.h"

I2Cdev::I2Cdev(uint8_t address) {
    if (bus_handle == nullptr) {
        ESP_ERROR_CHECK(ESP_FAIL);
    } else {
        this->address = address;

        dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = address,
            .scl_speed_hz = 100000,
        };

        ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, &dev_handle));
    }
}

I2Cdev::~I2Cdev() {
    if (bus_handle != nullptr) {
        delete bus_handle;
    }
}

void I2Cdev::initialize(i2c_master_bus_config_t bus_config) {
    if (bus_handle == nullptr) {
        bus_handle = new i2c_master_bus_handle_t;
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));
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