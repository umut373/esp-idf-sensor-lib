#ifndef I2C_DEV_H_
#define I2C_DEV_H_

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

#define delay(x) vTaskDelay(pdMS_TO_TICKS(x))


class I2Cdev {
    uint8_t address;
    i2c_master_dev_handle_t dev_handle;

    inline static i2c_master_bus_handle_t bus_handle = NULL;
    inline static bool bus_initialized = false;
    inline static int sensor_count = 0;

public:
    I2Cdev(uint8_t address);
    virtual ~I2Cdev();

    void initialize(gpio_num_t sda, gpio_num_t scl);
    virtual bool init() = 0;

protected:
    bool check_device();

    void i2c_write8(const uint8_t reg, const uint8_t buffer);
    void i2c_write16(const uint8_t reg, const uint16_t buffer);

    uint8_t i2c_read8(const uint8_t reg);
    uint16_t i2c_read16(const uint8_t reg);
    void i2c_read_data(const uint8_t reg, uint8_t* buffer, size_t length);
};

#endif // I2C_DEV_H_