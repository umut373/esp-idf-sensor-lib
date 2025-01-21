#ifndef I2C_DEV_H_
#define I2C_DEV_H_

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

#define delay(x) vTaskDelay(pdMS_TO_TICKS(x))

static const i2c_master_bus_config_t default_bus_config = {
    .i2c_port = -1,
    .sda_io_num = GPIO_NUM_21,
    .scl_io_num = GPIO_NUM_22,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7
};

class I2Cdev {
    uint8_t address;
    i2c_device_config_t dev_config; 
    i2c_master_dev_handle_t dev_handle;

    inline static i2c_master_bus_handle_t* bus_handle = nullptr;

public:
    I2Cdev(uint8_t address);
    ~I2Cdev();

    static void initialize(i2c_master_bus_config_t bus_config = default_bus_config);

    virtual bool init() = 0;

protected:
    void write8(const uint8_t reg, const uint8_t buffer);
    void write16(const uint8_t reg, const uint16_t buffer);

    uint8_t read8(const uint8_t reg);
    uint16_t read16(const uint8_t reg);
    void read_data(const uint8_t reg, uint8_t* buffer, size_t length);
};

#endif // I2C_DEV_H_