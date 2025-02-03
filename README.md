# ESP32 I2C and UART Sensor Libraries

This repository contains minimal libraries I wrote for the I2C and UART sensors used in my own projects. These libraries are developed for the ESP32 platform using the ESP-IDF framework (v5.4).

## I2C Sensors

For I2C sensors, the following pins are used by default:

- **SDA Pin**: GPIO21
- **SCL Pin**: GPIO22

The SDA pin of the sensor should be connected to GPIO21, and the SCL pin should be connected to GPIO22 on the ESP32.

### ⚠️ Important Note

Before performing sensor calibration or any operations that rely on NVS (Non-Volatile Storage), make sure to initialize NVS using the `nvs_flash_init()` function. Failing to do so may result in calibration data not being saved or read correctly.

Example usage:  
```c
#include "nvs_flash.h"

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Perform sensor calibration or other operations here
}
```

## UART Sensors

For UART sensors, the following pins are used by default:

- **TX Pin**: GPIO4
- **RX Pin**: GPIO5

The TX pin of the sensor should be connected to GPIO5, and the RX pin should be connected to GPIO4 on the ESP32.

## ESP-IDF Documentation

For more details on the ESP-IDF framework, refer to the official documentation:  
[ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)