# ESP32 I2C and UART Sensor Libraries

This repository contains minimal libraries I wrote for the I2C and UART sensors used in my own projects. These libraries are developed for the ESP32 platform using the ESP-IDF framework (v5.4).

## I2C Sensors

For I2C sensors, the following pins are used by default:

- **SDA Pin**: GPIO21
- **SCL Pin**: GPIO22

The SDA pin of the sensor should be connected to GPIO21, and the SCL pin should be connected to GPIO22 on the ESP32.add

## UART Sensors

For UART sensors, the following pins are used by default:

- **TX Pin**: GPIO4
- **RX Pin**: GPIO5

The TX pin of the sensor should be connected to GPIO5, and the RX pin should be connected to GPIO4 on the ESP32.

## ESP-IDF Documentation

For more details on the ESP-IDF framework, refer to the official documentation:  
[ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)