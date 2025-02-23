#include "ina226.h"
#include "math.h"

using namespace ina;

INA226::INA226() : I2Cdev::I2Cdev(ADDRESS) {}

INA226::INA226(uint8_t address) : I2Cdev::I2Cdev(address) {}

bool INA226::init() {
    if (!check_device() || read16(REG_ID) != CHIP_ID) {
        return false;
    }

    set_configs();
    set_calibration();

    return true;
}

void INA226::set_configs(const config_t& config) {
    write16(REG_CONFIG, config.get());
}

void INA226::set_calibration(double max_current, double shunt_resistor) {
    this->current_LSB = max_current * pow(2, -15);
    uint16_t calibration = 0.00512 / (this->current_LSB * shunt_resistor);

    write16(REG_CALIB, calibration);
}

double INA226::get_bus_voltage() {
    int16_t raw_bus_voltage = read16(REG_BUS);
    return raw_bus_voltage * 1.25e-3;
}

double INA226::get_shunt_voltage() {
    int16_t raw_shunt_voltage = read16(REG_SHUNT);
    return raw_shunt_voltage * 2.5e-6;
}

double INA226::get_current() {
    int16_t raw_current = read16(REG_CURRENT);
    return raw_current * this->current_LSB;
}

double INA226::get_power() {
    int16_t raw_power = read16(REG_PWR);
    return raw_power * (this->current_LSB * 25);
}