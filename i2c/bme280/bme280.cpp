#include "bme280.h"
#include "math.h"

using namespace bme;

inline uint16_t get_short_le(uint8_t* buffer, int offset) {
    return (buffer[offset + 1] << 8 ) + buffer[offset];
}

BME280::BME280() : I2Cdev::I2Cdev(ADDRESS) {}

BME280::BME280(uint8_t address) : I2Cdev::I2Cdev(address) {}

bool BME280::init() {
    if (read8(REG_ID) != CHIP_ID) {
        return false;
    }

    write8(REG_SOFTRESET, 0xB6);
    delay(10);

    while (is_reading_calibration()) {
        delay(10);
    }

    read_calibration_parameters();
    set_configs();

    delay(100);

    return true;
}

bool BME280::is_reading_calibration() {
    const uint8_t status = read8(REG_STATUS);

    return (status & 1) != 0;
}

void BME280::read_calibration_parameters() {
    uint8_t coeffs[24];
    read_data(REG_CALIB, coeffs, 24);

    uint16_t* ptr = reinterpret_cast<uint16_t*>(&this->cal_params);
    size_t length = sizeof(coeffs) / sizeof(uint16_t);

    for (int i = 0; i < length; i++) {
        ptr[i] = get_short_le(coeffs, i << 1);
    }
}

void BME280::set_configs(const config_t& config_reg, const meas_config_t& meas_reg) {
    this->config_reg = config_reg;
    this->meas_reg = meas_reg;

    write8(REG_CONTROL, MODE_SLEEP);

    write8(REG_CONFIG, config_reg.get());
    write8(REG_CONTROL, meas_reg.get());
}

double BME280::get_temperature() {
    int32_t adc_T, var1, var2, T;

    adc_T = get_adc_T();
    if (adc_T == 0x800000)
        return NAN;
    adc_T >>= 4;

    var1 = ((adc_T >> 3) - ((int32_t)this->cal_params.dig_T1 << 1)) * ((int32_t)this->cal_params.dig_T2) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)this->cal_params.dig_T1) * ((adc_T >> 4) - (int32_t)this->cal_params.dig_T1)) >> 12) * (int32_t)this->cal_params.dig_T3) >> 14;

    this->t_fine = var1 + var2;
    T = (this->t_fine * 5 + 128) >> 8;
    return T / 100.0;
}

double BME280::get_pressure() {
    int64_t adc_P, var1, var2, P;

    get_temperature();

    adc_P = get_adc_P();
    if (adc_P == 0x800000)
        return NAN;
    adc_P >>= 4;

    var1 = (int64_t)this->t_fine - 128000;
    var2 = pow(var1, 2) * (int64_t)this->cal_params.dig_P6;
    var2 += (var1 * (int64_t)this->cal_params.dig_P5) << 17;
    var2 += (int64_t)this->cal_params.dig_P4 << 35;

    var1 = ((var1 * var1 * (int64_t)this->cal_params.dig_P3) >> 8) + ((var1 * (int64_t)this->cal_params.dig_P2) << 12);
    var1 = ((((int64_t)1 << 47) + var1) * (int64_t)this->cal_params.dig_P1) >> 33;
    if (var1 == 0) 
        return 0;
    
    P = 1048576 - adc_P;
    P = (((P << 31) - var2) * 3125) / var1;

    var1 = ((int64_t)this->cal_params.dig_P9 * (P >> 13) * (P >> 13)) >> 25;
    var2 = ((int64_t)this->cal_params.dig_P8 * P) >> 19;
    P = ((P + var1 + var2) >> 8) + ((int64_t)this->cal_params.dig_P7 << 4);

    return (uint32_t)P / 256.0;
}

uint32_t BME280::get_adc_T() {
    uint8_t buffer[3];
    read_data(REG_TEMP_DATA, buffer, 3);

    return ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];
}

uint32_t BME280::get_adc_P() {
    uint8_t buffer[3];
    read_data(REG_PRESS_DATA, buffer, 3);

    return ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];
}