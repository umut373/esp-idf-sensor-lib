#include "bmp180.h"
#include "math.h"
#include "esp_log.h"

using namespace bmp;

#define TAG "BMP180"

inline uint16_t get_short(uint8_t* buffer, int offset) {
    return (buffer[offset] << 8 ) + buffer[offset + 1];
}

BMP180::BMP180() : I2Cdev::I2Cdev(ADDRESS), Calibrate::Calibrate() {
    this->oss = ULTRA_HIGH_RES;
}

BMP180::BMP180(uint8_t address) : I2Cdev::I2Cdev(address), Calibrate::Calibrate() {
    this->oss = ULTRA_HIGH_RES;
}

BMP180::BMP180(resulation_mode_t oss) : I2Cdev::I2Cdev(ADDRESS), Calibrate::Calibrate() {
    this->oss = oss;
}

BMP180::BMP180(uint8_t address, resulation_mode_t oss) : I2Cdev::I2Cdev(address), Calibrate::Calibrate() {
    this->oss = oss;
}

bool BMP180::init() {
    if (!check_device() || read8(REG_ID) != CHIP_ID) {
        ESP_LOGE(TAG, "Failed to initialize. Please check the connections.");
        return false;
    }

    open_storage("bmp180_calib");

    read_calibration_parameters();
    return true;
}

void BMP180::read_calibration_parameters() {
    uint8_t coeffs[22];
    read_data(REG_CALIB, coeffs, 22);

    uint16_t* ptr = reinterpret_cast<uint16_t*>(&this->cal_params);
    size_t length = sizeof(coeffs) / sizeof(uint16_t);

    for (int i = 0; i < length; i++) {
        ptr[i] = get_short(coeffs, i << 1);
    }
}

void BMP180::calibrate() {
    uint64_t calib_data;
    bool result = get_nvs_data<uint64_t>(&calib_data);

    if (result) {
        this->p0 = std::bit_cast<double>(calib_data);
    } else {
        this->p0 = 0;
        double p0 = 0;
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            p0 += get_pressure();
        }
        this->p0 = p0 / SAMPLE_SIZE;
        set_nvs_data<uint64_t>(std::bit_cast<uint64_t>(this->p0));
    }
}

double BMP180::get_temperature() {
    double UT, c5, c6, mc, md, a;

    UT = get_UT();
    c5 = (pow(2.0, -15) * this->cal_params.AC5) / 160.0;
    c6 = this->cal_params.AC6;
    mc = (pow(2.0, 11) * this->cal_params.MC) / pow(160.0, 2);
    md = this->cal_params.MD / 160.0;
    
    a = c5 * (UT - c6);

    return a + (mc / (a + md));
}

double BMP180::get_pressure() {
    double c3, c4, b1;
    c3 = 160.0 * pow(2, -15) * this->cal_params.AC3;
    c4 = pow(10.0, -3) * pow(2, -15) * this->cal_params.AC4;
    b1 = pow(160, 2) * pow(2, -30) * this->cal_params.B1;

    double x0, x1, x2;
    x0 = this->cal_params.AC1;
    x1 = 160.0 * pow(2, -13) * this->cal_params.AC2;
    x2 = pow(160, 2) * pow(2, -25) * this->cal_params.B2;

    double y0, y1, y2;
    y0 = c4 * pow(2, 15);
    y1 = c4 * c3;
    y2 = c4 * b1;

    double p0, p1, p2;
    p0 = (3791.0 - 8.0) / 1600.0;
    p1 = 1.0 - (7357.0 * pow(2, -20));
    p2 = 3038.0 * 100.0 * pow(2, -36);

    double UP, T, s, x, y, z;

    UP = get_UP();
    T = get_temperature();
    s = T -25;
    x = x2 * pow(s, 2) + x1 * s + x0;
    y = y2 * pow(s, 2) + y1 * s + y0;

    z = (UP - x) / y;

    return (p2 * pow(z, 2) + p1 * z + p0) * 100.0;
}

double BMP180::get_altitude() {
    int32_t p = get_pressure();

    return 44330.0 * (1 - (pow(p / this->p0, 1 / 5.255)));
}

double BMP180::get_UT() {
    write8(REG_MEAS, CRV_TEMP);

    delay(5);

    uint8_t MSB = read8(REG_MSB);
    uint8_t LSB = read8(REG_LSB);

    return  (MSB * 256.0) + LSB;
}

double BMP180::get_UP() {
    write8(REG_MEAS, CRV_PRES + (oss << 6));

    delay(26);

    uint8_t MSB = read8(REG_MSB);
    uint8_t LSB = read8(REG_LSB);
    uint8_t XLSB = read8(REG_XLSB);

    return (MSB * 256.0) + LSB + (XLSB / 256.0);
}