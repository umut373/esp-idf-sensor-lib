#include "hmc5883l.h"

using namespace hmc;

inline int16_t get_short(uint8_t* buffer, int offset) {
    return (buffer[offset] << 8 ) + buffer[offset + 1];
}

HMC5883L::HMC5883L() : I2Cdev(ADDRESS), Calibrate::Calibrate() {}

HMC5883L::HMC5883L(uint8_t address) : I2Cdev(address), Calibrate::Calibrate() {}

HMC5883L::~HMC5883L() {}

bool HMC5883L::init() {
    if (read8(REG_ID_A) != ID_A || read8(REG_ID_B) != ID_B || read8(REG_ID_C) != ID_C)
        return false;

    write8(REG_MODE, MODE_CONT);
    set_configs();

    return true;
}

void HMC5883L::calibrate() {
    for (int i = 0; i < 3; i++) {
        this->mag_offsets[i] = 0.0;
    }

    for (int i = 0; i < SAMPLE_SIZE; i++) {
        int16_t* mag_values = get_raw_mag();

        for (int i = 0; i < 3; i++) {
            mag_offsets[i] += mag_values[i];
        }

        delete[] mag_values;
        delay(1);
    }

    for (int i = 0; i < 3; i++) {
        this->mag_offsets[i] /= SAMPLE_SIZE;
    }
}

void HMC5883L::calibrate(const calibration_data_t& calib_data) {
    mag_offsets[0] = calib_data.mag_x;
    mag_offsets[1] = calib_data.mag_y;
    mag_offsets[2] = calib_data.mag_z;
}

void HMC5883L::set_configs(const config_a_t& config_A, const config_b_t& config_B) {
    switch(config_B.mag_gain) {
        case MAG_GAIN_0_88: 
            this->mag_resolution = 1370;
            break;
        case MAG_GAIN_1_3:
            this->mag_resolution = 1090;
            break;
        case MAG_GAIN_1_9:
            this->mag_resolution = 820;
            break;
        case MAG_GAIN_2_5:
            this->mag_resolution = 660;
            break;
        case MAG_GAIN_4_0:
            this->mag_resolution = 440;
            break;
        case MAG_GAIN_4_7:
            this->mag_resolution = 390;
            break;
        case MAG_GAIN_5_6:
            this->mag_resolution = 330;
            break;
        case MAG_GAIN_8_1:
            this->mag_resolution = 230;
            break;
    }

    write8(REG_CONFIG_A, config_A.get());
    write8(REG_CONFIG_B, config_B.get());
}

double* HMC5883L::get_magXYZ() {
    int16_t* raw_mag_values = get_raw_mag();

    double* magXYZ = new double[3];
    for (int i = 0; i < 3; i++) {
        magXYZ[i] = (double)raw_mag_values[i] / this->mag_resolution;
    }
    delete raw_mag_values;

    return magXYZ;
}

int16_t* HMC5883L::get_raw_mag() {
    int16_t* mag_values = new int16_t[3];

    uint8_t buffer[6];
    read_data(REG_DATA, buffer, 6);

    for (int i = 0; i < 3; i++) {
        mag_values[i] = get_short(buffer, i << 1);
    }

    return mag_values;
}