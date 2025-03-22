#include "hmc5883l.h"
#include "esp_log.h"

using namespace hmc;

#define TAG "HMC5883L"

inline int16_t get_short(uint8_t* buffer, int offset) {
    return (buffer[offset] << 8 ) + buffer[offset + 1];
}

HMC5883L::HMC5883L() : I2Cdev(ADDRESS), Calibrate::Calibrate() {}

HMC5883L::HMC5883L(uint8_t address) : I2Cdev(address), Calibrate::Calibrate() {}

HMC5883L::~HMC5883L() {}

bool HMC5883L::init() {
    if (!check_device() || i2c_read8(REG_ID_A) != ID_A || i2c_read8(REG_ID_B) != ID_B || i2c_read8(REG_ID_C) != ID_C) {
        ESP_LOGE(TAG, "Failed to initialize. Please check the connections.");
        return false;
    }

    i2c_write8(REG_MODE, MODE_CONT);
    set_configs();

    open_storage("hmc5883l_calib");

    return true;
}

void HMC5883L::calibrate() {
    calibration_data_t calib_data;
    bool result = get_nvs_data<calibration_data_t>(&calib_data);

    if (result) {
        this->mag_offsets[0] = calib_data.mag_x;
        this->mag_offsets[1] = calib_data.mag_y;
        this->mag_offsets[2] = calib_data.mag_z;
    } else {
        for (int i = 0; i < 3; i++) {
        this->mag_offsets[i] = 0.0;
        }

        for (int i = 0; i < SAMPLE_SIZE; i++) {
            std::array<int16_t, 3> mag_values = get_raw_mag();

            for (int i = 0; i < 3; i++) {
                this->mag_offsets[i] += mag_values[i];
            }

            delay(1);
        }

        for (int i = 0; i < 3; i++) {
            this->mag_offsets[i] /= SAMPLE_SIZE;
        }

        calib_data.mag_x = this->mag_offsets[0];
        calib_data.mag_y = this->mag_offsets[1];
        calib_data.mag_z = this->mag_offsets[2];

        set_nvs_data<calibration_data_t>(calib_data);
    }
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

    i2c_write8(REG_CONFIG_A, config_A.get());
    i2c_write8(REG_CONFIG_B, config_B.get());
}

std::array<double, 3> HMC5883L::get_magXYZ() {
    std::array<int16_t, 3> raw_mag_values = get_raw_mag();

    std::array<double, 3> magXYZ;
    for (int i = 0; i < 3; i++) {
        magXYZ[i] = (double)raw_mag_values[i] / this->mag_resolution;
    }

    return magXYZ;
}

std::array<int16_t, 3> HMC5883L::get_raw_mag() {
    std::array<int16_t, 3> mag_values;

    uint8_t buffer[6];
    i2c_read_data(REG_DATA, buffer, 6);

    for (int i = 0; i < 3; i++) {
        mag_values[i] = get_short(buffer, i << 1);
    }

    return mag_values;
}