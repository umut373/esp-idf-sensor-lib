#include "bno055.h"
#include "math.h"
#include "esp_log.h"

using namespace imu;

#define TAG "BNO055"

inline int16_t get_short_le(uint8_t* buffer, int offset) {
    return (buffer[offset + 1] << 8) | buffer[offset];
}

BNO055::BNO055() : I2Cdev(ADDRESS), Calibrate::Calibrate() {}

BNO055::BNO055(uint8_t address) : I2Cdev(address), Calibrate::Calibrate() {}

bool BNO055::init() {
    if (!check_device() || i2c_read8(REG_CHIP_ID) != CHIP_ID || i2c_read8(REG_ACC_ID) != ACC_ID || i2c_read8(REG_MAG_ID) != MAG_ID || i2c_read8(REG_GYR_ID) != GYR_ID) {
        ESP_LOGE(TAG, "Failed to initialize. Please check the connections.");
        return false;
    }

    open_storage("bno055_calib");

    set_configs();
    delay(20);

    return true;
}

void BNO055::calibrate() {
    calibration_data_t calib_data;
    bool result = get_nvs_data<calibration_data_t>(&calib_data);

    if (result) {
        reset();

        const uint8_t* offsets = reinterpret_cast<const uint8_t*>(&calib_data);
        for (uint8_t i = 0; i < 22; i++) {
            i2c_write8(REG_OFFSET + i, offsets[i]);
        }

        set_configs({this->remap, this->mode});
    } else {
        while(!is_fully_calibrated()) {
            delay(500);
        }

        i2c_write8(REG_OPR_MODE, OPR_MODE_CONFIG);
        delay(10);

        uint8_t calib_buffer[22];
        i2c_read_data(REG_OFFSET, calib_buffer, 22);

        i2c_write8(REG_OPR_MODE, this->mode);
        delay(10);

        int16_t* calib_ptr = reinterpret_cast<int16_t*>(&calib_data);
        for (int i = 0; i < 11; i++) {
            calib_ptr[i] = get_short_le(calib_buffer, i << 1);
        }

        set_nvs_data<calibration_data_t>(calib_data);
    }
}

void BNO055::set_configs(const config_t& config) {
    i2c_write8(REG_OPR_MODE, OPR_MODE_CONFIG);
    delay(10);
    i2c_write8(REG_UNIT_SEL, 0x80);
    delay(10);
    set_axis_remap(config.axis_remap);
    delay(10);
    set_mode(config.mode);
}

std::vector<double> BNO055::get_data_vector(vector_t vector) {
    std::vector<double> data_vector;

    switch (vector) {
        case VEC_ACC: {
            uint8_t buffer[6];
            i2c_read_data(REG_ACC_DATA, buffer, 6);

            data_vector.reserve(3);
            for (int i = 0; i < 3; i++) {
                data_vector.push_back(get_short_le(buffer, i << 1) / 100.0);
            }
            break;
        }
        case VEC_MAG: {
            uint8_t buffer[6];
            i2c_read_data(REG_MAG_DATA, buffer, 6);

            data_vector.reserve(3);
            for (int i = 0; i < 3; i++) {
                data_vector.push_back(get_short_le(buffer, i << 1) / 16.0);
            }
            break;
        }
        case VEC_GYR: {
            uint8_t buffer[6];
            i2c_read_data(REG_GYRO_DATA, buffer, 6);

            data_vector.reserve(3);
            for (int i = 0; i < 3; i++) {
                data_vector.push_back(get_short_le(buffer, i << 1) / 16.0);
            }
            break;
        }
        case VEC_EUL: {
            uint8_t buffer[6];
            i2c_read_data(REG_EUL_DATA, buffer, 6);

            data_vector.reserve(3);
            for (int i = 0; i < 3; i++) {
                data_vector.push_back(get_short_le(buffer, i << 1) / 16.0);
            }
            break;
        }
        case VEC_QUA: {
            uint8_t buffer[8];
            i2c_read_data(REG_QUA_DATA, buffer, 8);

            data_vector.reserve(4);
            for (int i = 0; i < 4; i++) {
                data_vector.push_back(get_short_le(buffer, i << 1) / pow(2.0, 14));
            }
            break;
        }
        case VEC_LIA: {
            uint8_t buffer[6];
            i2c_read_data(REG_LIA_DATA, buffer, 6);

            data_vector.reserve(3);
            for (int i = 0; i < 3; i++) {
                data_vector.push_back(get_short_le(buffer, i << 1) / 100.0);
            }
            break;
        }
        case VEC_GRV: {
            uint8_t buffer[6];
            i2c_read_data(REG_GRV_DATA, buffer, 6);

            data_vector.reserve(3);
            for (int i = 0; i < 3; i++) {
                data_vector.push_back(get_short_le(buffer, i << 1) / 100.0);
            }
            break;
        }
    }
    return data_vector;
}

std::array<uint8_t, 4> BNO055::get_calib_status() {
    std::array<uint8_t, 4> data_array;
    uint8_t buffer = i2c_read8(REG_CALIB_STAT);

    for (int i = 3; i >= 0; i--) {
        data_array[3-i] = (buffer >> (2*i)) & 0x03;
    }
    
    return data_array;
}

void BNO055::reset() {
    i2c_write8(REG_OPR_MODE, OPR_MODE_CONFIG);
    delay(10);
    i2c_write8(REG_SYS_TRIGGER, 0x20);

    // wait for the device to reboot
    while(!check_device()) {
        delay(1);
    }
}

void BNO055::set_mode(opr_mode_t mode) {
    this->mode = mode;
    i2c_write8(REG_OPR_MODE, mode);
}

void BNO055::set_axis_remap(remap_t remap) {
    this->remap = remap;

    switch(remap) {
        case REMAP_P0:
            i2c_write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P0);
            delay(10);
            i2c_write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P0);
            break;
        case REMAP_P1:
            i2c_write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P1);
            delay(10);
            i2c_write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P1);
            break;
        case REMAP_P2:
            i2c_write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P2);
            delay(10);
            i2c_write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P2);
            break;
        case REMAP_P3:
            i2c_write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P3);
            delay(10);
            i2c_write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P3);
            break;
        case REMAP_P4:
            i2c_write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P4);
            delay(10);
            i2c_write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P4);
            break;
        case REMAP_P5:
            i2c_write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P5);
            delay(10);
            i2c_write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P5);
            break;
        case REMAP_P6:
            i2c_write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P6);
            delay(10);
            i2c_write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P6);
            break;
        case REMAP_P7:
            i2c_write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P7);
            delay(10);
            i2c_write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P7);
            break;
    }
}

bool BNO055::is_fully_calibrated() {
    std::array<uint8_t, 4> calib_status = get_calib_status();

    uint8_t system = calib_status[0];
    uint8_t gyro = calib_status[1];
    uint8_t accel = calib_status[2];
    uint8_t mag = calib_status[3];

    switch (this->mode) {
        case OPR_MODE_ACCONLY:
            printf("accel=%d\n", accel);
            return (accel == 3);
        case OPR_MODE_MAGONLY:
            printf("mag=%d\n", mag);
            return (mag == 3);
        case OPR_MODE_GYRONLY:
        case OPR_MODE_M4G:
            printf("gyro=%d\n", gyro);
            return (gyro == 3);
        case OPR_MODE_ACCMAG:
        case OPR_MODE_COMPASS:
            printf("accel=%d, mag=%d\n", accel, mag);
            return (accel == 3 && mag == 3);
        case OPR_MODE_ACCGYRO:
        case OPR_MODE_IMU:
            printf("accel=%d, gyro=%d\n", accel, gyro);
            return (accel == 3 && gyro == 3);
        case OPR_MODE_MAGGYRO:
            printf("mag=%d, gyro=%d\n", mag, gyro);
            return (mag == 3 && gyro == 3);
        default:
            printf("system=%d, gyro=%d, accel=%d, mag=%d\n", system, gyro, accel, mag);
            return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
    }
}