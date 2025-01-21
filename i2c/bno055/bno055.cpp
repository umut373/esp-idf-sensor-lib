#include "bno055.h"
#include "math.h"

using namespace imu;

inline int16_t get_short_le(uint8_t* buffer, int offset) {
    return (buffer[offset + 1] << 8) | buffer[offset];
}

BNO055::BNO055() : I2Cdev(ADDRESS), Calibrate::Calibrate() {}

BNO055::BNO055(uint8_t address) : I2Cdev(address), Calibrate::Calibrate() {}

bool BNO055::init() {
    if (read8(REG_CHIP_ID) != CHIP_ID || read8(REG_ACC_ID) != ACC_ID || read8(REG_MAG_ID) != MAG_ID || read8(REG_GYR_ID) != GYR_ID)
        return false;

    ESP_ERROR_CHECK(nvs_open("bno055_calib", NVS_READWRITE, &this->nvs_handle));

    set_configs();
    delay(20);

    return true;
}

void BNO055::calibrate() {
    calibration_data_t calib_data;
    size_t size = sizeof(calib_data);

    esp_err_t result = nvs_get_blob(this->nvs_handle, "bno055_calib", &calib_data, &size);

    if (result == ESP_OK) {
        write8(REG_OPR_MODE, OPR_MODE_CONFIG);
        delay(10);

        const uint8_t* offsets = reinterpret_cast<const uint8_t*>(&calib_data);
        for (uint8_t i = 0; i < 22; i++) {
            write8(REG_OFFSET + i, offsets[i]);
        }

        write8(REG_OPR_MODE, this->mode);
        delay(10);
    } else {
        while(!is_fully_calibrated()) {
            delay(500);
        }

        write8(REG_OPR_MODE, OPR_MODE_CONFIG);
        delay(10);

        uint8_t calib_buffer[22];
        read_data(REG_OFFSET, calib_buffer, 22);

        write8(REG_OPR_MODE, this->mode);
        delay(10);

        int16_t* calib_ptr = reinterpret_cast<int16_t*>(&calib_data);
        for (int i = 0; i < 11; i++) {
            calib_ptr[i] = get_short_le(calib_buffer, i << 1);
        }

        nvs_set_blob(this->nvs_handle, "bno055_calib", &calib_data, size);
    }
}

void BNO055::set_configs(const config_t& config) {
    write8(REG_OPR_MODE, OPR_MODE_CONFIG);
    delay(10);
    write8(REG_UNIT_SEL, config.unit_select);
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
            read_data(REG_ACC_DATA, buffer, 6);

            data_vector.reserve(3);
            for (int i = 0; i < 3; i++) {
                data_vector.push_back(get_short_le(buffer, i << 1) / 100.0);
            }
            break;
        }
        case VEC_MAG: {
            uint8_t buffer[6];
            read_data(REG_MAG_DATA, buffer, 6);

            data_vector.reserve(3);
            for (int i = 0; i < 3; i++) {
                data_vector.push_back(get_short_le(buffer, i << 1) / 16.0);
            }
            break;
        }
        case VEC_GYR: {
            uint8_t buffer[6];
            read_data(REG_GYRO_DATA, buffer, 6);

            data_vector.reserve(3);
            for (int i = 0; i < 3; i++) {
                data_vector.push_back(get_short_le(buffer, i << 1) / 16.0);
            }
            break;
        }
        case VEC_EUL: {
            uint8_t buffer[6];
            read_data(REG_EUL_DATA, buffer, 6);

            data_vector.reserve(3);
            for (int i = 0; i < 3; i++) {
                data_vector.push_back(get_short_le(buffer, i << 1) / 16.0);
            }
            break;
        }
        case VEC_QUA: {
            uint8_t buffer[8];
            read_data(REG_QUA_DATA, buffer, 8);

            data_vector.reserve(4);
            for (int i = 0; i < 4; i++) {
                data_vector.push_back(get_short_le(buffer, i << 1) / pow(2.0, 14));
            }
            break;
        }
        case VEC_LIA: {
            uint8_t buffer[6];
            read_data(REG_LIA_DATA, buffer, 6);

            data_vector.reserve(3);
            for (int i = 0; i < 3; i++) {
                data_vector.push_back(get_short_le(buffer, i << 1) / 100.0);
            }
            break;
        }
        case VEC_GRV: {
            uint8_t buffer[6];
            read_data(REG_GRV_DATA, buffer, 6);

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
    uint8_t buffer = read8(REG_CALIB_STAT);

    for (int i = 3; i >= 0; i--) {
        data_array[3-i] = (buffer >> (2*i)) & 0x03;
    }
    
    return data_array;
}

void BNO055::set_mode(opr_mode_t mode) {
    this->mode = mode;
    write8(REG_OPR_MODE, mode);
}

void BNO055::set_axis_remap(remap_t remap) {
    switch(remap) {
        case REMAP_P0:
            write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P0);
            delay(10);
            write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P0);
            break;
        case REMAP_P1:
            write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P1);
            delay(10);
            write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P1);
            break;
        case REMAP_P2:
            write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P2);
            delay(10);
            write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P2);
            break;
        case REMAP_P3:
            write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P3);
            delay(10);
            write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P3);
            break;
        case REMAP_P4:
            write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P4);
            delay(10);
            write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P4);
            break;
        case REMAP_P5:
            write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P5);
            delay(10);
            write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P5);
            break;
        case REMAP_P6:
            write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P6);
            delay(10);
            write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P6);
            break;
        case REMAP_P7:
            write8(REG_AXIS_REMAP_CONF, AXIS_REMAP_CONF_P7);
            delay(10);
            write8(REG_AXIS_REMAP_SIGN, AXIS_REMAP_SIGN_P7);
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