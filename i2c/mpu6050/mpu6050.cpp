#include "mpu6050.h"
#include "math.h"
#include "esp_timer.h"
#include "esp_log.h"

using namespace mpu;

#include TAG "MPU6050"

inline int16_t get_short(uint8_t* buffer, int offset) {
    return (buffer[offset] << 8 ) + buffer[offset + 1];
}

MPU6050::MPU6050() : I2Cdev(ADDRESS), Calibrate::Calibrate() {}

MPU6050::MPU6050(uint8_t address) : I2Cdev(address), Calibrate::Calibrate() {}

bool MPU6050::init() {
    if (!check_device() || i2c_read8(REG_ID) != CHIP_ID) {
        ESP_LOGE(TAG, "Failed to initialize. Please check the connections.");
        return false;
    }

    open_storage("mpu6050_calib");

    i2c_write8(REG_PWR_MGMT_1, pwr_mgmt.reset());
    i2c_write8(REG_SMPLRT_DIV, 0x00);
    set_configs();

    delay(100);

    return true;
}

MPU6050::~MPU6050() {
    stop();
}

void MPU6050::calibrate() {
    calibration_data_t calib_data;
    bool result = get_nvs_data<calibration_data_t>(&calib_data);

    if (result) {
        this->gyro_offsets[0] = calib_data.gyro_x;
        this->gyro_offsets[1] = calib_data.gyro_y;
        this->gyro_offsets[2] = calib_data.gyro_z;
    } else {
        for (int i = 0; i < 3; i++) {
            this->gyro_offsets[i] = 0.0;
        }

        for (int i = 0; i < SAMPLE_SIZE; i++) {
            std::array<int16_t, 3> gyro_values = get_raw_gyro();

            for (int i = 0; i < 3; i++) {
                this->gyro_offsets[i] += gyro_values[i];
            }

            delay(this->sampling_delay);
        }

        for (int i = 0; i < 3; i++) {
            this->gyro_offsets[i] /= SAMPLE_SIZE;
        }

        calib_data.gyro_x = this->gyro_offsets[0];
        calib_data.gyro_y = this->gyro_offsets[1];
        calib_data.gyro_z = this->gyro_offsets[2];

        set_nvs_data<calibration_data_t>(calib_data);
    }
}

void MPU6050::begin() {
    if (!this->data_mutex) {
        this->data_mutex = xSemaphoreCreateMutex();
    }

    if (!this->update_task_handle) {
        dt_timer = esp_timer_get_time();
        xTaskCreate(update_task_entry_point, "MPU6050_UPDATE", 2048, this, 5, &this->update_task_handle);
    }
}

void MPU6050::stop() {
    if (this->data_mutex) {
        if(xSemaphoreGetMutexHolder(this->data_mutex)) 
            xSemaphoreGive(this->data_mutex);

        vSemaphoreDelete(this->data_mutex);
    }

    if (this->update_task_handle) {
        vTaskDelete(this->update_task_handle);
        this->update_task_handle = NULL;
    }
}

void MPU6050::set_configs(const pwr_mngmt_t& pwr_mgmt, const config_t& config) {
    this->pwr_mgmt = pwr_mgmt;
    this->config = config;

    switch(config.bandwidth) {
        case BAND_260_HZ:
            this->sampling_delay = 1.0;
            break;
        case BAND_184_HZ:
            this->sampling_delay = 2.0;
            break;
        case BAND_94_HZ:
            this->sampling_delay = 4.0;
            break;
        case BAND_44_HZ:
            this->sampling_delay = 5.0;
            break;
        case BAND_21_HZ:
            this->sampling_delay = 9.0;
            break;
        case BAND_10_HZ:
            this->sampling_delay = 14.0;
            break;
        case BAND_5_HZ:
            this->sampling_delay = 20.0;
            break;
    }

    switch (config.gyro_range) {
        case GYRO_250_DPS:
            this->gyro_resolution = 131.072;
            break;
        case GYRO_500_DPS:
            this->gyro_resolution = 65.536;
            break;
        case GYRO_1000_DPS:
            this->gyro_resolution = 32.768;
            break;
        case GYRO_2000_DPS:
            this->gyro_resolution = 16.384;
    }

    switch (config.acc_range) {
        case ACCEL_2_G:
            this->acc_resolution = 16384.0;
            break;
        case ACCEL_4_G:
            this->acc_resolution = 8192.0;
            break;
        case ACCEL_8_G:
            this->acc_resolution = 4096.0;
            break;
        case ACCEL_16_G:
            this->acc_resolution = 2048.0;
    }

    i2c_write8(REG_PWR_MGMT_1, pwr_mgmt.get());
    i2c_write8(REG_CONFIG, config.bandwidth);
    i2c_write8(REG_GYRO_CONFIG, config.gyro_range << 3);
    i2c_write8(REG_ACCEL_CONFIG, config.acc_range << 3);
}

std::array<int16_t, 3> MPU6050::get_raw_gyro() {
    std::array<int16_t, 3> gyro_values;

    uint8_t buffer[6];
    i2c_read_data(REG_GYRO, buffer, 6);

    for (int i = 0; i < 3; i++) {
        gyro_values[i] = get_short(buffer, i << 1);
    }

    return gyro_values;
}

std::array<int16_t, 3> MPU6050::get_raw_acc() {
    std::array<int16_t, 3> acc_values;

    uint8_t buffer[6];
    i2c_read_data(REG_ACCEL, buffer, 6);

    for (int i = 0; i < 3; i++) {
        acc_values[i] = get_short(buffer, i << 1);
    }

    return acc_values;
}

void MPU6050::update_task_entry_point(void* obj) {
    MPU6050* mpu_instance = static_cast<MPU6050*>(obj);

    for(;;) {
        mpu_instance->update_task();
        delay(mpu_instance->sampling_delay);
    }
    vTaskDelete(NULL);
}

void MPU6050::update_task() {
    std::array<int16_t, 3> gyro = get_raw_gyro();
    std::array<int16_t, 3> acc = get_raw_acc();

    double gyroX = (gyro[0] - gyro_offsets[0]) / gyro_resolution;
    double gyroY = (gyro[1] - gyro_offsets[1]) / gyro_resolution;
    double gyroZ = (gyro[2] - gyro_offsets[2]) / gyro_resolution;
    
    double accX = ((double)acc[0]) / acc_resolution;
    double accY = ((double)acc[1]) / acc_resolution;
    double accZ = ((double)acc[2]) / acc_resolution;

    double dt = (esp_timer_get_time() - dt_timer) * 1e-6;

    double acc_roll = atan2(accY, accZ + abs(accX)) * RAD_2_DEG;
    double acc_pitch = -atan2(accX, accZ + abs(accY)) * RAD_2_DEG;

    if (xSemaphoreTake(this->data_mutex, portMAX_DELAY)) {
        this->roll = 0.98 * (this->roll + gyroX * dt) + 0.02 * acc_roll;
        this->pitch = 0.98 * (this->pitch + gyroY * dt) + 0.02 * acc_pitch;
        this->yaw += gyroZ * dt;

        xSemaphoreGive(this->data_mutex);
    }

    dt_timer = esp_timer_get_time();
}

double MPU6050::get_roll() {
    static double roll_val = 0.0;

    if (xSemaphoreTake(this->data_mutex, 0) == pdTRUE) {
        roll_val = this->roll;
        xSemaphoreGive(this->data_mutex);
    }
    return roll_val;
}

double MPU6050::get_pitch() {
    static double pitch_val = 0.0;

    if (xSemaphoreTake(this->data_mutex, 0) == pdTRUE) {
        pitch_val = this->pitch;
        xSemaphoreGive(this->data_mutex);
    }
    return pitch_val;
}

double MPU6050::get_yaw() {
    static double yaw_val = 0.0;

    if (xSemaphoreTake(this->data_mutex, 0) == pdTRUE) {
        yaw_val = this->yaw;
        xSemaphoreGive(this->data_mutex);
    }
    return yaw_val;
}