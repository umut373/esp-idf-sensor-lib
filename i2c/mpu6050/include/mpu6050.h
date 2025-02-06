#ifndef MPU6050_H_
#define MPU6050_H_

#include "i2c_dev.h"
#include "calibrate.h"
#include <array>

namespace mpu {
    #define RAD_2_DEG       57.29578
    #define FILTER_ALPHA    0.98

    #define SAMPLE_SIZE     5000

    const uint8_t ADDRESS = 0x68; // i2c address
    const uint8_t CHIP_ID = 0x68; // id number

    enum register_t : uint8_t {
        REG_ID = 0x75,
        REG_ACCEL = 0x3B,
        REG_GYRO = 0x43,
        REG_SMPLRT_DIV = 0x19,
        REG_CONFIG = 0x1A,
        REG_GYRO_CONFIG = 0x1B,
        REG_ACCEL_CONFIG = 0x1C,
        REG_PWR_MGMT_1 = 0x6B
    };

    enum clock_select_t : uint8_t {
        INTR_8MHz = 0b000,
        PLL_GYRO_X = 0b001,
        PLL_GYRO_Y = 0b010,
        PLL_GYRO_Z = 0b011,
        PLL_EXT_32K = 0b100,
        PLL_EXT_19MHz = 0b101,
        STOP = 0b111
    };

    enum bandwidth_t : uint8_t {
        BAND_260_HZ = 0b000,
        BAND_184_HZ = 0b001,
        BAND_94_HZ = 0b010,
        BAND_44_HZ = 0b011,
        BAND_21_HZ = 0b100,
        BAND_10_HZ = 0b101,
        BAND_5_HZ = 0b110
    };

    enum gyro_fs_t : uint8_t {
        GYRO_250_DPS = 0b00,
        GYRO_500_DPS = 0b01,
        GYRO_1000_DPS = 0b10,
        GYRO_2000_DPS = 0b11
    };

    enum acc_fs_t : uint8_t {
        ACCEL_2_G = 0b00,
        ACCEL_4_G = 0b01,
        ACCEL_8_G = 0b10,
        ACCEL_16_G = 0b11
    };

    struct pwr_mngmt_t {
        uint8_t _reset : 1;
        uint8_t sleep : 1;
        uint8_t cycle : 1;
        uint8_t temp_dis : 1;
        clock_select_t clk_sel : 3;

        uint8_t get() const {
            return (_reset << 7) | (sleep << 6) | (cycle << 5) | (0 << 4) | (temp_dis << 3) | clk_sel;
        }

        uint8_t reset() {
            return (1 << 7);
        }
    };

    struct config_t {
        bandwidth_t bandwidth : 3;
        gyro_fs_t gyro_range : 2;
        acc_fs_t acc_range : 2;
    };

    struct calibration_data_t {
        double gyro_x = 0.0;
        double gyro_y = 0.0;
        double gyro_z = 0.0;
    };

    class MPU6050: public I2Cdev, public Calibrate {
        TaskHandle_t update_task_handle = NULL;

        pwr_mngmt_t pwr_mgmt;
        config_t config;

        double gyro_resolution, acc_resolution;
        double sampling_delay;
        double gyro_offsets[3];
        double roll, pitch, yaw;

        uint64_t dt_timer;

    public:
        MPU6050();
        MPU6050(uint8_t address);
        ~MPU6050();

        virtual bool init() override;
        virtual void calibrate() override;

        void begin();
        void stop();

        void set_configs(const pwr_mngmt_t& pwr_mgmt = { 0, 0, 0, 1, PLL_GYRO_X }, const config_t& config = {BAND_5_HZ, GYRO_500_DPS, ACCEL_4_G});

        double get_roll();
        double get_pitch();
        double get_yaw();

    private:
        std::array<int16_t, 3> get_raw_gyro();
        std::array<int16_t, 3> get_raw_acc();

        static void update_task_entry_point(void* obj);
        void update_task();
    };
} // namespace mpu

#endif // MPU6050_H_