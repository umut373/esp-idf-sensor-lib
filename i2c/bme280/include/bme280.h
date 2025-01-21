#ifndef BME280_H_
#define BME280_H_

#include "i2c_dev.h"

namespace bme {
    #define SAMPLE_SIZE 5000

    static const uint8_t ADDRESS = 0x76; // i2c address
    static const uint8_t CHIP_ID = 0x60; // id number

    enum register_t : uint8_t {
        REG_ID = 0xD0,
        REG_CALIB = 0x88,
        REG_SOFTRESET = 0xE0,
        REG_STATUS = 0XF3,
        REG_CONTROL = 0xF4,
        REG_CONFIG = 0xF5,
        REG_PRESS_DATA = 0xF7,
        REG_TEMP_DATA = 0xFA
    };

    struct cal_params_t {
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;

        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;
    };

    enum sensor_mode_t : uint8_t {
        MODE_SLEEP = 0b00,
        MODE_FORCED = 0b01,
        MODE_NORMAL = 0b11
    };

    enum sensor_sampling_t : uint8_t {
        SAMPLING_NONE = 0b000,
        SAMPLING_X1 = 0b001,
        SAMPLING_X2 = 0b010,
        SAMPLING_X4 = 0b011,
        SAMPLING_X8 = 0b100,
        SAMPLING_X16 = 0b101
    };

    enum sensor_filter_t : uint8_t {
        FILTER_OFF = 0b000,
        FILTER_X2 = 0b001,
        FILTER_X4 = 0b010,
        FILTER_X8 = 0b011,
        FILTER_X16 = 0b100
    };

    enum standby_duration_t : uint8_t {
        STANDBY_MS_0_5 = 0b000,
        STANDBY_MS_10 = 0b110,
        STANDBY_MS_20 = 0b111,
        STANDBY_MS_62_5 = 0b001,
        STANDBY_MS_125 = 0b010,
        STANDBY_MS_250 = 0b011,
        STANDBY_MS_500 = 0b100,
        STANDBY_MS_1000 = 0b101
    };

    struct config_t {
        standby_duration_t t_sb : 3;
        sensor_filter_t filter : 3;

        uint8_t get() const {
            return (t_sb << 5) | (filter << 2) | 0;
        }
    };

    struct meas_config_t {
        sensor_sampling_t osrs_t : 3;
        sensor_sampling_t osrs_p : 3;
        sensor_mode_t mode : 2;

        uint8_t get() const {
            return (osrs_t << 5) | (osrs_p << 2) | mode;
        }
    };


    class BME280 : public I2Cdev {
        config_t config_reg;
        meas_config_t meas_reg;

        cal_params_t cal_params;

    public:
        BME280();
        BME280(uint8_t address);

        virtual bool init() override;

        void set_configs(const config_t& config_reg = { STANDBY_MS_0_5, FILTER_X16 }, const meas_config_t& meas_reg = { SAMPLING_X16, SAMPLING_X16, MODE_NORMAL});

        double get_temperature();
        double get_pressure();

    private:
        int32_t t_fine;

        bool is_reading_calibration();
        void read_calibration_parameters();

        uint32_t get_adc_T();
        uint32_t get_adc_P();
    };
} // namespace bme

#endif // BME280_H_