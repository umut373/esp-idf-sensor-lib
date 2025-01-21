#ifndef HMC5883L_H_
#define HMC5883L_H_

#include "i2c_dev.h"
#include "calibrate.h"

namespace hmc {
    #define SAMPLE_SIZE 5000

    const uint8_t ADDRESS = 0x0D; // i2c address
    const uint8_t ID_A = 0x48; // A id number
    const uint8_t ID_B = 0x34; // B id number
    const uint8_t ID_C = 0x33; // C id number

    enum register_t : uint8_t {
        REG_CONFIG_A = 0x00,
        REG_CONFIG_B = 0x01,
        REG_MODE = 0x02,
        REG_DATA = 0x03,
        REG_ID_A = 0x0A,
        REG_ID_B = 0x0B,
        REG_ID_C = 0x0C
    };

    enum sample_size_t : uint8_t {
        SAMPLE_SIZE_1 = 0b00,
        SAMPLE_SIZE_2 = 0b01,
        SAMPLE_SIZE_4 = 0b10,
        SAMPLE_SIZE_8 = 0b11
    };

    enum output_rate_t : uint8_t {
        OUTPUT_RATE_0_75 = 0b000,
        OUTPUT_RATE_1_5 = 0b001,
        OUTPUT_RATE_3 = 0b010,
        OUTPUT_RATE_7_5 = 0b011,
        OUTPUT_RATE_15 = 0b100,
        OUTPUT_RATE_30 = 0b101,
        OUTPUT_RATE_75 = 0b110
    };

    enum mag_gain_t : uint8_t {
        MAG_GAIN_0_88 = 0b000,
        MAG_GAIN_1_3 = 0b001,
        MAG_GAIN_1_9 = 0b010,
        MAG_GAIN_2_5 = 0b011,
        MAG_GAIN_4_0 = 0b100,
        MAG_GAIN_4_7 = 0b101,
        MAG_GAIN_5_6 = 0b110,
        MAG_GAIN_8_1 = 0b111
    };

    enum mode_t : uint8_t {
        MODE_CONT = 0b00,
        MODE_SINGLE = 0b01,
        MODE_IDLE = 0b10
    };

    struct config_a_t {
        sample_size_t sample_size : 2;
        output_rate_t output_rate : 3;

        uint8_t get() const {
            return (0x0 << 7) | (sample_size << 5) | (output_rate << 2) | 0x00;
        }
    };

    struct config_b_t {
        mag_gain_t mag_gain : 3;

        uint8_t get() const {
            return (mag_gain << 5) | 0x00;
        }
    };

    struct calibration_data_t {
        double mag_x = 0.0;
        double mag_y = 0.0;
        double mag_z = 0.0;
    };


    class HMC5883L : public I2Cdev, public Calibrate<calibration_data_t> {
        int16_t mag_resolution;
        double mag_offsets[3];

    public:
        HMC5883L();
        HMC5883L(uint8_t address);
        virtual ~HMC5883L();

        virtual bool init() override;
        virtual void calibrate() override;
        virtual void calibrate(const calibration_data_t& calib_data) override;

        void set_configs(const config_a_t& config_A = {SAMPLE_SIZE_8, OUTPUT_RATE_75}, const config_b_t& config_B = {MAG_GAIN_1_3});

        double* get_magXYZ();

    private:
        int16_t* get_raw_mag();
    };
} // namespace hmc

#endif // HMC5883L_H_