#ifndef BMP180_H_
#define BMP180_H_

#include "i2c_dev.h"
#include "calibrate.h"

namespace bmp {
    #define SAMPLE_SIZE 5000

    const uint8_t ADDRESS = 0x77; // i2c address
    const uint8_t CHIP_ID = 0x55; // id number

    enum register_t : uint8_t {
        REG_ID = 0xD0,
        REG_CALIB = 0xAA,
        REG_MEAS = 0xF4,
        REG_MSB = 0xF6,
        REG_LSB = 0xF7,
        REG_XLSB = 0xF8,
        CRV_TEMP = 0x2E,
        CRV_PRES = 0x34
    };

    struct cal_params_t {
        int16_t AC1;
        int16_t AC2;
        int16_t AC3;
        uint16_t AC4;
        uint16_t AC5;
        uint16_t AC6;

        int16_t B1;
        int16_t B2;

        int16_t MB;
        int16_t MC;
        int16_t MD;
    };

    enum resulation_mode_t : uint8_t {
        ULTRA_LOW = 0x00,
        STANDART = 0x01,
        HIGH_RES = 0x02,
        ULTRA_HIGH_RES = 0x03
    };


    class BMP180 : public I2Cdev, public Calibrate {
        resulation_mode_t oss;
        cal_params_t cal_params;
        double p0;

    public:
        BMP180();
        BMP180(uint8_t address);
        BMP180(resulation_mode_t oss);
        BMP180(uint8_t address, resulation_mode_t oss);

        virtual bool init() override;
        virtual void calibrate() override;

        double get_temperature();
        double get_pressure();
        double get_altitude();

    private:
        void read_calibration_parameters();

        double get_UT();
        double get_UP();
    };
} // namespace bmp

#endif // BMP180_H_