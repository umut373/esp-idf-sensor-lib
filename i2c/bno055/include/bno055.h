#ifndef BNO055_H_
#define BNO055_H_

#include "i2c_dev.h"
#include "calibrate.h"
#include <vector>
#include <array>

namespace imu {
    const uint8_t ADDRESS = 0x28; // i2c address
    const uint8_t CHIP_ID = 0xA0; // id number
    const uint8_t ACC_ID = 0xFB; // acc id number
    const uint8_t MAG_ID = 0x32; // mag id number
    const uint8_t GYR_ID = 0x0F; // gyro id number

    enum register_t : uint8_t {
        REG_CHIP_ID = 0x00,
        REG_ACC_ID = 0x01,
        REG_MAG_ID = 0x02,
        REG_GYR_ID = 0x03,
        REG_ACC_DATA = 0x08,
        REG_MAG_DATA = 0x0E,
        REG_GYRO_DATA = 0x14,
        REG_EUL_DATA = 0x1A,
        REG_QUA_DATA = 0x20,
        REG_LIA_DATA = 0x28,
        REG_GRV_DATA = 0x2E,
        REG_CALIB_STAT = 0x35,
        REG_UNIT_SEL = 0x3B,
        REG_OPR_MODE = 0x3D,
        REG_PWR_MODE = 0x3E,
        REG_AXIS_REMAP_CONF = 0x41,
        REG_AXIS_REMAP_SIGN = 0x42,
        REG_OFFSET = 0x55
    };

    enum pwr_mode_t : uint8_t {
        PWR_MODE_NORMAL = 0b00,
        PWR_MODE_LOW = 0b01,
        PWR_MODE_SUSPEND = 0b10
    };

    enum opr_mode_t : uint8_t {
        OPR_MODE_CONFIG = 0b000,
        OPR_MODE_ACCONLY = 0b0001,
        OPR_MODE_MAGONLY = 0b0010,
        OPR_MODE_GYRONLY = 0b0011,
        OPR_MODE_ACCMAG = 0b0100,
        OPR_MODE_ACCGYRO = 0b0101,
        OPR_MODE_MAGGYRO = 0b0110,
        OPR_MODE_AMG = 0b0111,
        OPR_MODE_IMU = 0b1000,
        OPR_MODE_COMPASS = 0b1001,
        OPR_MODE_M4G = 0b1010,
        OPR_MODE_INDOF_FMC_OFF = 0b1011,
        OPR_MODE_INDOF = 0b1100
    };

    enum axis_remap_conf_t : uint8_t {
        AXIS_REMAP_CONF_P0 = 0x21,
        AXIS_REMAP_CONF_P1 = 0x24,
        AXIS_REMAP_CONF_P2 = 0x24,
        AXIS_REMAP_CONF_P3 = 0x21,
        AXIS_REMAP_CONF_P4 = 0x24,
        AXIS_REMAP_CONF_P5 = 0x21,
        AXIS_REMAP_CONF_P6 = 0x21,
        AXIS_REMAP_CONF_P7 = 0x24
    };

    enum axis_remap_sign_t : uint8_t {
        AXIS_REMAP_SIGN_P0 = 0x04,
        AXIS_REMAP_SIGN_P1 = 0x00,
        AXIS_REMAP_SIGN_P2 = 0x06,
        AXIS_REMAP_SIGN_P3 = 0x02,
        AXIS_REMAP_SIGN_P4 = 0x03,
        AXIS_REMAP_SIGN_P5 = 0x01,
        AXIS_REMAP_SIGN_P6 = 0x07,
        AXIS_REMAP_SIGN_P7 = 0x05
    };

    enum remap_t {
        REMAP_P0,
        REMAP_P1,
        REMAP_P2,
        REMAP_P3,
        REMAP_P4,
        REMAP_P5,
        REMAP_P6,
        REMAP_P7
    } ;

    enum vector_t {
        VEC_ACC,
        VEC_MAG,
        VEC_GYR,
        VEC_EUL,
        VEC_QUA,
        VEC_LIA,
        VEC_GRV,
    };

    struct config_t {
        uint8_t unit_select;
        remap_t axis_remap; 
        opr_mode_t mode;
    };

    struct calibration_data_t {
        uint16_t acc_x, acc_y, acc_z;
        uint16_t mag_x, mag_y, mag_z;
        uint16_t gyro_x,  gyro_y, gyro_z;
        uint16_t acc_radius;
        uint16_t mag_radius;
    };

class BNO055 : public I2Cdev, public Calibrate<calibration_data_t> {
    opr_mode_t mode;
    calibration_data_t offsets;

public:
    BNO055();
    BNO055(uint8_t address);

    virtual bool init() override;
    virtual void calibrate() override;
    virtual void calibrate(const calibration_data_t& calib_data) override;
    void set_configs(const config_t& config = {0x80, REMAP_P0, OPR_MODE_INDOF});

    std::array<uint8_t, 4> get_calib_status();
    std::vector<double> get_data_vector(vector_t vector);

private:
    void set_mode(opr_mode_t mode);
    void set_axis_remap(remap_t remap);

    bool is_fully_calibrated();
};
} // namespace imu

#endif // BNO055_H_