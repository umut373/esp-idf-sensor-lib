#ifndef INA226_H_
#define INA226_H_

#include "i2c_dev.h"

namespace ina {
    const uint8_t ADDRESS = 0x40; // i2c address
    const uint16_t CHIP_ID = 0x5449; // id number

    enum register_t : uint8_t {
        REG_CONFIG = 0x00,
        REG_SHUNT = 0x01,
        REG_BUS = 0x02,
        REG_PWR = 0x03,
        REG_CURRENT = 0x04,
        REG_CALIB = 0x05,
        REG_ID = 0xFE,
    };

    enum avg_num_t : uint8_t {
        NUM_AVG_1 = 0b000,
        NUM_AVG_4 = 0b001,
        NUM_AVG_16 = 0b010,
        NUM_AVG_64 = 0b011,
        NUM_AVG_128 = 0b100,
        NUM_AVG_256 = 0b101,
        NUM_AVG_512 = 0b110,
        NUM_AVG_1024 = 0b111
    };

    enum bus_conv_time_t : uint8_t {
        VBUSCT_140_US = 0b000,
        VBUSCT_204_US = 0b001,
        VBUSCT_332_US = 0b010,
        VBUSCT_588_US = 0b011,
        VBUSCT_1_1MS = 0b100,
        VBUSCT_2_116MS = 0b101,
        VBUSCT_4_156MS = 0b110,
        VBUSCT_8_244_MS = 0b111
    };

    enum shunt_conv_time_t : uint8_t {
        VSHCT_140_US = 0b000,
        VSHCT_204_US = 0b001,
        VSHCT_332_US = 0b010,
        VSHCT_588_US = 0b011,
        VSHCT_1_1MS = 0b100,
        VSHCT_2_116MS = 0b101,
        VSHCT_4_156MS = 0b110,
        VSHCT_8_244_MS = 0b111
    };

    enum mode_t : uint8_t {
        MODE_POWERDOWN = 0b000,
        MODE_SHUNT_TRIG = 0b001,
        MODE_BUS_TRIG = 0b010,
        MODE_BUS_SHUNT_TRIG = 0b011,
        MODE_SHUNT_CONT = 0b101,
        MODE_BUS_CONT = 0b110,
        MODE_BUS_SHUNT_CONT = 0b111
    };

    struct config_t {
        avg_num_t avg : 3;
        bus_conv_time_t vbusct : 3;
        shunt_conv_time_t vshct : 3;
        mode_t mode : 3;

        uint16_t get() const {
            return (0x04 << 12) | (avg << 9) | (vbusct << 6) | (vshct << 3) | mode;
        }
    };

    class INA226 : public I2Cdev {
        double current_LSB;

    public:
        INA226();
        INA226(uint8_t address);

        virtual bool init() override;
        void set_configs(const config_t& config = {NUM_AVG_64, VBUSCT_8_244_MS, VSHCT_140_US, MODE_BUS_CONT});
        void set_calibration(double max_current = 0.05, double shunt_resistor = 0.1);

        double get_bus_voltage();
        double get_shunt_voltage();
        double get_current();
        double get_power();

        inline double get_bus_voltage_mV() { return get_bus_voltage() * 1e3; }
        inline double get_shunt_voltage_mV() { return get_shunt_voltage() * 1e3; }
        inline double get_current_mA() { return get_current() * 1e3; }
        inline double get_power_mW() { return get_power() * 1e3; }

        inline double get_bus_voltage_uV() { return get_bus_voltage() * 1e6; }
        inline double get_shunt_voltage_uV() { return get_shunt_voltage() * 1e6; }
        inline double get_current_uA() { return get_current() * 1e6; }
        inline double get_power_uW() { return get_power() * 1e6; }
    };
} // namespace ina

#endif // INA226_H_