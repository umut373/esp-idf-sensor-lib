#ifndef CALIBRATE_H_
#define CALIBRATE_H_

#include <bit>
#include <string>
#include "nvs_flash.h"

class Calibrate {
    nvs_handle_t nvs_handle;

    inline static bool nvs_initialized = false;
    inline static int nvs_count = 0;

public:
    Calibrate();
    virtual ~Calibrate();

    virtual void calibrate() = 0;

protected:
    std::string nvs_name;

    void open_storage(const std::string& nvs_name);

    template<typename T>
    bool get_nvs_data(T* buffer);

    template<typename T>
    void set_nvs_data(const T& data);
};

#endif // CALIBRATE_H_