#ifndef CALIBRATE_H_
#define CALIBRATE_H_

#include <bit>
#include "nvs_flash.h"

class Calibrate {
    inline static bool nvs_initialized = false;

protected:
    nvs_handle_t nvs_handle;
    inline static int nvs_count = 0;

public:
    Calibrate();
    virtual ~Calibrate();

    virtual void calibrate() = 0;
};

#endif // CALIBRATE_H_