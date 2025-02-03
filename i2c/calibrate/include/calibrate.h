#ifndef CALIBRATE_H_
#define CALIBRATE_H_

#include <bit>
#include "nvs_flash.h"

class Calibrate {
protected:
    nvs_handle_t nvs_handle;

public:
    virtual void calibrate() = 0;
};

#endif // CALIBRATE_H_