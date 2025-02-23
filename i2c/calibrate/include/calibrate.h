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

    template <typename T>
    bool get_nvs_data(T* buffer) {
        size_t size = sizeof(T);
        return nvs_get_blob(this->nvs_handle, this->nvs_name.c_str(), buffer, &size) == ESP_OK;
    }

    template <typename T>
    void set_nvs_data(const T& data) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_blob(this->nvs_handle, this->nvs_name.c_str(), &data, sizeof(T)));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_commit(this->nvs_handle));
    }
};

#endif // CALIBRATE_H_