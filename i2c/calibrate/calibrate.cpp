#include "calibrate.h"

Calibrate::Calibrate() {
    if (!nvs_initialized) {
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
        nvs_initialized = true;
    }
}

Calibrate::~Calibrate() {
    nvs_close(nvs_handle);
    nvs_count--;

    if (!nvs_count) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_deinit());
        nvs_initialized = false;
    }
}

void Calibrate::open_storage(const std::string& nvs_name) {
    this->nvs_name = nvs_name;
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_open(nvs_name.c_str(), NVS_READWRITE, &this->nvs_handle));
    nvs_count++;
}