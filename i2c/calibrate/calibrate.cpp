#include "calibrate.h"

Calibrate::Calibrate() {
    if (!nvs_initialized) {
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
        nvs_initialized = true;
    }
}

Calibrate::~Calibrate() {
    nvs_close(nvs_handle);
    nvs_count--;

    if (!nvs_count) {
        ESP_ERROR_CHECK(nvs_flash_deinit());
        nvs_initialized = false;
    }
}

void Calibrate::open_storage(const std::string& nvs_name) {
    this->nvs_name = nvs_name;
    ESP_ERROR_CHECK(nvs_open(nvs_name.c_str(), NVS_READWRITE, &this->nvs_handle));
    nvs_count++;
}

template <typename T>
bool Calibrate::get_nvs_data(T* buffer) {
    size_t size = sizeof(T);
    return nvs_get_blob(this->nvs_handle, this->nvs_name.c_str(), buffer, &size) == ESP_OK;
}

template <typename T>
void Calibrate::set_nvs_data(const T& data) {
    ESP_ERROR_CHECK(nvs_set_blob(this->nvs_handle, this->nvs_name.c_str(), &data, sizeof(T)));
    ESP_ERROR_CHECK(nvs_commit(this->nvs_handle));
}