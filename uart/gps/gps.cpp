#include "gps.h"
#include <iostream>
#include <vector>
#include <string>
#include "esp_log.h"

using namespace gps;

static const char *TAG = "uart_events";

GPS::GPS() {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {
            .allow_pd = false,
            .backup_before_sleep = false
        }
    };

    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 20, &this->uart_queue, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

GPS::GPS(uart_port_t uart_port, gpio_num_t txd_pin, gpio_num_t rxd_pin) {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {
            .allow_pd = false,
            .backup_before_sleep = false
        }
    };

    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 20, &this->uart_queue, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(uart_port, txd_pin, rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

GPS::~GPS() {
    stop();
    ESP_ERROR_CHECK(uart_driver_delete(UART_NUM));
}

void GPS::init() {
    uart_write_bytes(UART_NUM, PMTK_API_SET_FIX_CTL_5HZ, strlen(PMTK_API_SET_FIX_CTL_5HZ));
    uart_write_bytes(UART_NUM, PMTK_SET_NMEA_UPDATE_2HZ, strlen(PMTK_SET_NMEA_UPDATE_2HZ));
    uart_write_bytes(UART_NUM, PMTK_SET_NMEA_OUTPUT_GGAONLY, strlen(PMTK_SET_NMEA_OUTPUT_GGAONLY));

    uart_enable_pattern_det_baud_intr(UART_NUM, PATTERN, PATTERN_CHR_NUM, 9, 0, 0);
    uart_pattern_queue_reset(UART_NUM, 20);
}

void GPS::begin() {
    if (!this->data_mutex) {
        this->data_mutex = xSemaphoreCreateMutex();
    }

    if (!this->update_task_handle) {
        xTaskCreate(uart_event_task_entry_point, "GPS_UART_EVENT", 2048, this, 12, &this->update_task_handle);
    }
}

void GPS::stop() {
    if (this->data_mutex) {
        if(xSemaphoreGetMutexHolder(this->data_mutex)) 
            xSemaphoreGive(this->data_mutex);

        vSemaphoreDelete(this->data_mutex);
    }

    if (this->update_task_handle) {
        vTaskDelete(this->update_task_handle);
        this->update_task_handle = NULL;
    }
}

void GPS::uart_event_task_entry_point(void* obj) {
    GPS* gps_instance = static_cast<GPS*>(obj);
    uart_event_t event;
    uint8_t* data = new uint8_t[BUF_SIZE];

    for(;;) {
        gps_instance->uart_event_task(event, data);
    }
    delete[] data;
    vTaskDelete(NULL);
}

void GPS::uart_event_task(uart_event_t event, uint8_t* data) {
    if (xQueueReceive(this->uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
        bzero(data, BUF_SIZE);
        switch (event.type) {
            case UART_DATA:
                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(UART_NUM, data, event.size, portMAX_DELAY);
                ESP_LOGI(TAG, "[DATA EVT]: %s", data);
                break;
            case UART_FIFO_OVF:
                uart_flush_input(UART_NUM);
                xQueueReset(this->uart_queue);
                break;
            case UART_BUFFER_FULL:
                uart_flush_input(UART_NUM);
                xQueueReset(this->uart_queue);
                break;
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            case UART_DATA_BREAK:
                ESP_LOGI(TAG, "uart rx data break");
                break;
            case UART_PATTERN_DET: {
                int pos = uart_pattern_pop_pos(UART_NUM);
                if (pos == -1) {
                    uart_flush_input(UART_NUM);
                } else {
                    uint8_t* pattern = new uint8_t[PATTERN_CHR_NUM+1];
                    uart_read_bytes(UART_NUM, data, pos-1, pdMS_TO_TICKS(100));
                    uart_read_bytes(UART_NUM, pattern, PATTERN_CHR_NUM+1, pdMS_TO_TICKS(100));
                    delete[] pattern;

                    if (strncmp((char*)data, "$GPGGA", 6) == 0 && check_NMEA_data(data, pos-1)) {
                        parse_NMEA_data(data);
                    }
                }
                break;
            }
            case UART_EVENT_MAX:
                ESP_LOGI(TAG, "uart event max");
                break;

            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
        }
    }
}

bool GPS::get_fix() {
    static bool fix_val = false;

    if (xSemaphoreTake(this->data_mutex, 0) == pdTRUE) {
        fix_val = this->fix;
        xSemaphoreGive(this->data_mutex);
    }
    return fix_val;
}

double GPS::get_long_deg() {
    static double long_val = 0.0;

    if (xSemaphoreTake(this->data_mutex, 0) == pdTRUE) {
        int degrees = static_cast<int>(this->longitude) / 100;
        double minutes = this->longitude - (degrees * 100);
        xSemaphoreGive(this->data_mutex);

        long_val = degrees + (minutes / 60.0);
    }
    return long_val;
}

double GPS::get_lat_deg() {
    static double lat_val = 0.0;

    if (xSemaphoreTake(this->data_mutex, 0) == pdTRUE) {
        int degrees = static_cast<int>(this->latitude) / 100;
        double minutes = this->latitude - (degrees * 100);
        xSemaphoreGive(this->data_mutex);

        lat_val = degrees + (minutes / 60.0);
    }
    return lat_val;
}

double GPS::get_alt() {
    static double alt_val = 0.0;

    if (xSemaphoreTake(this->data_mutex, 0) == pdTRUE) {
        alt_val = this->altitude;
        xSemaphoreGive(this->data_mutex);
    }
    return alt_val;
}

bool GPS::check_NMEA_data(uint8_t* buffer, size_t size) {
    uint8_t checksum = 0;
    size_t checksum_index = 0;

    for (int i = 0; i < size; i++) {
        char c = buffer[i];

        switch(c) {
            case '$':
                break;
            case '*':
                checksum_index = i + 1;
                i = size;
                continue;
            default:
                checksum = checksum ^ c;
                break;
        }
    }
    char checksum_str[3];
    sprintf(checksum_str, "%02X", checksum);

    return strncmp(checksum_str, (char*)(buffer + checksum_index), 2) == 0;
}

void GPS::parse_NMEA_data(uint8_t* buffer) {
    std::string data = (char*)buffer;

    size_t pos_start = 0, pos_end;
    std::vector<std::string> parsed_data;

    while((pos_end = data.find(",", pos_start)) != std::string::npos) {
        parsed_data.push_back(data.substr(pos_start, pos_end - pos_start));
        pos_start = pos_end + 1; 
    }
    parsed_data.push_back(data.substr(pos_start));

    if (xSemaphoreTake(this->data_mutex, portMAX_DELAY) == pdTRUE) {
        if (parsed_data[6].empty()) {
            this->fix = false;
            this->latitude = 0.0;
            this->longitude = 0.0;
            this->altitude = 0.0;
        }
        else {
            this->fix = std::stoi(parsed_data[6]) > 0;
            if (this->fix) {
                this->latitude = std::stod(parsed_data[2]);
                this->longitude = std::stod(parsed_data[4]);
                this->altitude = std::stod(parsed_data[9]);
            }
            else {
                this->latitude = 0.0;
                this->longitude = 0.0;
                this->altitude = 0.0;
            }
        }

        xSemaphoreGive(this->data_mutex);
    }
}