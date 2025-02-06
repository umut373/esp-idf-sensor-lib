#ifndef GPS_H_
#define GPS_H_

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define PMTK_API_SET_FIX_CTL_5HZ        "$PMTK300,200,0,0,0,0*2F\r\n"
#define PMTK_SET_NMEA_UPDATE_2HZ        "$PMTK220,500*2B\r\n"
#define PMTK_SET_NMEA_OUTPUT_GGAONLY    "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"

#define UART_NUM UART_NUM_1
#define TXD_PIN GPIO_NUM_4
#define RXD_PIN GPIO_NUM_5

#define PATTERN         ('\n')
#define PATTERN_CHR_NUM (1)

#define BUF_SIZE (1024)

namespace gps {
    class GPS {
        bool fix;
        double latitude;
        double longitude;
        double altitude;

    public:
        GPS();
        GPS(uart_port_t uart_port, gpio_num_t txd_pin, gpio_num_t rxd_pin);
        ~GPS();

        void init();
        void begin();

        bool get_fix();
        double get_lat_deg();
        double get_long_deg();
        double get_alt();

    private:
        bool check_NMEA_data(uint8_t* buffer, size_t size);
        void parse_NMEA_data(uint8_t* buffer);

        static void uart_event_task_entry_point(void* obj);
        void uart_event_task(uart_event_t event, uint8_t* data);
    };
} // namespace GPS



#endif // GPS_H_