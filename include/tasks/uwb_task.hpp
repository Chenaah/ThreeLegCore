#pragma once

#include "Arduino.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace Task {

    // UWB data accessible from other tasks
    extern float uwb_distance;           // Latest distance in meters
    extern uint64_t uwb_timestamp_us;    // Timestamp of latest measurement
    extern bool uwb_initialized;         // Whether UWB is ready

    namespace UWBTask {
        /**
         * @brief Initialize UWB hardware as responder
         * @param cs_pin SPI chip select pin
         * @param irq_pin Interrupt pin
         * @param rst_pin Reset pin
         * @return true if initialization successful
         * @note SPI bus must be initialized before calling this
         */
        bool initialize(uint8_t cs_pin, uint8_t irq_pin, uint8_t rst_pin);

        /**
         * @brief FreeRTOS task function for UWB processing
         * @param pvParameters Task parameters (unused)
         */
        void run(void *pvParameters);

        /**
         * @brief Get the latest distance measurement
         * @return Distance in meters, or -1.0 if no valid reading
         */
        float getDistance();
    }
}
