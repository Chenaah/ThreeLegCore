#include "tasks.hpp"
#include <DW1000Responder.hpp>

namespace Task {

    // Global UWB data
    float uwb_distance = -1.0f;
    uint64_t uwb_timestamp_us = 0;
    bool uwb_initialized = false;

    namespace UWBTask {

        // Queue handle for receiving ranging results
        static QueueHandle_t ranging_queue = nullptr;

        bool initialize(uint8_t cs_pin, uint8_t irq_pin, uint8_t rst_pin) {
            Serial.println("[UWB] Initializing as Responder...");

            // Initialize as responder with higher callback priority for timing sensitivity
            // Parameters: CS, IRQ, RST, callback_priority
            ranging_queue = UWBRanging::Responder::Initialize(cs_pin, irq_pin, rst_pin, 7);

            if (ranging_queue == nullptr) {
                Serial.println("[UWB] ERROR: Failed to initialize Responder!");
                return false;
            }

            Serial.println("[UWB] Responder initialized successfully!");
            uwb_initialized = true;
            return true;
        }

        void run(void *pvParameters) {
            // Wait for system to stabilize
            vTaskDelay(pdMS_TO_TICKS(500));

            // Check initialization - Begin() is called in main.cpp after IMU init
            if (!uwb_initialized) {
                Serial.println("[UWB] ERROR: Cannot start - not initialized!");
                vTaskDelete(NULL);
                return;
            }
            
            Serial.println("[UWB] Task started, listening for ranging results...");

            // Statistics tracking
            uint32_t measurement_count = 0;
            uint32_t last_stats_time = millis();
            float min_distance = 999.0f;
            float max_distance = 0.0f;
            float sum_distance = 0.0f;

            while (true) {
                // Check for new ranging results (non-blocking with short timeout)
                UWBRanging::RangingResult result;
                if (xQueueReceive(ranging_queue, &result, pdMS_TO_TICKS(10)) == pdTRUE) {
                    // Valid measurement received
                    uwb_distance = result.distance_m;
                    uwb_timestamp_us = result.measurement_time_us;
                    
                    // Update statistics
                    measurement_count++;
                    sum_distance += result.distance_m;
                    if (result.distance_m < min_distance) min_distance = result.distance_m;
                    if (result.distance_m > max_distance) max_distance = result.distance_m;

                    // Print each measurement
                    Serial.printf("[UWB] Distance: %.3f m (timestamp: %llu us)\n", 
                                  result.distance_m, result.measurement_time_us);
                }

                // Print statistics every second
                uint32_t now = millis();
                if (now - last_stats_time >= 1000) {
                    if (measurement_count > 0) {
                        float avg_distance = sum_distance / measurement_count;
                        Serial.printf("[UWB] Stats: %u measurements/s, avg=%.3f m, min=%.3f m, max=%.3f m\n",
                                      measurement_count, avg_distance, min_distance, max_distance);
                    } else {
                        Serial.println("[UWB] No measurements received in last second");
                    }
                    
                    // Reset statistics
                    measurement_count = 0;
                    sum_distance = 0.0f;
                    min_distance = 999.0f;
                    max_distance = 0.0f;
                    last_stats_time = now;
                }

                // Small delay to prevent task starvation
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        float getDistance() {
            return uwb_distance;
        }
    }
}
