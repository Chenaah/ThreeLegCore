#include "tasks.hpp"
#include <ArduinoOTA.h>
#include <WiFi.h>

namespace Task {
    namespace OTATask {
        volatile bool ota_in_progress = false;

        namespace {
            bool ota_initialized = false;
            char ota_hostname[32] = {0};

            void build_ota_hostname() {
                uint64_t chip_id = ESP.getEfuseMac();
                snprintf(
                    ota_hostname,
                    sizeof(ota_hostname),
                    "capy-%llu-%04llX",
                    static_cast<unsigned long long>(module_id),
                    static_cast<unsigned long long>(chip_id & 0xFFFFULL)
                );
            }

            void begin_ota() {
                if (ota_initialized) {
                    return;
                }

                build_ota_hostname();
                ArduinoOTA.setHostname(ota_hostname);
                ArduinoOTA.setTimeout(30000);

                ArduinoOTA.onStart([]() {
                    ota_in_progress = true;
                });

                ArduinoOTA.onEnd([]() {
                    ota_in_progress = false;
                });

                ArduinoOTA.onError([](ota_error_t error) {
                    (void)error;
                    ota_in_progress = false;
                });

                ArduinoOTA.begin();
                ota_initialized = true;

                Serial.printf("[OTA] Ready. Hostname: %s.local\n", ota_hostname);
                Serial.printf("[OTA] Ready. IP: %s\n", WiFi.localIP().toString().c_str());
            }
        }

        void run(void *pvParameters) {
            (void)pvParameters;

            while (true) {
                if (!ota_initialized) {
                    if (!CommTask::connected || WiFi.status() != WL_CONNECTED) {
                        vTaskDelay(pdMS_TO_TICKS(500));
                        continue;
                    }
                    begin_ota();
                }

                ArduinoOTA.handle();
                vTaskDelay(pdMS_TO_TICKS(ota_in_progress ? 1 : 10));
            }
        }
    }
}
