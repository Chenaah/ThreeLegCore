#include "tasks.hpp"
namespace Task {

    float monitored_value = 0;

    namespace MonitorTask {

        TLA20XX tla2024(TLA20XX_I2C_ADDR);

        void set_channel(uint8_t channel) {
            if (channel == ADC_CHANNEL_VOLTAGE) {
                tla2024.setMux(TLA20XX::MUX_AIN0_GND);
            } else if (channel == ADC_CHANNEL_CURRENT) {
                tla2024.setMux(TLA20XX::MUX_AIN1_GND);
            } else if (channel == ADC_CHANNEL_HALL) {
                tla2024.setMux(TLA20XX::MUX_AIN2_GND);
            } 
            delay(100);
        }

        void run(void *pvParameters) {

            Wire.setPins(SDA, SCL);
            Wire.begin();

            tla2024.begin();
            tla2024.setMode(TLA20XX::OP_CONTINUOUS);
            tla2024.setDR(TLA20XX::DR_3300SPS);
            tla2024.setFSR(TLA20XX::FSR_4_096V);

            // Set default channel as AIN0 <-> GND
            set_channel(ADC_CHANNEL_HALL);

            while (true) {
                // esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(2));
                monitored_value = tla2024.read_adc();
                // Serial.println(monitored_value);
            }


        }

    }
}