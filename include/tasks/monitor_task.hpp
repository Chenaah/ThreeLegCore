#pragma once

#include "Arduino.h"


#include <Wire.h>
#include <protocentral_TLA20xx.h>

#define TLA20XX_I2C_ADDR 0x48


#define ADC_CHANNEL_VOLTAGE 0
#define ADC_CHANNEL_CURRENT 1
#define ADC_CHANNEL_HALL 2

namespace Task {

    
    extern float monitored_value;
    //  = tla2024.read_adc();

    namespace MonitorTask {
        void set_channel(uint8_t channel);
        void run(void *pvParameters);
    }
}