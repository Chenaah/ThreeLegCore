#pragma once

#include "Arduino.h"


#include <Wire.h>
// #include <Adafruit_Sensor.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
// #include <utility/imumaths.h>
// #include "SPI.h"
#include <vector>


namespace Task {

    // IMU data
    // extern sensors_event_t orientationData, linearAccelData, gyroData;
    // extern imu::Quaternion quat;
    extern std::vector<float> euler_imu;
    extern std::vector<float> quat_imu;
    extern std::vector<float> acc_imu;
    extern std::vector<float> ang_vel_imu;

    namespace IMUTask {
        void run(void *pvParameters);
    }
}