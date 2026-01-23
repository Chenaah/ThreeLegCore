#pragma once

#include "Arduino.h"
#include <IMUManager.hpp>
#include <vector>

namespace Task {

    // IMU data (backward compatible exports)
    extern std::vector<float> euler_imu;
    extern std::vector<float> quat_imu;  // x, y, z, w
    extern std::vector<float> acc_imu;   // x, y, z
    extern std::vector<float> ang_vel_imu; // x, y, z

    namespace IMUTask {
        // Initialize IMU - call this before starting the task
        bool initialize(uint8_t cs_pin, uint8_t int_pin, uint8_t rst_pin);
        
        void run(void *pvParameters);
    }
}