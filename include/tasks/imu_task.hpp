#pragma once

#include "Arduino.h"

#include <IMUManager.hpp>
#include <vector>


namespace Task {

    // IMU data (kept as std::vector<float> for backward compatibility)
    extern std::vector<float> euler_imu;
    extern std::vector<float> quat_imu;   // x, y, z, w
    extern std::vector<float> acc_imu;    // x, y, z
    extern std::vector<float> ang_vel_imu; // x, y, z

    namespace IMUTask {
        void run(void *pvParameters);
    }
}