#pragma once

#include <vector>
#include <queue>
#include "Arduino.h"
#include <MotorCtrl.hpp>

#define MOTOR_ID 1


namespace Task {

    // Data
    extern Motor_state st;
    extern int remote_switch;
    extern int switch_off_request;
    extern int calibrate_command;
    extern int restart_command;
    extern float voltage;
    extern float current;
    extern uint32_t motor_error;
    extern uint32_t motor_error2;
    extern float large_motor_pos;
    extern std::queue<int> info_queue;
    extern bool motor_calibrated;  // Motor calibration status

    extern float target_pos;
    extern float target_vel;
    extern float command_kp;
    extern float command_kd;
    extern int enable_filter;

    // Config
    extern float offset;
    extern const float DELTA_T;

    namespace MotorTask {
        void run(void *pvParameters);
    }
}