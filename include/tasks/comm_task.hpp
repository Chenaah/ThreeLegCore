#pragma once

#include "Arduino.h"
#include <WiFi.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "esp_wifi.h"

#include "esp32s3/rom/rtc.h" // For reset reason
#include "local_obs_config.h"

#define WIFI_SSID "Xenobot" //"Device-Northwestern" //"RovingXenobot" //"NUMSR" //  
#define WIFI_PW "Xenobotlab" // "" //"RovingXenobot" //"" // "robotics!" // 
#define SERVER_IP "129.105.69.100" //"129.105.73.226" // "129.105.73.204" // "129.105.73.189" //"129.105.73.235" //"129.105.73.251" //"192.168.1.57" //  "129.105.69.124" //
#define SERVER_PORT 6666
#define CLIENT_PORT 6666

struct MotorCommand {
    float target;
    float target_vel;
    float kp;
    float kd;
    int enable_filter;
    int switch_;
    int calibrate;
    int restart;
    float timestamp;
    int control_mode;     // 0=direct PD target from PC, 1=ESP32 onboard model
    float joint_offset;   // Per-joint default offset (radians), sent explicitly by PC
    int policy_hash;      // Positive int32 FNV-1a hash of the expected onboard-model weights
    int joint_id;         // Action/joint index this command targets (0-based); -1 = all
    float command_context[LOCAL_TRANSPORT_COMMAND_CONTEXT_DIM];  // Auxiliary command context; all zeros when unused
};

// Legacy alias for compatibility
using ReceivedData = MotorCommand;

struct MotorData {
    float pos;
    float large_pos;
    float vel;
    float torque;
    float voltage;
    float current;
    int temperature;
    int motor_error;    // Motor error flags (6 bits)
    int motor_mode;     // Motor mode (0=Reset/Off, 1=Calibration, 2=Active/On)
    int driver_error;   // Driver chip error/fault state
};

struct IMUOrientation{float x; float y; float z;};
struct IMUQuaternion{float x; float y; float z; float w;};
struct IMUOmega{float x; float y; float z;};
struct IMUAcceleration{float x; float y; float z;};

struct IMUData {
    IMUOrientation orientation;
    IMUQuaternion quaternion;
    IMUOmega omega;
    IMUAcceleration acceleration;
};

struct ErrorData{
    int reset_reason0;
    int reset_reason1;
};

struct UWBDistances {
    float d0;  // Distance to anchor 0 (meters)
    float d1;  // Distance to anchor 1 (meters)
    float d2;  // Distance to anchor 2 (meters)
    float d3;  // Distance to anchor 3 (meters)
};

struct PolicyDebugData {
    int valid;             // 1 when debug data is populated
    int seq;               // Monotonic onboard-model tick counter
    float nn_action;       // Raw onboard-model action in [-0.8, 0.8]
    float motor_target;    // Base motor target before interpolation/filter
    float interp_target;   // Interpolated target at the current PD tick
    float applied_target;  // Final target after optional low-pass filtering
    float joint_offset;    // Joint offset applied on ESP32
    float dof_pos;         // Filtered joint position used by the onboard model
    float dof_vel;         // Filtered joint velocity used by the onboard model
    float command_context[LOCAL_TRANSPORT_COMMAND_CONTEXT_DIM];  // Latest command context used by the onboard model
    float local_obs[LOCAL_TRANSPORT_DEBUG_OBS_DIM];   // Debug view; may be truncated if LOCAL_OBS_DIM is larger
};

struct SensorData {
    int module_id;
    int receive_dt;
    int timestamp;
    int switch_off;
    float last_rcv_timestamp;
    int info;
    MotorData motor;
    IMUData imu;
    ErrorData error;
    int policy_hash;
    int policy_status;
    int policy_error;
    float goal_distance;  // Distance to goal (meters), updated externally
    UWBDistances uwb;     // UWB distance measurements
    PolicyDebugData policy_debug;
};

// Legacy alias for compatibility
using SentData = SensorData;


namespace Task {


    namespace CommTask {
        extern bool connected;
        extern MotorCommand received_data;
        extern SensorData data_to_send;
        extern float last_rcv_timestamp;
        extern uint64_t receive_dt;
        extern float goal_distance;  // Goal distance to be set externally

        void run(void *pvParameters);
    }
}
