#pragma once

#include "Arduino.h"
#include <WiFi.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "esp_wifi.h"

#include "esp32s3/rom/rtc.h" // For reset reason

#define WIFI_SSID "Xenobot" //"RovingXenobot" //"Device-Northwestern" //"NUMSR" //  
#define WIFI_PW "Xenobotlab" // "RovingXenobot" //"" // "robotics!" // 
#define SERVER_IP "129.105.69.100" //"192.168.1.57" // "129.105.69.100" // "129.105.69.124" //
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
    int error0;
    int error1;
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
    float goal_distance;  // Distance to goal (meters), updated externally
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