#include "tasks.hpp"
#include "capybarish_pubsub.h"
#include "motor_control_messages.hpp"
#include "LocalPolicy.hpp"
#include "deploy_config.h"
#include <cstring>

// Forward declaration of the global onboard-model instance defined in main.cpp
extern LocalPolicy onboard_model;

// Use type aliases for motor_control namespace types
using CapySensorData = motor_control::SensorData;
using CapyMotorCommand = motor_control::MotorCommand;

namespace Task {

    namespace CommTask {

        // Pub/Sub components
        cpy::Node* node = nullptr;
        cpy::Publisher<CapySensorData>* feedbackPub = nullptr;
        cpy::Subscription<CapyMotorCommand>* commandSub = nullptr;

        MotorCommand received_data;
        SensorData data_to_send;
        float goal_distance = 0.233f;  // Goal distance to be set externally

        bool connected = false;

        float last_rcv_timestamp = 0;
        uint64_t receive_dt = 0;

        int reset_reason0 = rtc_get_reset_reason(0);
        int reset_reason1 = rtc_get_reset_reason(1);

        // Callback for received commands
        void _on_command_received(const CapyMotorCommand& cmd) {
            uint64_t start_time = esp_timer_get_time();

            // Copy to local struct for compatibility
            received_data.target = cmd.target;
            received_data.target_vel = cmd.target_vel;
            received_data.kp = cmd.kp;
            received_data.kd = cmd.kd;
            received_data.enable_filter = cmd.enable_filter;
            received_data.switch_ = cmd.switch_;
            received_data.calibrate = cmd.calibrate;
            received_data.restart = cmd.restart;
            received_data.timestamp = cmd.timestamp;
            received_data.control_mode = cmd.control_mode;
            received_data.joint_offset = cmd.joint_offset;
            received_data.policy_hash = cmd.policy_hash;
            received_data.joint_id = cmd.joint_id;
            memcpy(received_data.command_context, cmd.command_context, sizeof(cmd.command_context));

            // Update shared variables
            target_pos = received_data.target;
            target_vel = received_data.target_vel;
            command_kp = received_data.kp;
            command_kd = received_data.kd;
            enable_filter = received_data.enable_filter;
            remote_switch = received_data.switch_;
            calibrate_command = received_data.calibrate;
            restart_command = received_data.restart;
            last_rcv_timestamp = received_data.timestamp;
            received_control_mode = received_data.control_mode;
            received_joint_offset = received_data.joint_offset;
            received_policy_hash = received_data.policy_hash;
            received_joint_id = received_data.joint_id;
            memset(received_command_context, 0, sizeof(received_command_context));
            for (size_t i = 0; i < Task::COMMAND_CONTEXT_DIM; ++i) {
                received_command_context[i] = received_data.command_context[i];
            }

            // Push new waypoint into the interpolator for smooth 100 Hz control
            cmd_interpolator.pushCommand(
                received_data.target,
                received_data.target_vel,
                received_data.kp,
                received_data.kd
            );

            uint64_t end_time = esp_timer_get_time();
            receive_dt = end_time - start_time;
        }

        void _connect_to_wifi() {
            Serial.println("Connecting to WiFi...");
            Serial.print("ESP MAC Address: ");
            Serial.println(WiFi.macAddress());

            WiFi.setSleep(false);
            WiFi.disconnect(true);
            WiFi.begin(WIFI_SSID, WIFI_PW);
            
            int i_connect = 0;
            while (WiFi.status() != WL_CONNECTED) {
                vTaskDelay(pdMS_TO_TICKS(200));
                if (i_connect % 2 == 0)
                    send_led_message(LED_MSG_WIFI);
                else
                    set_led_color(255, 255, 255);
                i_connect++;
                Serial.println("Connecting to WiFi...");
            }

            connected = true;
            send_led_message(LED_MSG_NORMAL);
            WiFi.setAutoReconnect(true);

            Serial.print("WiFi initialized! IP address: ");
            Serial.println(WiFi.localIP());
            Serial.print("ESP MAC Address: ");
            Serial.println(WiFi.macAddress());
        }

        void _setup_pubsub() {
            // Create node
            node = new cpy::Node("motor_module");

            // Create publisher - sends feedback to server
            feedbackPub = node->createPublisher<CapySensorData>(
                "/motor_feedback",
                SERVER_IP,
                SERVER_PORT
            );

            // Create subscriber - listens for commands on a DIFFERENT port
            // Note: Must use a different port than SERVER_PORT for receiving
            const uint16_t COMMAND_PORT = 6667;
            commandSub = node->createSubscription<CapyMotorCommand>(
                "/motor_cmd",
                _on_command_received,
                COMMAND_PORT
            );

            Serial.println("[Node] Pub/Sub ready!");
            Serial.printf("[Node] Sending feedback to %s:%d\n", SERVER_IP, SERVER_PORT);
            Serial.printf("[Node] Listening for commands on port %d\n", COMMAND_PORT);
        }

        void _send_data() {
            uint64_t timestamp = esp_timer_get_time();

            CapySensorData feedback;
            
            feedback.module_id = module_id;
            feedback.receive_dt = receive_dt;
            feedback.timestamp = timestamp;
            feedback.switch_off = switch_off_request;
            feedback.last_rcv_timestamp = last_rcv_timestamp;
            feedback.info = dequeue(info_queue);
            // Motor data (apply wrap_offset to report wrapped position)
            feedback.motor.pos = (st.angle + 0 - offset);
            feedback.motor.large_pos = (large_motor_pos + 0 - offset);
            feedback.motor.vel = st.angle_v;
            feedback.motor.torque = st.torque;
            feedback.motor.voltage = voltage;
            feedback.motor.current = current;
            feedback.motor.temperature = st.temperature;
            // Motor error: 6 bits of error flags from st.error_state
            feedback.motor.motor_error = st.error_state & 0x3F;
            // Motor mode: 0=Reset/Off, 1=Calibration, 2=Active/On
            // If motor is not calibrated, force mode to 1 (Calibration)
            feedback.motor.motor_mode = motor_calibrated ? ((st.error_state >> 6) & 0x03) : 1;
            // feedback.motor.motor_mode = ((st.error_state >> 6) & 0x03);
            // Driver error: packed fault state from driver chip
            feedback.motor.driver_error = motor_error2;
            // IMU data
            feedback.imu.orientation.x = euler_imu[0];
            feedback.imu.orientation.y = euler_imu[1];
            feedback.imu.orientation.z = euler_imu[2];
            feedback.imu.quaternion.x = quat_imu[0];
            feedback.imu.quaternion.y = quat_imu[1];
            feedback.imu.quaternion.z = quat_imu[2];
            feedback.imu.quaternion.w = quat_imu[3];
            feedback.imu.omega.x = ang_vel_imu[0];
            feedback.imu.omega.y = ang_vel_imu[1];
            feedback.imu.omega.z = ang_vel_imu[2];
            feedback.imu.acceleration.x = acc_imu[0];
            feedback.imu.acceleration.y = acc_imu[1];
            feedback.imu.acceleration.z = acc_imu[2];
            // Error data
            feedback.error.reset_reason0 = reset_reason0;
            feedback.error.reset_reason1 = reset_reason1;
            feedback.policy_hash = ::onboard_model.get_policy_hash();
            feedback.policy_status = policy_status_bits;
            feedback.policy_error = policy_error_code;
            feedback.policy_debug.valid = policy_debug_valid;
            feedback.policy_debug.seq = policy_debug_seq;
            feedback.policy_debug.nn_action = policy_debug_nn_action;
            feedback.policy_debug.motor_target = policy_debug_motor_target;
            feedback.policy_debug.joint_offset = policy_debug_joint_offset;
            feedback.policy_debug.dof_pos = policy_debug_dof_pos;
            feedback.policy_debug.dof_vel = policy_debug_dof_vel;
            memset(
                feedback.policy_debug.command_context,
                0,
                sizeof(feedback.policy_debug.command_context)
            );
            for (size_t i = 0; i < LOCAL_DEBUG_COMMAND_CONTEXT_DIM; ++i) {
                feedback.policy_debug.command_context[i] = policy_debug_command_context[i];
            }
            memset(feedback.policy_debug.local_obs, 0, sizeof(feedback.policy_debug.local_obs));
            for (size_t i = 0; i < LOCAL_DEBUG_OBS_DIM; ++i) {
                feedback.policy_debug.local_obs[i] = policy_debug_local_obs[i];
            }
            
            // Goal distance (set externally)
            feedback.goal_distance = goal_distance;
            
            // UWB distances (placeholder - will be updated with actual UWB data)
            feedback.uwb.d0 = 0.0f;
            feedback.uwb.d1 = 0.0f;
            feedback.uwb.d2 = 0.0f;
            feedback.uwb.d3 = 0.0f;

            // Copy to local struct for compatibility
            data_to_send.module_id = feedback.module_id;
            data_to_send.receive_dt = feedback.receive_dt;
            data_to_send.timestamp = feedback.timestamp;
            data_to_send.switch_off = feedback.switch_off;
            data_to_send.last_rcv_timestamp = feedback.last_rcv_timestamp;
            data_to_send.info = feedback.info;
            data_to_send.motor.pos = feedback.motor.pos;
            data_to_send.motor.large_pos = feedback.motor.large_pos;
            data_to_send.motor.vel = feedback.motor.vel;
            data_to_send.motor.torque = feedback.motor.torque;
            data_to_send.motor.voltage = feedback.motor.voltage;
            data_to_send.motor.current = feedback.motor.current;
            data_to_send.motor.temperature = feedback.motor.temperature;
            data_to_send.motor.motor_error = feedback.motor.motor_error;
            data_to_send.motor.motor_mode = feedback.motor.motor_mode;
            data_to_send.motor.driver_error = feedback.motor.driver_error;
            data_to_send.imu.orientation.x = feedback.imu.orientation.x;
            data_to_send.imu.orientation.y = feedback.imu.orientation.y;
            data_to_send.imu.orientation.z = feedback.imu.orientation.z;
            data_to_send.imu.quaternion.x = feedback.imu.quaternion.x;
            data_to_send.imu.quaternion.y = feedback.imu.quaternion.y;
            data_to_send.imu.quaternion.z = feedback.imu.quaternion.z;
            data_to_send.imu.quaternion.w = feedback.imu.quaternion.w;
            data_to_send.imu.omega.x = feedback.imu.omega.x;
            data_to_send.imu.omega.y = feedback.imu.omega.y;
            data_to_send.imu.omega.z = feedback.imu.omega.z;
            data_to_send.imu.acceleration.x = feedback.imu.acceleration.x;
            data_to_send.imu.acceleration.y = feedback.imu.acceleration.y;
            data_to_send.imu.acceleration.z = feedback.imu.acceleration.z;
            data_to_send.policy_hash = feedback.policy_hash;
            data_to_send.policy_status = feedback.policy_status;
            data_to_send.policy_error = feedback.policy_error;
            data_to_send.goal_distance = feedback.goal_distance;
            data_to_send.uwb.d0 = feedback.uwb.d0;
            data_to_send.uwb.d1 = feedback.uwb.d1;
            data_to_send.uwb.d2 = feedback.uwb.d2;
            data_to_send.uwb.d3 = feedback.uwb.d3;
            data_to_send.policy_debug.valid = feedback.policy_debug.valid;
            data_to_send.policy_debug.seq = feedback.policy_debug.seq;
            data_to_send.policy_debug.nn_action = feedback.policy_debug.nn_action;
            data_to_send.policy_debug.motor_target = feedback.policy_debug.motor_target;
            data_to_send.policy_debug.joint_offset = feedback.policy_debug.joint_offset;
            data_to_send.policy_debug.dof_pos = feedback.policy_debug.dof_pos;
            data_to_send.policy_debug.dof_vel = feedback.policy_debug.dof_vel;
            memcpy(data_to_send.policy_debug.command_context,
                   feedback.policy_debug.command_context,
                   sizeof(feedback.policy_debug.command_context));
            memcpy(data_to_send.policy_debug.local_obs,
                   feedback.policy_debug.local_obs,
                   sizeof(feedback.policy_debug.local_obs));

            // Publish feedback
            feedbackPub->publish(feedback);
        }
        
        void run(void *pvParameters) {
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // Connect to WiFi
            _connect_to_wifi();
            
            // Setup pub/sub
            _setup_pubsub();

            uint32_t last_feedback_ms = millis();

            while (true) {
                if (OTATask::ota_in_progress) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                    continue;
                }

                // Process incoming messages
                node->spinOnce();
                commandSub->spinOnce();

                // Delay here to help the motor perform the latest commands
                vTaskDelay(pdMS_TO_TICKS(DELAY_PERIOD));

                uint32_t now_ms = millis();
                if ((now_ms - last_feedback_ms) >= FEEDBACK_PERIOD_MS) {
                    last_feedback_ms = now_ms;
                    _send_data();
                }
            }
        }
    }


}
