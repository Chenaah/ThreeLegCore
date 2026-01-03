#include "tasks.hpp"
#include "capybarish_pubsub.h"
#include "motor_control_messages.hpp"

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
            Serial.println("Received switch command: ");
            Serial.print("Switch: "); Serial.println(cmd.switch_);
            received_data.calibrate = cmd.calibrate;
            received_data.restart = cmd.restart;
            received_data.timestamp = cmd.timestamp;

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

            uint64_t end_time = esp_timer_get_time();
            receive_dt = end_time - start_time;
        }

        void _connect_to_wifi() {
            Serial.println("Connecting to WiFi...");

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
            feedback.motor.error0 = motor_error;
            feedback.motor.error1 = motor_error2;
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
            
            // Goal distance (set externally)
            feedback.goal_distance = goal_distance;

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
            data_to_send.motor.error0 = feedback.motor.error0;
            data_to_send.motor.error1 = feedback.motor.error1;
            data_to_send.goal_distance = feedback.goal_distance;

            // Publish feedback
            feedbackPub->publish(feedback);
        }
        
        void run(void *pvParameters) {
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // Connect to WiFi
            _connect_to_wifi();
            
            // Setup pub/sub
            _setup_pubsub();

            while (true) {
                // Process incoming messages
                node->spinOnce();
                commandSub->spinOnce();

                // Delay here to help the motor perform the latest commands
                vTaskDelay(pdMS_TO_TICKS(DELAY_PERIOD));

                // Send data
                _send_data();
            }
        }
    }


}