#include "tasks.hpp"


namespace Task {

    namespace CommTask {

        ReceivedData received_data;
        SentData data_to_send;

        bool connected = false;
        struct sockaddr_in server_addr, client_addr;
        int sockfd;

        float last_rcv_timestamp = 0;
        uint64_t receive_dt = 0;

        int reset_reason0 = rtc_get_reset_reason(0);
        int reset_reason1 = rtc_get_reset_reason(1);

        void _connect_to_wifi() {
            char SSID[] = WIFI_SSID;
            char PW[] = WIFI_PW;

            Serial.println("Connecting to WiFi...");

            // init WiFi and wait till it's fully connected
            WiFi.setSleep(false);
            WiFi.disconnect(true);
            WiFi.begin(SSID, PW);
            connected = true;
            int i_connect = 0;
            while (WiFi.status() != WL_CONNECTED)
            {
                // esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(200));
                if (i_connect % 2 == 0)
                    send_led_message(LED_MSG_WIFI);
                else
                    set_led_color(255, 255, 255);
                i_connect ++;
                Serial.println("Connecting to WiFi...");
            }
            send_led_message(LED_MSG_NORMAL);

            Serial.print("WiFi initialized! WiFi connected! IP address: ");
            Serial.println(WiFi.localIP());

            Serial.print("ESP MAC Address:  ");
            Serial.println(WiFi.macAddress());

            // delay(10000);

            // Configure WiFi channel bandwidth
            // esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20); // Set to 20MHz bandwidth for lowest latency
            // Set WiFi transmit power
            // WiFi.setTxPower(WIFI_POWER_11dBm); // Set transmit power to an appropriate level
            // Enable fast WiFi reconnection
            WiFi.setAutoReconnect(true);
                    

        }

        void _setup_socket() {

            // Create socket
            if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
                Serial.println("Unable to create socket!");
            } else {
                Serial.println("Socket created!");
            }

            // Set the socket to non-blocking mode
            int flags = fcntl(sockfd, F_GETFL, 0);
            if (flags < 0) {
                Serial.println("fcntl F_GETFL failed");
            }
            flags = fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
            if (flags < 0) {
                Serial.println("fcntl F_SETFL failed");
            }

            // Fill client address structure
            memset(&client_addr, 0, sizeof(client_addr));
            client_addr.sin_family = AF_INET;
            client_addr.sin_addr.s_addr = INADDR_ANY;
            client_addr.sin_port = htons(CLIENT_PORT);

            // Bind the socket to the client address
            if (bind(sockfd, (const struct sockaddr *)&client_addr, sizeof(client_addr)) < 0) {
                Serial.println("Unable to bind socket!");
                close(sockfd);
            } else {
                Serial.println("Socket bound!");
            }

            // Fill server address structure
            memset(&server_addr, 0, sizeof(server_addr));
            server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
            server_addr.sin_family = AF_INET;
            server_addr.sin_port = htons(SERVER_PORT);

            // Set timeout
            struct timeval timeout;
            timeout.tv_sec = 1; // 10000;
            timeout.tv_usec = 0;
            if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0){
                Serial.println("Error setting receive timeout!");
                close(sockfd);
            }

            Serial.print("Server IP address : ");
            Serial.print(SERVER_IP);
            Serial.print(", Port : ");
            Serial.println(SERVER_PORT);
        }

        bool _send(uint8_t *const content, const uint32_t len){
            if (connected){
                int error = sendto(sockfd, content, len, 0, (struct sockaddr *)&server_addr, sizeof(server_addr));
                if (error < 0){
                    DEBUG_PRINT("Error occurred during sending: ");
                    DEBUG_PRINT(error);
                    return 0;
                } else {
                    return 1;
                }
            } else {
                DEBUG_PRINT("WiFi not connected!");
            }
            return 0;
        }

        int _receive(uint8_t *const content, const uint32_t max_len){
            if (connected){
                struct sockaddr_storage temp_addr;
                socklen_t temp_len = sizeof(temp_addr);

                int len = recvfrom(sockfd, content, max_len, 0, (struct sockaddr *)&temp_addr, &temp_len);
                return len;
            } else {
                DEBUG_PRINT("WiFi not connected!");
            }

            return -1;
        }

        void _receive_data() {
            uint64_t start_time = esp_timer_get_time();
            int n = _receive(reinterpret_cast<uint8_t*>(&received_data), sizeof(ReceivedData));
            if (n >= 0) {
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
            
        }

        void _send_data() {
            // Serial.println("Sending data...");
            uint64_t timestamp = esp_timer_get_time();

            data_to_send.module_id = module_id;
            data_to_send.receive_dt = receive_dt;
            data_to_send.timestamp = timestamp;
            data_to_send.switch_off = switch_off_request;
            data_to_send.last_rcv_timestamp = last_rcv_timestamp;
            data_to_send.info = dequeue(info_queue);
            // Motor data (apply wrap_offset to report wrapped position)
            data_to_send.motor.pos = (st.angle + MotorTask::wrap_offset - offset);
            data_to_send.motor.large_pos = (large_motor_pos + MotorTask::wrap_offset - offset);
            data_to_send.motor.vel = st.angle_v;
            data_to_send.motor.torque = st.torque;
            data_to_send.motor.voltage = voltage;
            data_to_send.motor.current = current;
            data_to_send.motor.temperature = st.temperature;
            data_to_send.motor.error0 = motor_error; // mode & error
            data_to_send.motor.error1 = motor_error2;
            // IMU data
            data_to_send.imu.orientation.x = euler_imu[0];
            data_to_send.imu.orientation.y = euler_imu[1];
            data_to_send.imu.orientation.z = euler_imu[2];
            data_to_send.imu.quaternion.x = quat_imu[0];
            data_to_send.imu.quaternion.y = quat_imu[1];
            data_to_send.imu.quaternion.z = quat_imu[2];
            data_to_send.imu.quaternion.w = quat_imu[3];
            data_to_send.imu.omega.x = ang_vel_imu[0];
            data_to_send.imu.omega.y = ang_vel_imu[1];
            data_to_send.imu.omega.z = ang_vel_imu[2];
            data_to_send.imu.acceleration.x = acc_imu[0];
            data_to_send.imu.acceleration.y = acc_imu[1];
            data_to_send.imu.acceleration.z = acc_imu[2];
            // Error data
            data_to_send.error.reset_reason0 = reset_reason0;
            data_to_send.error.reset_reason1 = reset_reason1;

            uint8_t buffer[sizeof(SentData)];
            memcpy(buffer, &data_to_send, sizeof(SentData));

            _send(buffer, sizeof(SentData));
        }
        
        void run(void *pvParameters) {
            // esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(100));
            
            _connect_to_wifi();
            _setup_socket();

            while (true) {

                // Check if WiFi is connected
                // if (WiFi.status() != WL_CONNECTED){
                //     enqueue(info_queue, 100);
                //     connected = false;
                //     _connect_to_wifi();
                // }

                // Receive data
                _receive_data();

                // Delay here to help the motor performs the lastestest commands
                // esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(DELAY_PERIOD));

                // Send data
                _send_data();

                // Serial.printf("Data sent:\n");
                // Serial.printf("st.angle: %d\n", st.angle);
                // Serial.printf("Motor Position: %d\n", data_to_send.motor.pos);

                

            }
        }
    }


}