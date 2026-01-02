#include "tasks.hpp"


namespace Task {

    // Data
    Motor_state st;
    int remote_switch = 0;
    int switch_off_request = 0;
    int calibrate_command = 0;
    int restart_command = 0;
    float voltage = 0;
    float current = 0;
    uint32_t motor_error = 0;
    uint32_t motor_error2 = 0;
    float large_motor_pos = 0;
    float target_pos = 0;
    float target_vel = 0;
    float command_kp = 0;
    float command_kd = 0;
    int enable_filter = 1;
    std::queue<int> info_queue;

    // Config
    float offset = 0; //-1.0471975512; // 1.65806; // motor offset, shared for correcting sent observation
    const float DELTA_T = 100; // 20 (ms)

    namespace MotorTask {

        Motor motor;
        ButterworthFilter filter(20, 100);
        int motor_running = false;
        unsigned long epi_start_time;
        float wrap_offset = 0;  // K*2π offset to wrap motor position to (-π, π)

        // Calculate K*2π offset so that (angle + K*2π) is in (-π, π) range
        float _calculate_wrap_offset(float angle) {
            // Find K such that angle + K*2π is in (-π, π)
            // K = -floor((angle + π) / (2π))
            int k = -static_cast<int>(floor((angle + M_PI) / (2 * M_PI)));
            return k * 2 * M_PI;
        }

        bool _switch_on() {
            if (!digitalRead(MOTOR_TRIG_PIN) != motor_running) {
                if (motor_running) {
                    DEBUG_PRINT("motor.Disable()");
                    motor.Disable();
                } else {
                    DEBUG_PRINT("motor.Enable()");
                    motor.Enable();
                }
                motor_running = !motor_running;
            }
            return motor_running;
        }

        bool _safe() {
            bool safe = true;
            // if (!motor.calibrated){
            //     safe = false;
            //     enqueue(info_queue, 300);
            // }
            return safe;
        }

        void _disable(){
            DEBUG_PRINT("motor.Disable()");
            send_led_message(LED_MSG_MOTOR_OFF);
            enqueue(info_queue, 302);
            st = motor.Disable();
            motor_running = false;
        }

        void _move_to_zero_safely(float kp = 5.0, float kd = 0.3) {
            Serial.println("[Motor] Moving to zero position safely...");
            
            // Get current RAW motor position
            st = motor.Get_state();
            float rawStartPos = st.angle;
            
            // The target raw position when wrapped position = 0 is:
            // wrapped = raw + wrap_offset => raw = wrapped - wrap_offset
            // But we also have mechanical offset, so: raw_target = 0 - wrap_offset + offset
            float rawTargetPos = -wrap_offset + offset;
            
            float wrappedStartPos = rawStartPos + wrap_offset;  // For logging
            
            Serial.printf("[Motor] Raw start: %.3f rad (wrapped: %.3f), Raw target: %.3f rad\n", 
                          rawStartPos, wrappedStartPos, rawTargetPos);
            
            // Calculate distance in raw space
            float directDistance = rawTargetPos - rawStartPos;
            float totalDistance = abs(directDistance);
            
            Serial.printf("[Motor] Distance: %.3f rad (%.1f degrees), Direction: %s\n", 
                           totalDistance, totalDistance * 180.0 / PI,
                           directDistance > 0 ? "positive" : "negative");
            
            int steps = (int)(totalDistance / 0.02); // Move in ~0.02 rad increments
            steps = max(steps, 10); // Minimum 10 steps for smooth motion
            
            // Interpolate from current raw position to target raw position
            for (int i = 0; i <= steps; i++) {
                float t = (float)i / (float)steps; // 0.0 to 1.0
                float interpolatedRawPos = rawStartPos + directDistance * t;
                
                st = motor.Set_control(0, interpolatedRawPos, 0, kp, kd);
                
                // Print progress every 20 steps
                if (i % 20 == 0 || i == steps) {
                    st = motor.Get_state();
                    Serial.printf("[Motor] Progress: %d%%, Raw: %.3f rad, Wrapped: %.3f rad\n", 
                                 (i * 100) / steps, st.angle, st.angle + wrap_offset);
                }
                
                vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz control rate
                
                // Safety check: abort if motor encounters high torque (obstacle)
                st = motor.Get_state();
                if (abs(st.torque) > 15.0) {
                    Serial.printf("[Motor] WARNING: High torque detected (%.2f), aborting safe move!\n", st.torque);
                    return;
                }
            }
            
            Serial.println("[Motor] Reached zero position safely");
        }

        void _enable(){
            DEBUG_PRINT("motor.Enable()");
            send_led_message(LED_MSG_MOTOR_ON);
            enqueue(info_queue, 301);

            st = motor.Enable();
            motor_running = true;
            
            // Calculate wrap_offset to pretend motor starts from (-π, π)
            st = motor.Get_state();
            wrap_offset = _calculate_wrap_offset(st.angle);
            Serial.printf("[Motor] Enabled. Raw angle: %.3f rad, wrap_offset: %.3f rad (K=%d)\n", 
                          st.angle, wrap_offset, (int)(wrap_offset / (2 * M_PI)));
            
            // Move to zero position safely after enabling
            // vTaskDelay(pdMS_TO_TICKS(100)); // Wait for motor to stabilize
            _move_to_zero_safely();
            
            epi_start_time = millis();
        }

        bool _check_remote_switch() {
            bool enable = (bool)remote_switch;
            if (motor_running && ! enable)
                _disable();
            else if (!motor_running && enable)
                _enable();
            return motor_running;
        }

        Motor_state _manual_pd_control(float target_angle, float target_vel, float kp = 20, float kd = 0.5) {
            float pos_error = target_angle - large_motor_pos;
            float vel_error = target_vel - st.angle_v;
            float torque = kp * pos_error + kd * vel_error;
            st = motor.Set_control(torque, 0, 0, 0, 0);
            return st;
        }

        void _step(float target_angle, float target_vel, float kp = 20, float kd = 0.5) {
            if (_safe())
                // target_angle is in wrapped space, convert back to raw motor space
                st = motor.Set_control(0, target_angle - wrap_offset + offset, target_vel, kp, kd);
           
        }

        void _wait_for_switch_off() {
            while (remote_switch != 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                enqueue(info_queue, 303);
                st = motor.Get_state();
                send_led_message(LED_MSG_UNSAFE);
            }
        }

        void _wait_for_switch_on() {
            while (remote_switch == 0) {
                // esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(100));
                enqueue(info_queue, 304);
                st = motor.Get_state();
                send_led_message(LED_MSG_WAIT_CALI);

            }
        }

        void _wait_for_help() {
            _disable();
            while (1) {
                // esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(100));
                st = motor.Get_state();
                send_led_message(LED_MSG_WAIT_HELP);

            }
        }

        void _check_safety() {
            if (not _safe()){
                enqueue(info_queue, 305);
                _disable();
                switch_off_request = 1;
                enqueue(info_queue, 314);
                // esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(100));
                enqueue(info_queue, 315);
                // esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(100));
                // esp_restart();
                _wait_for_switch_off();
            }
        }

        void _move_to_middle(){
            enqueue(info_queue, 306);
            _enable();
            for (float i = 0; i <= PI/2; i += 0.01){
                vTaskDelay(pdMS_TO_TICKS(10));
                    // motor.Set_position(offset*sin(i));
                _step(-offset*cos(i), 0, 20, 0.5);
                DEBUG_PRINT("==> ");
                DEBUG_PRINT(st.angle);
            }
            enqueue(info_queue, 307);

        }

        void _init_motor(){
            motor.Init(MOTOR_ID, CAN_TX_PIN, CAN_RX_PIN);
        }

        void _manage_monitor(){
            unsigned long elapsed_time = millis() - epi_start_time;
            if (elapsed_time/1000. > 5){
                MonitorTask::set_channel(ADC_CHANNEL_CURRENT);
                current = monitored_value;
            } else {
                MonitorTask::set_channel(ADC_CHANNEL_VOLTAGE);
                voltage = monitored_value; // motor.get_voltage();

            }
        }

        // Function to spin the motor until the monitored value crosses a threshold
        float _spin_until_threshold(Motor &motor, float target_value, float speed, bool reverse = false) {
            float motor_angle_sum = 0.0F;
            float motor_temp_val = 0.0F;

            // Set motor control speed and direction
            st = motor.Set_control(0, 0, reverse ? -speed : speed, 0, 40);

            // Wait until monitored_value is below the target threshold
            while (monitored_value < target_value) {
                vTaskDelay(pdMS_TO_TICKS(1));
                motor_temp_val = motor.Get_state().angle;
            }
            motor_angle_sum += motor_temp_val;

            // Wait until monitored_value exceeds a secondary threshold for finer control
            while (monitored_value > target_value - 20) {
                vTaskDelay(pdMS_TO_TICKS(1));
                motor_temp_val = motor.Get_state().angle;
            }
            motor_angle_sum += motor_temp_val;

            return motor_angle_sum;
        }

        // Main function to align motor and set zero position
        void _align_motor_and_set_zero(Motor &motor, float hall_threshold, float speed = 0.5F) {
            float motor_total = 0.0F;

            // Spin motor in the forward direction
            motor_total += _spin_until_threshold(motor, hall_threshold, speed);

            // Pause before reversing
            vTaskDelay(pdMS_TO_TICKS(500));

            // Spin motor in the reverse direction
            motor_total += _spin_until_threshold(motor, hall_threshold, speed, true);

            // Average the total motor angle and set position
            st = motor.Set_control(0, motor_total / 4.0F, 0, 20, 0.5);
            vTaskDelay(pdMS_TO_TICKS(1000));

            // Disable motor and set zero position
            _disable();
            motor.Set_zero();
        }

        void _find_limit_and_set_zero(float torque_threshold=-2, float speed=-0.5F) {
            st = motor.Set_control(torque_threshold, 0, speed, 0, 2);
            vTaskDelay(pdMS_TO_TICKS(6000));
            _disable();
            st = motor.Set_zero();
        }

        void _auto_calibrate() {
            send_led_message(LED_MSG_CALI);
            st = motor.Get_state();
            MonitorTask::set_channel(ADC_CHANNEL_HALL);
            // TODO: Check the reading from the hall sensor is correct
            vTaskDelay(pdMS_TO_TICKS(100));
            _disable();
            _init_motor();
            st = motor.Set_zero();
            _enable();
            send_led_message(LED_MSG_CALI);

            enqueue(info_queue, 308);

            if (calibrate_command < 4){
                // Search for the magnetic marker
                _align_motor_and_set_zero(motor, 940, 0.5);
            } else if (calibrate_command == 4){
                // Move to the mechanical limit
                _find_limit_and_set_zero(5, 0.1);
                offset = 0;
            } else if (calibrate_command == 5){
                // Move to the mechanical limit
                _find_limit_and_set_zero(-5, -0.1);
                offset = 0;
            }

            motor.calibrated = true;
            MonitorTask::set_channel(ADC_CHANNEL_VOLTAGE);
            enqueue(info_queue, 309);
            // vTaskDelay(pdMS_TO_TICKS(100));
            _enable();
            if (offset != 0)
                _move_to_middle();
        }

        void _manual_calibrate(){
            enqueue(info_queue, 310);
            _disable();
            _init_motor();
            vTaskDelay(pdMS_TO_TICKS(100));
            st = motor.Set_zero();
            vTaskDelay(pdMS_TO_TICKS(100));
            motor.calibrated = true;
            enqueue(info_queue, 311);
            vTaskDelay(pdMS_TO_TICKS(100));
            _move_to_middle();

        }

        void _check_commands() {
            if (calibrate_command == 1){
                // Manual calibration
                enqueue(info_queue, 312);
                vTaskDelay(pdMS_TO_TICKS(100));
                // _auto_calibrate();
                _manual_calibrate();
                vTaskDelay(pdMS_TO_TICKS(100));
            } else if (calibrate_command == 2){
                // Auto calibration
                vTaskDelay(pdMS_TO_TICKS(100));
                _auto_calibrate();
                vTaskDelay(pdMS_TO_TICKS(100));
            } else if (calibrate_command == 3){
                // Set current position as zero position
                st = motor.Set_zero();
            }

            if (restart_command){
                esp_restart();
                // _wait_for_help();
            }
        }

        void _check_health() {
            if ((motor_error >> 6) & 0x03 == 0 && motor_running){
                send_led_message(LED_MSG_MOTOR_ERROR);
                Serial.println("[Motor] Motor error detected.");
            }

        }

        void run(void *pvParameters) {
            delay(2000); // Wait for the motor switch
            _init_motor();
            enqueue(info_queue, 313);

            motor.Set_mode(Motor_mode::Motion);
            // motor.Set_parameter(Motor_param::limit_spd, 1.0F);
            // motor.Set_parameter(Motor_param::imit_torque, 1.0F);
            // motor.Set_parameter(Motor_param::limit_cur, 15.0F);


            // _disable();
            // _wait_for_switch_off();
            // _wait_for_switch_on();
            // _auto_calibrate();
            // _manual_calibrate();
            motor.calibrated = true;
            
            _enable();
            // strcpy(log_info, "[Motor] motor auto enabled.");
            
            
            TickType_t lastWakeTime = xTaskGetTickCount();
            const TickType_t dt = pdMS_TO_TICKS(DELTA_T);
            
            // For frequency measurement
            unsigned long last_loop_time = micros();
            unsigned long loop_count = 0;
            float loop_freq = 0;

            while (true) {
                // Timing measurements
                unsigned long t_start = micros();
                unsigned long t_checkpoint;
                
                // esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(DELAY_PERIOD));
                t_checkpoint = micros();
                unsigned long time_delay = t_checkpoint - t_start;
                
                // Calculate loop frequency
                unsigned long current_time = micros();
                unsigned long delta_time = current_time - last_loop_time;
                last_loop_time = current_time;
                loop_freq = 1000000.0 / delta_time; // Convert microseconds to Hz
                loop_count++;

                t_checkpoint = micros();
                _check_remote_switch();
                unsigned long time_remote_switch = micros() - t_checkpoint;
                
                t_checkpoint = micros();
                _check_commands();
                unsigned long time_check_commands = micros() - t_checkpoint;

                t_checkpoint = micros();
                float filtered = 0;
                if (enable_filter)
                    filtered = filter.filter(target_pos);
                else
                    filtered = target_pos;
                unsigned long time_filter = micros() - t_checkpoint;
                
                t_checkpoint = micros();
                _step(filtered, target_vel, command_kp, command_kd);
                unsigned long time_step = micros() - t_checkpoint;
                // _step(filtered, target_vel, 12, 0.4);
                // _step(filtered, 25, 0.8);
                // _step(filtered, 20, 0.8);

                // large_motor_pos = motor.Read_parameter(Motor_param::mech_pos);
                // large_motor_pos = motor.Read_parameter(Motor_param::mech_pos);

                t_checkpoint = micros();
                // st = motor.Get_state();
                unsigned long time_get_state = micros() - t_checkpoint;
                
                // Serial.printf("!! Motor Angle: %f\n", st.angle);
                // Serial.printf("!! Loop Frequency: %.2f Hz (Period: %.2f ms)\n", loop_freq, 1000.0/loop_freq);
                // Serial.printf("!! Timing breakdown (us): Delay=%lu, RemoteSwitch=%lu, Commands=%lu, Filter=%lu, Step=%lu, GetState=%lu\n",
                //               time_delay, time_remote_switch, time_check_commands, time_filter, time_step, time_get_state);
                
                t_checkpoint = micros();
                // _manage_monitor();
                unsigned long time_manage_monitor = micros() - t_checkpoint;
                
                DEBUG_PRINT("voltage");
                DEBUG_PRINT(voltage);
                motor_error = st.error_state; // motor.get_error();
                // motor_error2 = motor.get_error();

                t_checkpoint = micros();
                _check_health();
                unsigned long time_check_health = micros() - t_checkpoint;

                DEBUG_PRINT("Position");
                DEBUG_PRINT(st.angle);

                t_checkpoint = micros();
                _check_safety();
                unsigned long time_check_safety = micros() - t_checkpoint;
                
                unsigned long time_total = micros() - t_start;
                // Serial.printf("!! Timing continued (us): ManageMonitor=%lu, CheckHealth=%lu, CheckSafety=%lu, Total=%lu\n",
                            //   time_manage_monitor, time_check_health, time_check_safety, time_total);
                // vTaskDelayUntil(&lastWakeTime, dt); ????????????????????????

            }
        }
    }  // namespace MotorTask



}