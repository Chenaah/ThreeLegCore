#include "tasks.hpp"
#include "LocalPolicy.hpp"

// Forward declaration of the global policy instance defined in main.cpp
extern LocalPolicy local_policy;

static constexpr size_t LOCAL_LATENT_HISTORY_BUFFER_STEPS =
    (LOCAL_LATENT_HISTORY_STEPS > 0) ? LOCAL_LATENT_HISTORY_STEPS : 1;

// Circular history rings: newest at [0], oldest at [N-1]
static float local_frame_history[LOCAL_FRAME_HISTORY_STEPS][LOCAL_FRAME_DIM] = {};
static float local_latent_history[LOCAL_LATENT_HISTORY_BUFFER_STEPS][LOCAL_LATENT_DIM] = {};

static void push_local_obs_frame(const float frame[LOCAL_FRAME_DIM]) {
    for (size_t h = LOCAL_FRAME_HISTORY_STEPS - 1; h > 0; --h) {
        for (size_t d = 0; d < LOCAL_FRAME_DIM; ++d)
            local_frame_history[h][d] = local_frame_history[h - 1][d];
    }
    for (size_t d = 0; d < LOCAL_FRAME_DIM; ++d)
        local_frame_history[0][d] = frame[d];
}

static void push_local_latent(const float latent[LOCAL_LATENT_DIM]) {
    if constexpr (LOCAL_LATENT_HISTORY_STEPS > 0) {
        for (size_t h = LOCAL_LATENT_HISTORY_STEPS - 1; h > 0; --h) {
            for (size_t d = 0; d < LOCAL_LATENT_DIM; ++d)
                local_latent_history[h][d] = local_latent_history[h - 1][d];
        }
        for (size_t d = 0; d < LOCAL_LATENT_DIM; ++d)
            local_latent_history[0][d] = latent[d];
    }
}

static std::array<float, LOCAL_OBS_DIM> build_local_obs() {
    std::array<float, LOCAL_OBS_DIM> obs{};
    size_t out_idx = 0;

    for (size_t h = 0; h < LOCAL_FRAME_HISTORY_STEPS; ++h) {
        const float* frame = local_frame_history[LOCAL_FRAME_HISTORY_STEPS - 1 - h];
        for (size_t d = 0; d < LOCAL_FRAME_DIM; ++d)
            obs[out_idx++] = frame[d];
    }

    if constexpr (LOCAL_LATENT_HISTORY_STEPS > 0) {
        for (size_t h = 0; h < LOCAL_LATENT_HISTORY_STEPS; ++h) {
            const float* latent = local_latent_history[LOCAL_LATENT_HISTORY_STEPS - 1 - h];
            for (size_t d = 0; d < LOCAL_LATENT_DIM; ++d)
                obs[out_idx++] = latent[d];
        }
    }
    return obs;
}

// Helper: project gravity vector into body frame from quaternion [x,y,z,w]
// Matches training: quat_rotate_inverse(q, [0,0,-1])
//   pg_x =  2*(qw*qy - qx*qz)
//   pg_y = -2*(qw*qx + qy*qz)
//   pg_z =  1 - 2*(qw*qw + qz*qz)  =  -(1 - 2*(qx*qx + qy*qy))  [NOT negated]
static void projected_gravity(const float q[4], float pg[3]) {
    float qx = q[0], qy = q[1], qz = q[2], qw = q[3];
    // R^T * (0,0,-1): rotate world gravity into body frame
    pg[0] =  2.0f * (qw * qy - qx * qz);
    pg[1] = -2.0f * (qw * qx + qy * qz);
    pg[2] = 1.0f - 2.0f * (qw * qw + qz * qz);
}

static void build_current_local_frame(
    const float q[4],
    const float gyro[3],
    float dof_pos,
    float dof_vel,
    float frame[LOCAL_FRAME_DIM]
) {
    float pg[3] = {};
    bool pg_ready = false;
    size_t idx = 0;

    for (size_t component_idx = 0; component_idx < LOCAL_FRAME_COMPONENT_COUNT; ++component_idx) {
        switch (LOCAL_FRAME_COMPONENTS[component_idx]) {
            case LocalObsField::ProjectedGravity:
                if (!pg_ready) {
                    projected_gravity(q, pg);
                    pg_ready = true;
                }
                frame[idx++] = pg[0];
                frame[idx++] = pg[1];
                frame[idx++] = pg[2];
                break;
            case LocalObsField::Gyro:
                frame[idx++] = gyro[0];
                frame[idx++] = gyro[1];
                frame[idx++] = gyro[2];
                break;
            case LocalObsField::DofPos:
                frame[idx++] = dof_pos;
                break;
            case LocalObsField::DofVel:
                frame[idx++] = dof_vel;
                break;
        }
    }
}

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
    int received_joint_id = -1;             // Action/joint index from last command (-1 = all)
    float received_latent[LOCAL_LATENT_DIM] = {};  // Updated at 20 Hz by PC
    bool local_policy_active = false;       // Set to true in setup() if policy loaded successfully
    std::queue<int> info_queue;
    bool motor_calibrated = false;  // Motor calibration status

    // Command interpolator instance
    CommandInterpolator cmd_interpolator;

    // Config
    float offset = -1.0471975512; // 1.65806; // motor offset, shared for correcting sent observation
    const float DELTA_T = CONTROL_LOOP_DT_MS; // Control loop period in ms

    namespace MotorTask {

        Motor motor;
        ButterworthFilter filter(15, PD_LOOP_HZ);  // 15 Hz cutoff at 500 Hz sampling
        int motor_running = false;
        unsigned long epi_start_time;
        Motor_fault_state last_fault;  // Store last received fault
        bool fault_received = false;   // Flag to indicate if a fault was received

        /**
         * @brief Convert fault state to a 32-bit integer for transmission
         * 
         * Bit layout (matches datasheet):
         *   bit16: Phase-A current sampling overflow
         *   bit15-8: Overload fault (8 bits)
         *   bit7: Encoder not calibrated
         *   bit5: Phase-C current sampling overflow
         *   bit4: Phase-B current sampling overflow
         *   bit3: Over-voltage fault
         *   bit2: Under-voltage fault
         *   bit1: Driver chip fault
         *   bit0: Motor over-temperature fault
         * 
         * Additional bits (upper 16 bits):
         *   bit24: Temperature warning
         *   bit20-17: Fault flag (4 bits)
         * 
         * @param fault The fault state struct
         * @return uint32_t Packed fault bits
         */
        uint32_t fault_state_to_uint32(const Motor_fault_state& fault) {
            uint32_t result = 0;
            
            // Lower 17 bits - match datasheet layout
            result |= (fault.motor_over_temp ? 1 : 0) << 0;       // bit0
            result |= (fault.driver_chip_fault ? 1 : 0) << 1;     // bit1
            result |= (fault.under_voltage ? 1 : 0) << 2;         // bit2
            result |= (fault.over_voltage ? 1 : 0) << 3;          // bit3
            result |= (fault.phase_b_overflow ? 1 : 0) << 4;      // bit4
            result |= (fault.phase_c_overflow ? 1 : 0) << 5;      // bit5
            result |= (fault.encoder_not_calibrated ? 1 : 0) << 7; // bit7
            result |= (uint32_t(fault.overload_fault) & 0xFF) << 8; // bit15-8
            result |= (fault.phase_a_overflow ? 1 : 0) << 16;     // bit16
            
            // Upper bits - additional info
            result |= (uint32_t(fault.fault_flag) & 0x0F) << 17;  // bit20-17: fault flag
            result |= (fault.temp_warning ? 1 : 0) << 24;         // bit24: temp warning
            
            return result;
        }

        /**
         * @brief Check for fault feedback frames from the motor
         * @return true if a fault was detected
         */
        bool _check_fault_frame() {
            Motor_fault_state fault;
            if (motor.Check_Fault_Frame(&fault)) {
                last_fault = fault;
                fault_received = true;
                
                Serial.printf("[Motor] Fault frame received from motor %d!\n", fault.motor_id);
                Serial.printf("[Motor] Fault flag: %d\n", fault.fault_flag);
                
                if (fault.fault_flag != 0) {
                    // Log specific faults
                    if (fault.motor_over_temp) {
                        Serial.println("[Motor] FAULT: Motor over-temperature (>80°C)!");
                    }
                    if (fault.under_voltage) {
                        Serial.println("[Motor] FAULT: Under-voltage!");
                    }
                    if (fault.over_voltage) {
                        Serial.println("[Motor] FAULT: Over-voltage!");
                    }
                    if (fault.driver_chip_fault) {
                        Serial.println("[Motor] FAULT: Driver chip fault!");
                    }
                    if (fault.encoder_not_calibrated) {
                        Serial.println("[Motor] FAULT: Encoder not calibrated!");
                    }
                    if (fault.phase_a_overflow) {
                        Serial.println("[Motor] FAULT: Phase-A current sampling overflow!");
                    }
                    if (fault.phase_b_overflow) {
                        Serial.println("[Motor] FAULT: Phase-B current sampling overflow!");
                    }
                    if (fault.phase_c_overflow) {
                        Serial.println("[Motor] FAULT: Phase-C current sampling overflow!");
                    }
                    if (fault.overload_fault) {
                        Serial.printf("[Motor] FAULT: Overload fault (value: %d)!\n", fault.overload_fault);
                    }
                    if (fault.temp_warning) {
                        Serial.println("[Motor] WARNING: Temperature warning (>75°C)!");
                    }
                    
                    send_led_message(LED_MSG_MOTOR_ERROR);
                    return true;
                }
            }
            return false;
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
            if (!motor.calibrated){
                safe = false;
                enqueue(info_queue, 300);
            }
            return safe;
        }

        void _enable(){
            DEBUG_PRINT("motor.Enable()");
            send_led_message(LED_MSG_MOTOR_ON);
            enqueue(info_queue, 301);

            cmd_interpolator.reset();  // Clear stale waypoints before enabling
            filter.reset();            // Clear filter state to avoid transient
            st = motor.Enable();
            motor_running = true;
            epi_start_time = millis();
        }

        void _disable(){
            DEBUG_PRINT("motor.Disable()");
            send_led_message(LED_MSG_MOTOR_OFF);
            enqueue(info_queue, 302);
            cmd_interpolator.reset();  // Clear interpolator state on disable
            st = motor.Disable();
            motor_running = false;
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
                st = motor.Set_control(0, target_angle + offset, target_vel, kp, kd);
           
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
            motor_calibrated = true;  // Sync calibration status
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
            motor_calibrated = true;  // Sync calibration status

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

            _disable();
            _wait_for_switch_on();
            _auto_calibrate();
            
            _enable();
            
            // PD loop runs at 500 Hz, policy inference at 100 Hz (every PD_SUBSTEPS ticks)
            TickType_t lastWakeTime = xTaskGetTickCount();
            const TickType_t dt = pdMS_TO_TICKS(CONTROL_LOOP_DT_MS);  // 2 ms

            // For frequency measurement and diagnostics
            unsigned long last_loop_time = micros();
            unsigned long loop_count = 0;
            float loop_freq = 0;
            float loop_freq_filtered = PD_LOOP_HZ; // EMA-filtered frequency
            unsigned long last_print_time = millis();

            // Torque rate limiter: prevent sudden torque spikes
            float last_torque_cmd = 0.0f;
            const float MAX_TORQUE_RATE = 50.0f * CONTROL_LOOP_DT_S; // max torque change per tick (N.m/tick)

            // Interpolation state for local-policy mode
            float prev_policy_target = 0.0f;  // target from previous policy tick
            float curr_policy_target = 0.0f;  // target from current policy tick
            int substep = 0;                   // counts 0..PD_SUBSTEPS-1

            while (true) {
                unsigned long t_start = micros();

                // === 1. Check remote switch & commands (lightweight) ===
                _check_remote_switch();
                _check_commands();

                // === 2. Determine target position ===
                float interp_pos, interp_vel, interp_kp, interp_kd;

                if (local_policy_active) {
                    // --- Local-policy mode (hierarchical deployment) ---
                    // Run policy every PD_SUBSTEPS ticks (100 Hz), interpolate at 500 Hz
                    if (substep == 0) {
                        // Policy tick: build obs and run inference
                        float frame[LOCAL_FRAME_DIM];

                        float q[4] = {
                            CommTask::data_to_send.imu.quaternion.x,
                            CommTask::data_to_send.imu.quaternion.y,
                            CommTask::data_to_send.imu.quaternion.z,
                            CommTask::data_to_send.imu.quaternion.w,
                        };
                        float gyro[3] = {
                            CommTask::data_to_send.imu.omega.x,
                            CommTask::data_to_send.imu.omega.y,
                            CommTask::data_to_send.imu.omega.z,
                        };
                        build_current_local_frame(
                            q,
                            gyro,
                            st.angle - offset,
                            st.angle_v,
                            frame
                        );

                        push_local_obs_frame(frame);
                        push_local_latent(received_latent);

                        std::array<float, LOCAL_OBS_DIM> local_obs = build_local_obs();

                        std::array<float, LOCAL_LATENT_DIM> latent_arr{};
                        for (size_t i = 0; i < LOCAL_LATENT_DIM; ++i)
                            latent_arr[i] = received_latent[i];

                        float motor_target = ::local_policy.select_action(latent_arr, local_obs);

                        // Shift targets for interpolation
                        prev_policy_target = curr_policy_target;
                        curr_policy_target = motor_target;
                    }

                    // Linear interpolation between prev and curr policy targets
                    // substep=0 → alpha=0 (curr_policy_target), substep=PD_SUBSTEPS-1 → near 1
                    // We interpolate forward: at substep 0 we are at curr, moving toward next (unknown)
                    // So instead interpolate from prev→curr over the substep window
                    float alpha = (float)(substep + 1) / (float)PD_SUBSTEPS;
                    interp_pos = prev_policy_target + alpha * (curr_policy_target - prev_policy_target);
                    interp_vel = 0.0f;
                    interp_kp  = DEPLOY_KP;
                    interp_kd  = DEPLOY_KD;

                    substep = (substep + 1) % PD_SUBSTEPS;

                } else {
                    // --- Legacy PD mode: interpolate from PC-sent position target ---
                    if (cmd_interpolator.isTimedOut(COMMAND_TIMEOUT_MS)) {
                        interp_pos = target_pos;
                        interp_vel = 0.0f;
                        interp_kp = command_kp;
                        interp_kd = command_kd;
                    } else {
                        cmd_interpolator.sample(interp_pos, interp_vel, interp_kp, interp_kd);
                    }
                }

                // === 3. Apply Butterworth low-pass filter (optional) ===
                float filtered_pos;
                if (enable_filter)
                    filtered_pos = filter.filter(interp_pos);
                else
                    filtered_pos = interp_pos;

                // === 4. Send command to motor ===
                _step(filtered_pos, interp_vel, interp_kp, interp_kd);

                // === 5. Update motor error state ===
                motor_error = st.error_state;

                // Check for fault feedback frames actively sent by the motor
                if (_check_fault_frame()) {
                    enqueue(info_queue, 316);
                }
                motor_error2 = fault_state_to_uint32(last_fault);

                _check_health();
                _check_safety();

                // === 6. Diagnostics (throttled to ~2 Hz to reduce Serial overhead) ===
                unsigned long current_time_us = micros();
                unsigned long delta_time = current_time_us - last_loop_time;
                last_loop_time = current_time_us;
                loop_freq = 1000000.0f / delta_time;
                loop_freq_filtered = 0.95f * loop_freq_filtered + 0.05f * loop_freq;
                loop_count++;

                unsigned long now_ms = millis();
                if (now_ms - last_print_time >= 500) {
                    last_print_time = now_ms;
                    unsigned long time_total = micros() - t_start;
                    if (local_policy_active) {
                        Serial.printf("[Motor/NN] f=%.1fHz pos=%.3f target=%.3f latent[0]=%.3f sub=%d dt_us=%lu\n",
                                      loop_freq_filtered, st.angle, filtered_pos,
                                      received_latent[0], substep, time_total);
                    } else {
                        Serial.printf("[Motor] f=%.1fHz pos=%.3f interp_pos=%.3f vel=%.2f kp=%.1f kd=%.2f dt_us=%lu\n",
                                      loop_freq_filtered, st.angle, filtered_pos, interp_vel,
                                      interp_kp, interp_kd, time_total);
                    }
                }

                DEBUG_PRINT("Position");
                DEBUG_PRINT(st.angle);

                // === 7. Sleep until next tick (deterministic timing) ===
                vTaskDelayUntil(&lastWakeTime, dt);
            }
        }
    }  // namespace MotorTask



}
