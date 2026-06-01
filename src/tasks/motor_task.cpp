#include "tasks.hpp"
#include "LocalPolicy.hpp"
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// Forward declaration of the global onboard-model instance defined in main.cpp
extern LocalPolicy onboard_model;

// CAN-loss indicator surfaced via bit 25 of driver_error (unused on Cybergear).
// Bits 0-5, 7, 16, 24 are real driver fault flags; 8-15 overload; 17-20 fault_flag.
// Bit 25 is unused on the wire, so we hijack it as a transport-health bit.
static constexpr uint32_t DRIVER_ERROR_CAN_LOST_BIT = 1u << 25;
// Number of consecutive failed CAN_Transceive() calls before we consider the bus
// to that module persistently lost. With CAN_WAIT_TIME=10ms and up to 2 timeouts
// per call, 5 failures ≈ up to 100ms of dead bus — past any single-cycle glitch.
static constexpr uint32_t CAN_LOSS_FAILURE_THRESHOLD = 5;

static constexpr size_t COMMAND_CONTEXT_HISTORY_BUFFER_STEPS =
    (Task::COMMAND_CONTEXT_HISTORY_STEPS > 0) ? Task::COMMAND_CONTEXT_HISTORY_STEPS : 1;

// Circular history rings: newest at [0], oldest at [N-1]
static float local_frame_history[LOCAL_FRAME_HISTORY_STEPS][LOCAL_FRAME_DIM] = {};
static float command_context_history[COMMAND_CONTEXT_HISTORY_BUFFER_STEPS][Task::COMMAND_CONTEXT_STORAGE_DIM] = {};
static bool local_frame_history_initialized = false;
static bool command_context_history_initialized = false;

static float apply_local_obs_transform(float value, LocalObsTransform transform) {
    switch (transform) {
        case LocalObsTransform::None:
            return value;
        case LocalObsTransform::Cos:
            return cosf(value);
        case LocalObsTransform::Sin:
            return sinf(value);
    }
    return value;
}

static void reset_local_obs_history() {
    for (size_t h = 0; h < LOCAL_FRAME_HISTORY_STEPS; ++h) {
        for (size_t d = 0; d < LOCAL_FRAME_DIM; ++d)
            local_frame_history[h][d] = 0.0f;
    }
    for (size_t h = 0; h < COMMAND_CONTEXT_HISTORY_BUFFER_STEPS; ++h) {
        for (size_t d = 0; d < Task::COMMAND_CONTEXT_STORAGE_DIM; ++d)
            command_context_history[h][d] = 0.0f;
    }
    local_frame_history_initialized = false;
    command_context_history_initialized = false;
}

static void push_local_obs_frame(const float frame[LOCAL_FRAME_DIM]) {
    if (!local_frame_history_initialized) {
        for (size_t h = 0; h < LOCAL_FRAME_HISTORY_STEPS; ++h) {
            for (size_t d = 0; d < LOCAL_FRAME_DIM; ++d)
                local_frame_history[h][d] = frame[d];
        }
        local_frame_history_initialized = true;
        return;
    }
    for (size_t h = LOCAL_FRAME_HISTORY_STEPS - 1; h > 0; --h) {
        for (size_t d = 0; d < LOCAL_FRAME_DIM; ++d)
            local_frame_history[h][d] = local_frame_history[h - 1][d];
    }
    for (size_t d = 0; d < LOCAL_FRAME_DIM; ++d)
        local_frame_history[0][d] = frame[d];
}

static void push_command_context(const float command_context[Task::COMMAND_CONTEXT_STORAGE_DIM]) {
    if constexpr (Task::COMMAND_CONTEXT_HISTORY_STEPS > 0) {
        if (!command_context_history_initialized) {
            for (size_t h = 0; h < Task::COMMAND_CONTEXT_HISTORY_STEPS; ++h) {
                for (size_t d = 0; d < Task::COMMAND_CONTEXT_DIM; ++d)
                    command_context_history[h][d] = command_context[d];
            }
            command_context_history_initialized = true;
            return;
        }
        for (size_t h = Task::COMMAND_CONTEXT_HISTORY_STEPS - 1; h > 0; --h) {
            for (size_t d = 0; d < Task::COMMAND_CONTEXT_DIM; ++d)
                command_context_history[h][d] = command_context_history[h - 1][d];
        }
        for (size_t d = 0; d < Task::COMMAND_CONTEXT_DIM; ++d)
            command_context_history[0][d] = command_context[d];
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

    if constexpr (Task::COMMAND_CONTEXT_HISTORY_STEPS > 0) {
        for (size_t h = 0; h < Task::COMMAND_CONTEXT_HISTORY_STEPS; ++h) {
            const float* command_context = command_context_history[Task::COMMAND_CONTEXT_HISTORY_STEPS - 1 - h];
            for (size_t d = 0; d < Task::COMMAND_CONTEXT_DIM; ++d)
                obs[out_idx++] = command_context[d];
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
    float last_action,
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
                frame[idx++] = apply_local_obs_transform(
                    dof_pos,
                    LOCAL_FRAME_TRANSFORMS[component_idx]
                );
                break;
            case LocalObsField::DofVel:
                frame[idx++] = apply_local_obs_transform(
                    dof_vel,
                    LOCAL_FRAME_TRANSFORMS[component_idx]
                );
                break;
            case LocalObsField::LastAction:
                frame[idx++] = apply_local_obs_transform(
                    last_action,
                    LOCAL_FRAME_TRANSFORMS[component_idx]
                );
                break;
        }
    }
}

static uint64_t local_debug_scenario_start_us = 0;
static int local_debug_last_mode = -1;
static float local_debug_last_signature[6] = {};
static bool local_debug_signature_valid = false;

static bool local_debug_protocol_valid(
    const float command_context[Task::COMMAND_CONTEXT_STORAGE_DIM]
) {
    if constexpr (Task::COMMAND_CONTEXT_STORAGE_DIM <= Task::LOCAL_DEBUG_CTX_BIAS) {
        return false;
    }
    return fabsf(
        command_context[Task::LOCAL_DEBUG_CTX_VERSION] - Task::LOCAL_DEBUG_PROTOCOL_VERSION
    ) < 1e-3f;
}

static void maybe_reset_local_debug_clock(
    int control_mode,
    const float command_context[Task::COMMAND_CONTEXT_STORAGE_DIM]
) {
    if (control_mode != Task::CONTROL_MODE_LOCAL_DEBUG_SCENARIO) {
        local_debug_last_mode = control_mode;
        local_debug_signature_valid = false;
        return;
    }

    float signature[6] = {
        command_context[Task::LOCAL_DEBUG_CTX_VERSION],
        command_context[Task::LOCAL_DEBUG_CTX_SCENARIO_ID],
        command_context[Task::LOCAL_DEBUG_CTX_AMPLITUDE],
        command_context[Task::LOCAL_DEBUG_CTX_FREQUENCY_HZ],
        command_context[Task::LOCAL_DEBUG_CTX_PHASE_OFFSET_RAD],
        command_context[Task::LOCAL_DEBUG_CTX_BIAS],
    };

    bool changed = (local_debug_last_mode != control_mode) || !local_debug_signature_valid;
    if (!changed) {
        for (size_t i = 0; i < 6; ++i) {
            if (fabsf(signature[i] - local_debug_last_signature[i]) > 1e-6f) {
                changed = true;
                break;
            }
        }
    }

    if (changed) {
        local_debug_scenario_start_us = esp_timer_get_time();
        for (size_t i = 0; i < 6; ++i)
            local_debug_last_signature[i] = signature[i];
        local_debug_signature_valid = true;
    }
    local_debug_last_mode = control_mode;
}

static float compute_local_debug_action(
    const float command_context[Task::COMMAND_CONTEXT_STORAGE_DIM],
    int joint_id
) {
    if (!local_debug_protocol_valid(command_context))
        return 0.0f;

    int scenario_id = (int)lroundf(command_context[Task::LOCAL_DEBUG_CTX_SCENARIO_ID]);
    float amplitude = command_context[Task::LOCAL_DEBUG_CTX_AMPLITUDE];
    float frequency_hz = command_context[Task::LOCAL_DEBUG_CTX_FREQUENCY_HZ];
    float phase_offset_rad = command_context[Task::LOCAL_DEBUG_CTX_PHASE_OFFSET_RAD];
    float bias = command_context[Task::LOCAL_DEBUG_CTX_BIAS];
    float elapsed_sec = 0.0f;
    if (local_debug_scenario_start_us != 0) {
        elapsed_sec = (float)(esp_timer_get_time() - local_debug_scenario_start_us) * 1e-6f;
    }

    switch (scenario_id) {
        case 1:
            if (joint_id <= 0)
                return 0.0f;
            return bias + amplitude * sinf(
                6.28318530718f * frequency_hz * elapsed_sec +
                (float)(joint_id - 1) * phase_offset_rad
            );
        case 2: {
            float phase = 6.28318530718f * frequency_hz * elapsed_sec + phase_offset_rad;
            if (joint_id == 1)
                return bias + amplitude * sinf(phase);
            if (joint_id == 2)
                return bias - amplitude * sinf(phase);
            return 0.0f;
        }
        default:
            return 0.0f;
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
    float hall_threshold = 940.0F;
    int received_control_mode =
        DEPLOY_USE_XBOX_CONTROLLER ? CONTROL_MODE_ONBOARD_MODEL : CONTROL_MODE_DIRECT_PD;
    float received_joint_offset = DEPLOY_DEFAULT_DOF_POS[0];
    int received_policy_hash = DEPLOY_USE_XBOX_CONTROLLER ? DEPLOY_POLICY_HASH : 0;
    int received_joint_id = DEPLOY_USE_XBOX_CONTROLLER ? 0 : -1; // Action/joint index from last command (-1 = all)
    float received_command_context[COMMAND_CONTEXT_STORAGE_DIM] = {};  // Updated by the PC command stream
    bool onboard_model_loaded = false;      // Set to true in setup() if model loads successfully
    int policy_status_bits = 0;
    int policy_error_code = POLICY_ERROR_NONE;
    int policy_debug_valid = 0;
    int policy_debug_seq = 0;
    float policy_debug_nn_action = 0.0f;
    float policy_debug_motor_target = 0.0f;
    float policy_debug_interp_target = 0.0f;
    float policy_debug_applied_target = 0.0f;
    float policy_debug_joint_offset = 0.0f;
    float policy_debug_dof_pos = 0.0f;
    float policy_debug_dof_vel = 0.0f;
    float policy_debug_command_context[POLICY_DEBUG_COMMAND_CONTEXT_STORAGE_DIM] = {};
    float policy_debug_local_obs[LOCAL_OBS_DIM] = {};
    std::queue<int> info_queue;
    bool motor_calibrated = false;  // Motor calibration status
    bool motor_startup_ramping = false;
    uint32_t last_enable_attempt_ms = 0;

    // Command interpolator instance
    CommandInterpolator cmd_interpolator;

    // Config
    float offset = kMotorFrameOffset; // motor offset, shared for correcting sent observation
    const float DELTA_T = CONTROL_LOOP_DT_MS; // Control loop period in ms
    static constexpr int CALIBRATION_MODE_NONE = 0;
    static constexpr int CALIBRATION_MODE_STARTUP_ABSOLUTE = -1;
    static constexpr int CALIBRATION_MODE_MANUAL = 1;
    static constexpr int CALIBRATION_MODE_AUTO = 2;
    int pending_calibration_mode = CALIBRATION_MODE_NONE;

    static void _set_pending_calibration_mode(int mode) {
        pending_calibration_mode = mode;
    }

    static void _clear_pending_calibration_mode() {
        pending_calibration_mode = CALIBRATION_MODE_NONE;
    }

    static int _default_pending_calibration_mode() {
        return kMotorRequiresZeroCalibration
            ? CALIBRATION_MODE_AUTO
            : CALIBRATION_MODE_STARTUP_ABSOLUTE;
    }

    static int compute_policy_status_bits() {
        int status = 0;
        if (::onboard_model.is_loaded())
            status |= POLICY_STATUS_LOADED;
        if (::onboard_model.is_sanity_ok())
            status |= POLICY_STATUS_SANITY_OK;
        if (received_policy_hash != 0)
            status |= POLICY_STATUS_PC_HASH_SEEN;
        if (::onboard_model.is_loaded() && ::onboard_model.is_build_hash_match() &&
            received_policy_hash != 0 &&
            ::onboard_model.get_policy_hash() == received_policy_hash) {
            status |= POLICY_STATUS_HASH_MATCH;
        }
        if ((status & (POLICY_STATUS_LOADED | POLICY_STATUS_SANITY_OK |
                       POLICY_STATUS_PC_HASH_SEEN | POLICY_STATUS_HASH_MATCH)) ==
            (POLICY_STATUS_LOADED | POLICY_STATUS_SANITY_OK |
             POLICY_STATUS_PC_HASH_SEEN | POLICY_STATUS_HASH_MATCH)) {
            status |= POLICY_STATUS_RUNTIME_READY;
        }
        return status;
    }

    static int validate_onboard_model_runtime() {
        policy_status_bits = compute_policy_status_bits();

        if (!::onboard_model.is_loaded())
            return POLICY_ERROR_NOT_LOADED;
        if (!::onboard_model.is_build_hash_match())
            return POLICY_ERROR_HASH_MISMATCH;
        if (!::onboard_model.is_sanity_ok())
            return POLICY_ERROR_SANITY_FAILED;
        if (received_policy_hash == 0)
            return POLICY_ERROR_NO_PC_HASH;
        if (::onboard_model.get_policy_hash() != received_policy_hash)
            return POLICY_ERROR_HASH_MISMATCH;
        return POLICY_ERROR_NONE;
    }

    static void report_policy_error_if_needed(int error_code) {
        policy_error_code = error_code;
        policy_status_bits = compute_policy_status_bits();
        if (error_code != POLICY_ERROR_NONE) {
            send_led_message(LED_MSG_POLICY_ERROR);
        }
    }

    namespace MotorTask {

        bool _initialize_absolute_encoder_startup();
        bool _auto_calibrate(
            bool command_triggered = false,
            int calibration_mode = CALIBRATION_MODE_AUTO
        );
        bool _manual_calibrate(bool command_triggered = false);
        bool _resume_pending_calibration_if_needed();

        enum class StepCommandStatus : uint8_t {
            Sent = 0,
            SkipMotorOff = 1,
            SkipNotCalibrated = 2,
        };

        Motor motor(kMotorType);
        ButterworthFilter filter(15, PD_LOOP_HZ);      // 15 Hz cutoff for target position
        ButterworthFilter pos_filter(30, PD_LOOP_HZ);  // 30 Hz cutoff for measured dof_pos
        ButterworthFilter vel_filter(30, PD_LOOP_HZ);  // 30 Hz cutoff for measured dof_vel
        float filtered_dof_pos = 0.0f;  // filtered motor position (updated at PD rate)
        float filtered_dof_vel = 0.0f;  // filtered motor velocity (updated at PD rate)
        XboxSeriesXControllerESP32_asukiaaa::Core xbox_controller;
        bool xbox_started = false;
        bool xbox_ready = false;
        bool joystick_policy_enabled = !DEPLOY_USE_XBOX_CONTROLLER;
        bool xbox_btn_a_prev = false;
        bool xbox_btn_b_prev = false;
        bool xbox_btn_x_prev = false;
        bool xbox_btn_y_prev = false;
        int motor_running = false;
        unsigned long epi_start_time;
        Motor_fault_state last_fault;  // Store last received fault
        bool fault_received = false;   // Flag to indicate if a fault was received

        void _reset_xbox_button_edges() {
            xbox_btn_a_prev = false;
            xbox_btn_b_prev = false;
            xbox_btn_x_prev = false;
            xbox_btn_y_prev = false;
        }

        void _begin_xbox_controller_if_needed() {
            if constexpr (!DEPLOY_USE_XBOX_CONTROLLER) {
                return;
            }
            if (xbox_started) {
                return;
            }
            xbox_controller.begin();
            xbox_started = true;
            Serial.println("[Xbox] Waiting for controller connection...");
        }

        void _poll_xbox_controller() {
            if constexpr (!DEPLOY_USE_XBOX_CONTROLLER) {
                return;
            }

            _begin_xbox_controller_if_needed();
            xbox_controller.onLoop();

            const bool connected =
                xbox_controller.isConnected() &&
                !xbox_controller.isWaitingForFirstNotification();

            if (!connected) {
                if (xbox_ready) {
                    Serial.println("[Xbox] Controller disconnected. Disabling motor.");
                    remote_switch = 0;
                    joystick_policy_enabled = false;
                }
                xbox_ready = false;
                _reset_xbox_button_edges();
                return;
            }

            if (!xbox_ready) {
                xbox_ready = true;
                Serial.println("[Xbox] Controller connected. Press A to enable.");
            }

            const bool btn_a = xbox_controller.xboxNotif.btnA;
            const bool btn_b = xbox_controller.xboxNotif.btnB;
            const bool btn_x = xbox_controller.xboxNotif.btnX;
            const bool btn_y = xbox_controller.xboxNotif.btnY;

            if (btn_a && !xbox_btn_a_prev) {
                remote_switch = 1;
                joystick_policy_enabled = true;
                received_control_mode = CONTROL_MODE_ONBOARD_MODEL;
                received_joint_offset = DEPLOY_DEFAULT_DOF_POS[0];
                received_policy_hash = DEPLOY_POLICY_HASH;
                received_joint_id = 0;
                Serial.println("[Xbox] A pressed: onboard policy enabled.");
            }

            if (btn_b && !xbox_btn_b_prev) {
                joystick_policy_enabled = false;
                Serial.println("[Xbox] B pressed: policy soft-stopped.");
            }

            if (btn_x && !xbox_btn_x_prev) {
                remote_switch = 0;
                joystick_policy_enabled = false;
                Serial.println("[Xbox] X pressed: motor disabled.");
            }

            if (btn_y && !xbox_btn_y_prev) {
                remote_switch = 0;
                joystick_policy_enabled = false;
                Serial.println("[Xbox] Y pressed: motor disabled.");
            }

            xbox_btn_a_prev = btn_a;
            xbox_btn_b_prev = btn_b;
            xbox_btn_x_prev = btn_x;
            xbox_btn_y_prev = btn_y;
        }

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
            // Use the firmware-owned logical calibration state here.
            // The Motor library currently clears motor.calibrated on transient
            // CAN TX/RX failures, which is a transport-health signal rather than
            // a true loss of zero-position calibration.
            if (!motor_calibrated){
                safe = false;
                enqueue(info_queue, 300);
            }
            return safe;
        }

        bool _enable(bool allow_uncalibrated = false){
            if (!allow_uncalibrated && !motor_calibrated) {
                send_led_message(LED_MSG_WAIT_CALI);
                enqueue(info_queue, 300);
                Serial.println("[Motor] Enable blocked: logical calibration not complete.");
                return false;
            }

            if (received_control_mode == CONTROL_MODE_ONBOARD_MODEL) {
                int validation_error = validate_onboard_model_runtime();
                report_policy_error_if_needed(validation_error);
                if (validation_error != POLICY_ERROR_NONE) {
                    Serial.printf("[Model] Enable blocked: error=%d local_hash=%d pc_hash=%d sanity=%d\n",
                                  validation_error,
                                  ::onboard_model.get_policy_hash(),
                                  received_policy_hash,
                                  ::onboard_model.is_sanity_ok() ? 1 : 0);
                    enqueue(info_queue, 401 + validation_error);
                    return false;
                }
            } else {
                policy_error_code = POLICY_ERROR_NONE;
                policy_status_bits = compute_policy_status_bits();
            }

            cmd_interpolator.reset();  // Clear stale waypoints before enabling
            filter.reset();            // Clear filter state to avoid transient
            pos_filter.reset();
            vel_filter.reset();
            reset_local_obs_history();

            DEBUG_PRINT("motor.Enable()");
            st = motor.Enable();
            if (st.mode != 2) {
                motor_running = false;
                send_led_message(LED_MSG_MOTOR_ERROR);
                enqueue(info_queue, 317);
                Serial.printf(
                    "[Motor] Enable failed: mode=%d error_state=0x%02X calibrated=%d\n",
                    st.mode,
                    st.error_state,
                    motor.calibrated ? 1 : 0
                );
                return false;
            }

            send_led_message(LED_MSG_MOTOR_ON);
            enqueue(info_queue, 301);
            motor_running = true;
            switch_off_request = 0;
            epi_start_time = millis();
            return true;
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
            constexpr uint32_t ENABLE_RETRY_INTERVAL_MS = 100;
            if (motor_running && ! enable)
                _disable();
            else if (!motor_running && enable) {
                uint32_t now_ms = millis();
                if (now_ms - last_enable_attempt_ms >= ENABLE_RETRY_INTERVAL_MS) {
                    last_enable_attempt_ms = now_ms;
                    if (pending_calibration_mode == CALIBRATION_MODE_NONE && !motor_calibrated) {
                        _set_pending_calibration_mode(_default_pending_calibration_mode());
                    }
                    if (pending_calibration_mode != CALIBRATION_MODE_NONE) {
                        return _resume_pending_calibration_if_needed();
                    }
                    return _enable();
                }
                return false;
            }
            return motor_running;
        }

        bool _calibration_abort_requested(bool command_triggered = false) {
            if (remote_switch != 0) {
                return false;
            }
            if (!command_triggered) {
                return true;
            }
            return calibrate_command == 0;
        }

        bool _abort_calibration_if_requested(
            const char* stage,
            bool command_triggered = false
        ) {
            if (!_calibration_abort_requested(command_triggered)) {
                return false;
            }

            Serial.printf(
                "[Motor] Calibration aborted during %s (switch=%d calibrate=%d)\n",
                stage,
                remote_switch,
                calibrate_command
            );
            enqueue(info_queue, 318);
            _disable();
            return true;
        }

        bool _wait_abortable_ms(
            uint32_t total_ms,
            const char* stage,
            bool command_triggered = false,
            uint32_t poll_ms = 10
        ) {
            uint32_t waited_ms = 0;
            while (waited_ms < total_ms) {
                if (_abort_calibration_if_requested(stage, command_triggered)) {
                    return false;
                }
                uint32_t step_ms = poll_ms;
                if (step_ms > total_ms - waited_ms) {
                    step_ms = total_ms - waited_ms;
                }
                vTaskDelay(pdMS_TO_TICKS(step_ms));
                waited_ms += step_ms;
            }
            return !_abort_calibration_if_requested(stage, command_triggered);
        }

        Motor_state _manual_pd_control(float target_angle, float target_vel, float kp = 20, float kd = 0.5) {
            float pos_error = target_angle - large_motor_pos;
            float vel_error = target_vel - st.angle_v;
            float torque = kp * pos_error + kd * vel_error;
            st = motor.Set_control(torque, 0, 0, 0, 0);
            return st;
        }

        StepCommandStatus _step(float target_angle, float target_vel, float kp = 20, float kd = 0.5) {
            if (!motor_running)
                return StepCommandStatus::SkipMotorOff;
            if (!_safe())
                return StepCommandStatus::SkipNotCalibrated;

            st = motor.Set_control(0, target_angle + offset, target_vel, kp, kd);
            return StepCommandStatus::Sent;
        }

        void _wait_for_switch_off() {
            while (remote_switch != 0) {
                _poll_xbox_controller();
                vTaskDelay(pdMS_TO_TICKS(100));
                enqueue(info_queue, 303);
                st = motor.Get_state();
                Serial.println("[MotorTask] Waiting for switch OFF...");
                send_led_message(LED_MSG_UNSAFE);
            }
        }

        void _wait_for_switch_on() {
            while (remote_switch == 0) {
                _poll_xbox_controller();
                // esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(100));
                enqueue(info_queue, 304);
                st = motor.Get_state();
                Serial.println("[MotorTask] Waiting for switch ON...");
                send_led_message(LED_MSG_WAIT_CALI);

            }
        }

        void _wait_for_help() {
            _disable();
            while (1) {
                // esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(100));
                st = motor.Get_state();
                Serial.println("[MotorTask] Waiting for help...");
                send_led_message(LED_MSG_WAIT_HELP);

            }
        }

        void _check_safety() {
            if (!motor_running) {
                return;
            }

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

        bool _move_to_middle(bool command_triggered = false){
            enqueue(info_queue, 306);
            if (_abort_calibration_if_requested("move_to_middle.begin", command_triggered)) {
                return false;
            }
            if (!_enable()) {
                return false;
            }
            for (float i = 0; i <= PI/2; i += 0.01){
                if (_abort_calibration_if_requested("move_to_middle.step", command_triggered)) {
                    return false;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                if (_abort_calibration_if_requested("move_to_middle.step", command_triggered)) {
                    return false;
                }
                // motor.Set_position(offset*sin(i));
                _step(-offset*cos(i), 0, 20, 0.5);
                DEBUG_PRINT("==> ");
                DEBUG_PRINT(st.angle);
            }
            enqueue(info_queue, 307);
            return true;
        }

        void _init_motor(){
            motor.Init(MOTOR_ID, CAN_TX_PIN, CAN_RX_PIN);
        }

        void _mark_motor_calibrated() {
            motor.calibrated = true;
            motor_calibrated = true;
        }

        void _sync_runtime_offset_to_pi_range() {
            st = motor.Get_state();

            constexpr float kTwoPi = 2.0f * PI;
            float logical_angle = st.angle - offset;

            while (logical_angle > PI) {
                offset += kTwoPi;
                logical_angle -= kTwoPi;
            }
            while (logical_angle < -PI) {
                offset -= kTwoPi;
                logical_angle += kTwoPi;
            }

            Serial.printf(
                "[Motor] Startup offset sync: raw=%.3f logical=%.3f offset=%.3f\n",
                st.angle,
                logical_angle,
                offset
            );
        }

        bool _move_to_zero_with_soft_kp(bool command_triggered = false) {
            constexpr float kStartupPositionKp = 30.0f;
            constexpr float kStartupPositionKd = 2.0f;
            constexpr TickType_t kStepDelayMs = 20;       // ms per substep
            constexpr float kRampDurationMs   = 5000.0f;  // total ramp time in ms
            const int n_steps = (int)(kRampDurationMs / (float)kStepDelayMs);

            st = motor.Get_state();
            const float start_logical_angle = st.angle - offset;

            Serial.printf(
                "[Motor] Ramp to zero: logical_start=%.3f kp=%.1f kd=%.1f ramp_ms=%.0f offset=%.3f\n",
                start_logical_angle,
                kStartupPositionKp,
                kStartupPositionKd,
                kRampDurationMs,
                offset
            );

            // Linear interpolation over fixed time: always takes kRampDurationMs ms
            motor_startup_ramping = true;
            for (int i = 0; i <= n_steps; i++) {
                if (_abort_calibration_if_requested("startup_ramp", command_triggered)) {
                    motor_startup_ramping = false;
                    return false;
                }
                float alpha = (float)i / (float)n_steps;
                float cmd = start_logical_angle * (1.0f - alpha); // lerp toward 0
                StepCommandStatus s = _step(cmd, 0.0f, kStartupPositionKp, kStartupPositionKd);
                Serial.printf("[Ramp] i=%d/%d alpha=%.3f cmd=%.4f status=%d t=%lu\n",
                    i, n_steps, alpha, cmd, (int)s, millis());
                vTaskDelay(pdMS_TO_TICKS(kStepDelayMs));
            }

            motor_startup_ramping = false;
            if (!_wait_abortable_ms(500, "startup_ramp.settle", command_triggered, 20)) {
                return false;
            }
            return true;
        }

        bool _initialize_absolute_encoder_startup() {
            _sync_runtime_offset_to_pi_range();
            _mark_motor_calibrated();

            // Read position before enabling so we can hold it immediately after
            st = motor.Get_state();
            const float pre_enable_logical = st.angle - offset;

            // Retry enable — first attempt can return mode=0 if the motor needs
            // a moment after Set_mode(Motion) before it accepts Enable.
            bool enabled = false;
            for (int attempt = 0; attempt < 5 && !enabled; attempt++) {
                if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(200));
                if (_abort_calibration_if_requested("absolute_startup.enable_retry")) {
                    return false;
                }
                enabled = _enable();
                Serial.printf("[Motor] _enable attempt %d: %s\n", attempt + 1, enabled ? "ok" : "fail");
            }
            if (!enabled) {
                Serial.println("[Motor] Startup move aborted: _enable() failed after retries.");
                return false;
            }

            // Hold the pre-enable position immediately — prevents the motor from
            // snapping to encoder zero on the first motion-mode tick after Enable.
            Serial.printf("[Motor] pre_enable_logical=%.3f t=%lu\n", pre_enable_logical, millis());
            _step(pre_enable_logical, 0.0f, 3.0f, 0.5f);
            Serial.printf("[Motor] hold sent, entering ramp t=%lu\n", millis());

            return _move_to_zero_with_soft_kp();
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
        bool _spin_until_threshold(
            Motor &motor,
            float target_value,
            float speed,
            float &motor_angle_sum,
            bool reverse = false,
            bool command_triggered = false
        ) {
            motor_angle_sum = 0.0F;
            float motor_temp_val = 0.0F;

            if (_abort_calibration_if_requested("spin_until_threshold.start", command_triggered)) {
                return false;
            }

            // Set motor control speed and direction
            st = motor.Set_control(0, 0, reverse ? -speed : speed, 0, 40);

            // Wait until monitored_value is below the target threshold
            while (monitored_value < target_value) {
                if (_abort_calibration_if_requested("spin_until_threshold.search", command_triggered)) {
                    return false;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
                Serial.printf("[MotorTask] Spinning motor... monitored_value=%.2f target=%.2f\n", monitored_value, target_value);
                motor_temp_val = motor.Get_state().angle;
            }
            motor_angle_sum += motor_temp_val;

            // Wait until monitored_value exceeds a secondary threshold for finer control
            while (monitored_value > target_value - 20) {
                if (_abort_calibration_if_requested("spin_until_threshold.refine", command_triggered)) {
                    return false;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
                motor_temp_val = motor.Get_state().angle;
            }
            motor_angle_sum += motor_temp_val;

            return true;
        }

        // Main function to align motor and set zero position
        bool _align_motor_and_set_zero(
            Motor &motor,
            float hall_threshold,
            float speed = 0.5F,
            bool command_triggered = false
        ) {
            float motor_total = 0.0F;
            float motor_partial = 0.0F;

            // Spin motor in the forward direction
            if (!_spin_until_threshold(
                    motor,
                    hall_threshold,
                    speed,
                    motor_partial,
                    false,
                    command_triggered
                )) {
                return false;
            }
            motor_total += motor_partial;

            // Pause before reversing
            if (!_wait_abortable_ms(500, "align.pause", command_triggered, 10)) {
                return false;
            }

            // Spin motor in the reverse direction
            if (!_spin_until_threshold(
                    motor,
                    hall_threshold,
                    speed,
                    motor_partial,
                    true,
                    command_triggered
                )) {
                return false;
            }
            motor_total += motor_partial;

            // Average the total motor angle and set position
            if (_abort_calibration_if_requested("align.set_midpoint", command_triggered)) {
                return false;
            }
            st = motor.Set_control(0, motor_total / 4.0F, 0, 20, 0.5);
            if (!_wait_abortable_ms(1000, "align.settle", command_triggered, 10)) {
                return false;
            }

            // Disable motor and set zero position
            _disable();
            if (_abort_calibration_if_requested("align.set_zero", command_triggered)) {
                return false;
            }
            if (kMotorRequiresZeroCalibration) {
                st = motor.Set_zero();
            }
            return true;
        }

        bool _find_limit_and_set_zero(
            float torque_threshold=-2,
            float speed=-0.5F,
            bool command_triggered = false
        ) {
            if (_abort_calibration_if_requested("find_limit.start", command_triggered)) {
                return false;
            }
            st = motor.Set_control(torque_threshold, 0, speed, 0, 2);
            if (!_wait_abortable_ms(6000, "find_limit.search", command_triggered, 10)) {
                return false;
            }
            _disable();
            if (_abort_calibration_if_requested("find_limit.set_zero", command_triggered)) {
                return false;
            }
            if (kMotorRequiresZeroCalibration) {
                st = motor.Set_zero();
            }
            return true;
        }

        bool _auto_calibrate(
            bool command_triggered,
            int calibration_mode
        ) {
            if (!kMotorRequiresZeroCalibration) {
                _mark_motor_calibrated();
                return true;
            }

            send_led_message(LED_MSG_CALI);
            st = motor.Get_state();
            MonitorTask::set_channel(ADC_CHANNEL_HALL);
            // TODO: Check the reading from the hall sensor is correct
            if (!_wait_abortable_ms(100, "auto_calibrate.prepare", command_triggered, 10)) {
                return false;
            }
            _disable();
            if (_abort_calibration_if_requested("auto_calibrate.reinit", command_triggered)) {
                return false;
            }
            _init_motor();
            st = motor.Set_zero();
            if (_abort_calibration_if_requested("auto_calibrate.zero", command_triggered)) {
                return false;
            }
            _enable(true);
            send_led_message(LED_MSG_CALI);

            enqueue(info_queue, 308);

            if (calibration_mode < 4){
                // Search for the magnetic marker
                if (!_align_motor_and_set_zero(motor, hall_threshold, 0.5F, command_triggered)) {
                    return false;
                }
            } else if (calibration_mode == 4){
                // Move to the mechanical limit
                if (!_find_limit_and_set_zero(5, 0.1, command_triggered)) {
                    return false;
                }
                offset = 0;
            } else if (calibration_mode == 5){
                // Move to the mechanical limit
                if (!_find_limit_and_set_zero(-5, -0.1, command_triggered)) {
                    return false;
                }
                offset = 0;
            }
            _mark_motor_calibrated();

            MonitorTask::set_channel(ADC_CHANNEL_VOLTAGE);
            enqueue(info_queue, 309);
            // vTaskDelay(pdMS_TO_TICKS(100));
            if (_abort_calibration_if_requested("auto_calibrate.complete", command_triggered)) {
                return false;
            }
            _enable();
            if (offset != 0) {
                return _move_to_middle(command_triggered);
            }
            return true;
        }

        bool _manual_calibrate(bool command_triggered){
            if (!kMotorRequiresZeroCalibration) {
                _mark_motor_calibrated();
                return true;
            }

            enqueue(info_queue, 310);
            _disable();
            _init_motor();
            if (!_wait_abortable_ms(100, "manual_calibrate.reinit", command_triggered, 10)) {
                return false;
            }
            st = motor.Set_zero();
            if (!_wait_abortable_ms(100, "manual_calibrate.zero", command_triggered, 10)) {
                return false;
            }
            _mark_motor_calibrated();
            enqueue(info_queue, 311);
            if (!_wait_abortable_ms(100, "manual_calibrate.complete", command_triggered, 10)) {
                return false;
            }
            return _move_to_middle(command_triggered);

        }

        bool _resume_pending_calibration_if_needed() {
            int calibration_mode = pending_calibration_mode;
            if (calibration_mode == CALIBRATION_MODE_NONE) {
                return false;
            }

            Serial.printf("[Motor] Resuming pending calibration mode %d\n", calibration_mode);

            bool success = false;
            if (calibration_mode == CALIBRATION_MODE_STARTUP_ABSOLUTE) {
                success = _initialize_absolute_encoder_startup();
            } else if (calibration_mode == CALIBRATION_MODE_MANUAL) {
                success = _manual_calibrate(false);
            } else {
                success = _auto_calibrate(false, calibration_mode);
            }

            if (success) {
                _clear_pending_calibration_mode();
            }
            return success;
        }

        void _check_commands() {
            if (calibrate_command == 1){
                // Manual calibration
                _set_pending_calibration_mode(CALIBRATION_MODE_MANUAL);
                enqueue(info_queue, 312);
                if (!_wait_abortable_ms(100, "manual_calibrate.command", true, 10)) {
                    return;
                }
                // _auto_calibrate();
                if (!_manual_calibrate(true)) {
                    return;
                }
                _clear_pending_calibration_mode();
                _wait_abortable_ms(100, "manual_calibrate.cleanup", true, 10);
            } else if (calibrate_command == 2 || calibrate_command == 4 || calibrate_command == 5){
                // Auto calibration
                int requested_mode = calibrate_command;
                _set_pending_calibration_mode(requested_mode);
                if (!_wait_abortable_ms(100, "auto_calibrate.command", true, 10)) {
                    return;
                }
                if (!_auto_calibrate(true, requested_mode)) {
                    return;
                }
                _clear_pending_calibration_mode();
                _wait_abortable_ms(100, "auto_calibrate.cleanup", true, 10);
            } else if (calibrate_command == 3){
                // Set current position as zero position
                if (kMotorRequiresZeroCalibration) {
                    st = motor.Set_zero();
                }
            }

            if (restart_command){
                // Ignore software restart requests during debugging.
                // Leaving the command latched would repeatedly hit this branch.
                restart_command = 0;
                // esp_restart();
                // _wait_for_help();
            }
        }

        void _check_health() {
            if ((motor_error >> 6) & 0x03 == 0 && motor_running){
                send_led_message(LED_MSG_MOTOR_ERROR);
                Serial.println("[Motor] Motor error detected.");
            }

        }

        void init_xbox_controller() {
            _begin_xbox_controller_if_needed();
        }

        void run(void *pvParameters) {
            delay(2000); // Wait for the motor switch
            _init_motor();
            enqueue(info_queue, 313);

            motor.Set_mode(Motor_mode::Motion);

            _disable();
            _wait_for_switch_on();
            if (kMotorRequiresZeroCalibration) {
                _set_pending_calibration_mode(CALIBRATION_MODE_AUTO);
                if (_auto_calibrate(false, CALIBRATION_MODE_AUTO)) {
                    _clear_pending_calibration_mode();
                }
            } else {
                _set_pending_calibration_mode(CALIBRATION_MODE_STARTUP_ABSOLUTE);
                if (_initialize_absolute_encoder_startup()) {
                    _clear_pending_calibration_mode();
                }
            }

            // Only enable here if startup didn't already succeed — avoids a
            // second Enable that would let the motor snap to the PC target.
            if (!motor_running) {
                _enable();
            }
            
            // PD loop runs at 500 Hz. Policy inference rate comes from deploy_config.h.
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

            // Interpolation state for onboard-model mode
            float prev_policy_target = 0.0f;  // target from previous policy tick
            float curr_policy_target = 0.0f;  // target from current policy tick
            int substep = 0;                   // counts 0..PD_SUBSTEPS-1
            float last_onboard_action = 0.0f; // previous raw policy action, used in next observation

            while (true) {
                unsigned long t_start = micros();

                // === 1. Check remote switch & commands (lightweight) ===
                _poll_xbox_controller();
                _check_remote_switch();
                _check_commands();

                // === 2. Filter motor sensor readings at PD rate (500 Hz) ===
                filtered_dof_pos = pos_filter.filter(st.angle - offset);
                filtered_dof_vel = vel_filter.filter(st.angle_v);

                // === 3. Determine target position ===
                float interp_pos, interp_vel, interp_kp, interp_kd;

                bool requested_onboard_model =
                    (received_control_mode == CONTROL_MODE_ONBOARD_MODEL);
                bool requested_local_debug =
                    (received_control_mode == CONTROL_MODE_LOCAL_DEBUG_SCENARIO);
                bool use_onboard_model =
                    requested_onboard_model && onboard_model_loaded;
                bool use_local_debug = requested_local_debug;
                const bool policy_tick = (substep == 0);

                std::array<float, LOCAL_OBS_DIM> debug_local_obs{};
                std::array<float, COMMAND_CONTEXT_DIM> debug_command_context{};
                bool debug_obs_ready = false;

                if (requested_onboard_model) {
                    int validation_error = validate_onboard_model_runtime();
                    report_policy_error_if_needed(validation_error);
                    if (validation_error != POLICY_ERROR_NONE) {
                        if (motor_running) {
                            Serial.printf("[Model] Runtime validation failed: error=%d local_hash=%d pc_hash=%d\n",
                                          validation_error,
                                          ::onboard_model.get_policy_hash(),
                                          received_policy_hash);
                            _disable();
                            send_led_message(LED_MSG_POLICY_ERROR);
                        }
                        use_onboard_model = false;
                    }
                } else {
                    if (received_control_mode != CONTROL_MODE_ONBOARD_MODEL) {
                        policy_error_code = POLICY_ERROR_NONE;
                        last_onboard_action = 0.0f;
                    }
                    policy_status_bits = compute_policy_status_bits();
                }

                if (requested_local_debug) {
                    maybe_reset_local_debug_clock(
                        received_control_mode,
                        received_command_context
                    );
                    if (!local_debug_protocol_valid(received_command_context)) {
                        use_local_debug = false;
                    }
                    policy_error_code = POLICY_ERROR_NONE;
                    policy_status_bits = 0;
                } else if (received_control_mode != CONTROL_MODE_LOCAL_DEBUG_SCENARIO) {
                    maybe_reset_local_debug_clock(
                        received_control_mode,
                        received_command_context
                    );
                }

                if (policy_tick) {
                    float frame[LOCAL_FRAME_DIM];
                    float command_context_input[COMMAND_CONTEXT_STORAGE_DIM] = {};

                    float q[4] = {
                        quat_imu[0],
                        quat_imu[1],
                        quat_imu[2],
                        quat_imu[3],
                    };
                    float gyro[3] = {
                        ang_vel_imu[0],
                        ang_vel_imu[1],
                        ang_vel_imu[2],
                    };
                    build_current_local_frame(
                        q,
                        gyro,
                        filtered_dof_pos,
                        filtered_dof_vel,
                        last_onboard_action,
                        frame
                    );

                    if (requested_onboard_model) {
                        for (size_t i = 0; i < COMMAND_CONTEXT_STORAGE_DIM; ++i)
                            command_context_input[i] = received_command_context[i];
                    }

                    push_local_obs_frame(frame);
                    push_command_context(command_context_input);

                    debug_local_obs = build_local_obs();
                    debug_obs_ready = true;

                    for (size_t i = 0; i < COMMAND_CONTEXT_DIM; ++i)
                        debug_command_context[i] = command_context_input[i];

                    policy_debug_valid = 1;
                    policy_debug_seq += 1;
                    policy_debug_nn_action = 0.0f;
                    policy_debug_joint_offset = received_joint_offset;
                    policy_debug_dof_pos = filtered_dof_pos;
                    policy_debug_dof_vel = filtered_dof_vel;
                    for (size_t i = 0; i < LOCAL_DEBUG_COMMAND_CONTEXT_DIM; ++i)
                        policy_debug_command_context[i] = debug_command_context[i];
                    for (size_t i = 0; i < LOCAL_DEBUG_OBS_DIM; ++i)
                        policy_debug_local_obs[i] = debug_local_obs[i];
                }

                if (use_onboard_model || use_local_debug) {
                    // --- Firmware-generated local action mode ---
                    // Run the model every PD_SUBSTEPS ticks, interpolate at 500 Hz
                    if (policy_tick) {
                        if (motor_running && joystick_policy_enabled) {
                            send_led_message(LED_MSG_POLICY_ACTIVE);
                        }
                        float nn_action = 0.0f;
                        if (use_onboard_model && joystick_policy_enabled) {
                            nn_action = ::onboard_model.forward_nn(
                                debug_command_context,
                                debug_local_obs
                            );
                        } else if (use_local_debug) {
                            int joint_id = received_joint_id;
                            if (joint_id < 0) {
                                joint_id = ::onboard_model.get_module_index();
                            }
                            nn_action = compute_local_debug_action(
                                received_command_context,
                                joint_id
                            );
                        }
                        float motor_target = nn_action + received_joint_offset;

                        policy_debug_nn_action = nn_action;
                        policy_debug_motor_target = motor_target;
                        last_onboard_action = nn_action;

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
                    policy_debug_motor_target = curr_policy_target;

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
                    policy_debug_motor_target = target_pos;
                }

                policy_debug_interp_target = interp_pos;

                substep = (substep + 1) % PD_SUBSTEPS;

                // === 4. Apply Butterworth low-pass filter (optional) ===
                float filtered_pos;
                if (enable_filter)
                    filtered_pos = filter.filter(interp_pos);
                else
                    filtered_pos = interp_pos;
                policy_debug_applied_target = filtered_pos;

                // === 5. Send command to motor ===
                StepCommandStatus step_status =
                    _step(filtered_pos, interp_vel, interp_kp, interp_kd);

                // === 6. Update motor error state ===
                motor_error = st.error_state;

                // Check for fault feedback frames actively sent by the motor
                if (_check_fault_frame()) {
                    enqueue(info_queue, 316);
                }
                motor_error2 = fault_state_to_uint32(last_fault);

                // Surface persistent CAN loss as a dedicated bit in driver_error so the
                // dashboard can distinguish "module gone silent" from "module disabled".
                if (motor.consecutive_can_failures >= CAN_LOSS_FAILURE_THRESHOLD) {
                    motor_error2 |= DRIVER_ERROR_CAN_LOST_BIT;
                } else {
                    motor_error2 &= ~DRIVER_ERROR_CAN_LOST_BIT;
                }

                _check_health();
                _check_safety();

                // === 7. Diagnostics (throttled to ~2 Hz to reduce Serial overhead) ===
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
                    const char* step_status_str = "sent";
                    if (step_status == StepCommandStatus::SkipMotorOff) {
                        step_status_str = "skip_motor_off";
                    } else if (step_status == StepCommandStatus::SkipNotCalibrated) {
                        step_status_str = "skip_not_cal";
                    }
                    if (use_onboard_model) {
                        Serial.printf("[Motor/NN] ctrl_mode=%d run=%d lib_cal=%d logical_cal=%d st_mode=%d step=%s f=%.1fHz pos=%.3f target=%.3f offset=%.3f ctx[0]=%.3f hash=%d/%d pstatus=0x%x perr=%d raw_err=0x%02x drv=0x%08x sub=%d dt_us=%lu\n",
                                      received_control_mode,
                                      motor_running ? 1 : 0,
                                      motor.calibrated ? 1 : 0,
                                      motor_calibrated ? 1 : 0,
                                      st.mode,
                                      step_status_str,
                                      loop_freq_filtered, st.angle, filtered_pos,
                                      received_joint_offset, received_command_context[0],
                                      ::onboard_model.get_policy_hash(), received_policy_hash,
                                      policy_status_bits, policy_error_code,
                                      motor_error & 0xFF,
                                      motor_error2,
                                      substep, time_total);
                    } else {
                        Serial.printf("[Motor] ctrl_mode=%d run=%d lib_cal=%d logical_cal=%d st_mode=%d step=%s f=%.1fHz pos=%.3f interp_pos=%.3f vel=%.2f kp=%.1f kd=%.2f hash=%d/%d pstatus=0x%x perr=%d raw_err=0x%02x drv=0x%08x dt_us=%lu\n",
                                      received_control_mode,
                                      motor_running ? 1 : 0,
                                      motor.calibrated ? 1 : 0,
                                      motor_calibrated ? 1 : 0,
                                      st.mode,
                                      step_status_str,
                                      loop_freq_filtered, st.angle, filtered_pos, interp_vel,
                                      interp_kp, interp_kd,
                                      ::onboard_model.get_policy_hash(), received_policy_hash,
                                      policy_status_bits, policy_error_code,
                                      motor_error & 0xFF,
                                      motor_error2,
                                      time_total);
                    }
                }

                // === 8. Sleep until next tick (deterministic timing) ===
                vTaskDelayUntil(&lastWakeTime, dt);
            }
        }
    }  // namespace MotorTask



}
