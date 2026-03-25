#pragma once

#include <vector>
#include <queue>
#include "Arduino.h"
#include <MotorCtrl.hpp>
#include "deploy_config.h"
#include "local_obs_config.h"

#define MOTOR_ID 1

// Control loop timing
#define POLICY_LOOP_HZ      DEPLOY_POLICY_LOOP_HZ
#define PD_LOOP_HZ           500    // PD controller / motor command frequency (Hz)
#define PD_SUBSTEPS         (PD_LOOP_HZ / POLICY_LOOP_HZ)  // PD ticks per policy update

// Legacy aliases (used by filter, torque rate limiter, etc.)
#define CONTROL_LOOP_HZ     PD_LOOP_HZ
#define CONTROL_LOOP_DT_MS  (1000 / PD_LOOP_HZ)    // 2 ms
#define CONTROL_LOOP_DT_S   (1.0f / PD_LOOP_HZ)    // 0.002 s

// Command timeout: if no new command arrives within this time, hold last position
#define COMMAND_TIMEOUT_MS   200   // 200 ms = ~4 missed 20 Hz packets

/**
 * @brief Interpolator for smooth command transitions
 * 
 * Stores two successive command waypoints and linearly interpolates
 * between them. At 20 Hz command rate and 100 Hz control rate, this
 * generates 5 intermediate setpoints per command interval, eliminating
 * step discontinuities that cause torque spikes.
 */
struct CommandInterpolator {
    // Previous command waypoint
    float prev_pos;
    float prev_vel;
    float prev_kp;
    float prev_kd;
    uint64_t prev_time_us;   // microsecond timestamp of previous command

    // Current (latest) command waypoint
    float curr_pos;
    float curr_vel;
    float curr_kp;
    float curr_kd;
    uint64_t curr_time_us;   // microsecond timestamp of current command

    // Interpolation state
    float interp_duration_us; // expected duration between commands (us)
    bool initialized;
    int command_count;        // number of commands received (for startup)

    CommandInterpolator() :
        prev_pos(0), prev_vel(0), prev_kp(0), prev_kd(0), prev_time_us(0),
        curr_pos(0), curr_vel(0), curr_kp(0), curr_kd(0), curr_time_us(0),
        interp_duration_us(50000.0f),  // default 50ms = 20 Hz
        initialized(false), command_count(0) {}

    /**
     * @brief Feed a new command waypoint
     * Called when a new MotorCommand arrives from the network (~20 Hz)
     */
    void pushCommand(float pos, float vel, float kp, float kd) {
        uint64_t now = esp_timer_get_time();

        if (initialized) {
            // Shift current -> previous
            prev_pos = curr_pos;
            prev_vel = curr_vel;
            prev_kp = curr_kp;
            prev_kd = curr_kd;
            prev_time_us = curr_time_us;

            // Estimate command interval with exponential smoothing
            float measured_dt = (float)(now - curr_time_us);
            if (command_count >= 2 && measured_dt > 10000 && measured_dt < 200000) {
                // Smoothly adapt to actual command rate
                interp_duration_us = 0.7f * interp_duration_us + 0.3f * measured_dt;
            }
        } else {
            // First command: initialize both waypoints to same value
            prev_pos = pos;
            prev_vel = vel;
            prev_kp = kp;
            prev_kd = kd;
            prev_time_us = now;
            initialized = true;
        }

        curr_pos = pos;
        curr_vel = vel;
        curr_kp = kp;
        curr_kd = kd;
        curr_time_us = now;
        command_count++;
    }

    /**
     * @brief Get interpolated setpoints for the current control tick
     * 
     * @param[out] out_pos  interpolated target position
     * @param[out] out_vel  interpolated target velocity
     * @param[out] out_kp   gains (held constant, not interpolated)
     * @param[out] out_kd   gains (held constant, not interpolated)
     */
    void sample(float &out_pos, float &out_vel, float &out_kp, float &out_kd) {
        if (!initialized || command_count < 2) {
            // Not enough data to interpolate, use latest command
            out_pos = curr_pos;
            out_vel = curr_vel;
            out_kp = curr_kp;
            out_kd = curr_kd;
            return;
        }

        uint64_t now = esp_timer_get_time();
        float elapsed = (float)(now - curr_time_us);

        // alpha in [0, 1]: 0 = at curr_time, 1 = one full command interval later
        // We extrapolate slightly beyond 1.0 for smoothness, clamped at 1.0
        float alpha = elapsed / interp_duration_us;
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        // Use gains from latest command (no interpolation on gains)
        out_kp = curr_kp;
        out_kd = curr_kd;

        bool is_velocity_mode = (curr_kp == 0.0f && curr_kd != 0.0f);

        if (is_velocity_mode) {
            // Velocity control mode: interpolate velocity, position target doesn't matter
            out_vel = curr_vel + alpha * (curr_vel - prev_vel);
            out_pos = curr_pos;  // not used in velocity mode but pass through
        } else {
            // Position control mode: interpolate position, extrapolate forward
            // Linear extrapolation from current command
            float pos_rate = (curr_pos - prev_pos);  // position delta per command interval
            out_pos = curr_pos + alpha * pos_rate;
            out_vel = curr_vel;  // usually 0 in position mode
        }
    }

    /**
     * @brief Check if commands have timed out
     * @param timeout_ms timeout threshold in milliseconds
     * @return true if the last command is stale
     */
    bool isTimedOut(uint32_t timeout_ms) const {
        if (!initialized) return true;
        uint64_t now = esp_timer_get_time();
        return (now - curr_time_us) > ((uint64_t)timeout_ms * 1000ULL);
    }

    void reset() {
        initialized = false;
        command_count = 0;
        prev_pos = curr_pos = 0;
        prev_vel = curr_vel = 0;
        prev_kp = curr_kp = 0;
        prev_kd = curr_kd = 0;
    }
};


namespace Task {
    constexpr size_t COMMAND_CONTEXT_DIM = LOCAL_LATENT_DIM;
    constexpr size_t COMMAND_CONTEXT_STORAGE_DIM =
        (COMMAND_CONTEXT_DIM > 0) ? COMMAND_CONTEXT_DIM : 1;
    constexpr size_t COMMAND_CONTEXT_HISTORY_STEPS = LOCAL_LATENT_HISTORY_STEPS;
    constexpr size_t POLICY_DEBUG_COMMAND_CONTEXT_STORAGE_DIM =
        (LOCAL_DEBUG_COMMAND_CONTEXT_DIM > 0) ? LOCAL_DEBUG_COMMAND_CONTEXT_DIM : 1;

    enum ControlMode : int32_t {
        CONTROL_MODE_DIRECT_PD = 0,
        CONTROL_MODE_ONBOARD_MODEL = 1,
    };

    enum PolicyStatusBits : int32_t {
        POLICY_STATUS_LOADED = 1 << 0,
        POLICY_STATUS_SANITY_OK = 1 << 1,
        POLICY_STATUS_PC_HASH_SEEN = 1 << 2,
        POLICY_STATUS_HASH_MATCH = 1 << 3,
        POLICY_STATUS_RUNTIME_READY = 1 << 4,
    };

    enum PolicyErrorCode : int32_t {
        POLICY_ERROR_NONE = 0,
        POLICY_ERROR_NOT_LOADED = 1,
        POLICY_ERROR_SANITY_FAILED = 2,
        POLICY_ERROR_NO_PC_HASH = 3,
        POLICY_ERROR_HASH_MISMATCH = 4,
    };

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
    extern int received_control_mode;       // Control mode from last command
    extern float received_joint_offset;     // Per-joint offset from last command
    extern int received_policy_hash;        // Expected policy hash from the PC
    extern int received_joint_id;            // Action/joint index from last command (-1 = all)
    extern float received_command_context[COMMAND_CONTEXT_STORAGE_DIM];  // Updated by the PC command stream

    // Whether the onboard model weights are loaded and available to use.
    extern bool onboard_model_loaded;
    extern int policy_status_bits;
    extern int policy_error_code;
    extern int policy_debug_valid;
    extern int policy_debug_seq;
    extern float policy_debug_nn_action;
    extern float policy_debug_motor_target;
    extern float policy_debug_joint_offset;
    extern float policy_debug_dof_pos;
    extern float policy_debug_dof_vel;
    extern float policy_debug_command_context[POLICY_DEBUG_COMMAND_CONTEXT_STORAGE_DIM];
    extern float policy_debug_local_obs[LOCAL_OBS_DIM];

    // Command interpolator (shared so comm_task can push commands)
    extern CommandInterpolator cmd_interpolator;

    // Config
    extern float offset;
    extern const float DELTA_T;

    namespace MotorTask {
        void run(void *pvParameters);
    }
}
