/**
 * @file MotorCtrl.hpp
 * @brief Motor control library for Deep Robotics motors via CAN (TWAI on ESP32)
 *
 * Adapted from the Deep Motor SDK (can_protocol.h / deep_motor_sdk.h)
 * to work on ESP32 using the TWAI driver instead of Linux SocketCAN.
 *
 * CAN ID format: standard 11-bit, (cmd << 5) | motor_id
 *
 * @note You are responsible for keeping the state of the motor!
 */
#ifndef MOTORCTRL_HPP__
#define MOTORCTRL_HPP__

#include "Arduino.h"
#include <cstdint>
#include <cstring>
#include "driver/gpio.h"
#include "hal/twai_types.h"
#include "driver/twai.h"
#include "tasks/utils.hpp"

// ============================================================================
//  Deep Motor CAN Protocol Constants (from can_protocol.h)
// ============================================================================

// CAN ID formation: (cmd << CAN_ID_SHIFT_BITS) | motor_id
#define CAN_ID_SHIFT_BITS 5

// --- Value ranges for float <-> uint conversion ---
#define POSITION_MIN -40.0f
#define POSITION_MAX  40.0f
#define VELOCITY_MIN -40.0f
#define VELOCITY_MAX  40.0f
#define KP_MIN        0.0f
#define KP_MAX     1023.0f
#define KD_MIN        0.0f
#define KD_MAX       51.0f
#define TORQUE_MIN  -40.0f
#define TORQUE_MAX   40.0f

#define GEAR_RATIO_MIN 0.0f
#define GEAR_RATIO_MAX 50.0f

#define MOTOR_TEMP_MIN  -20.0f
#define MOTOR_TEMP_MAX  200.0f
#define DRIVER_TEMP_MIN -20.0f
#define DRIVER_TEMP_MAX 200.0f
#define CURRENT_MIN      0.0f
#define CURRENT_MAX     40.0f

// --- Bit-field widths for packing ---
#define SEND_POSITION_LENGTH    16
#define SEND_VELOCITY_LENGTH    14
#define SEND_KP_LENGTH          10
#define SEND_KD_LENGTH           8
#define SEND_TORQUE_LENGTH      16
#define SEND_GEAR_RATIO_LENGTH  16
#define SEND_LIMIT_CURRENT_LENGTH 16

#define RECEIVE_POSITION_LENGTH  20
#define RECEIVE_VELOCITY_LENGTH  20
#define RECEIVE_TORQUE_LENGTH    16
#define RECEIVE_TEMP_FLAG_LENGTH  1
#define RECEIVE_TEMP_LENGTH       7

#define ERROR_CODE_LENGTH    16
#define ERROR_VOLTAGE_LENGTH 16
#define ERROR_CURRENT_LENGTH 16
#define ERROR_MOTOR_TEMP_LENGTH  8
#define ERROR_DRIVER_TEMP_LENGTH 8

// --- Command IDs ---
enum DeepMotorCmd : uint8_t {
    CMD_DISABLE_MOTOR         = 1,
    CMD_ENABLE_MOTOR          = 2,
    CMD_CALIBRATE_START       = 3,
    CMD_CONTROL_MOTOR         = 4,
    CMD_RESET_MOTOR           = 5,
    CMD_SET_HOME              = 6,
    CMD_SET_GEAR              = 7,
    CMD_SET_ID                = 8,
    CMD_SET_CAN_TIMEOUT       = 9,
    CMD_SET_BANDWIDTH         = 10,
    CMD_SET_LIMIT_CURRENT     = 11,
    CMD_SET_UNDER_VOLTAGE     = 12,
    CMD_SET_OVER_VOLTAGE      = 13,
    CMD_SET_MOTOR_TEMPERATURE = 14,
    CMD_SET_DRIVE_TEMPERATURE = 15,
    CMD_SAVE_CONFIG           = 16,
    CMD_ERROR_RESET           = 17,
    CMD_GET_FW_VERSION        = 22,
    CMD_GET_STATUS_WORD       = 23,
    CMD_GET_CONFIG            = 24,
};

// --- Send DLC (Data Length Code) per command ---
#define SEND_DLC_DISABLE_MOTOR     0
#define SEND_DLC_ENABLE_MOTOR      0
#define SEND_DLC_CALIBRATE_START   0
#define SEND_DLC_CONTROL_MOTOR     8
#define SEND_DLC_RESET_MOTOR       0
#define SEND_DLC_SET_HOME          0
#define SEND_DLC_SET_GEAR          2
#define SEND_DLC_SET_ID            1
#define SEND_DLC_SET_CAN_TIMEOUT   1
#define SEND_DLC_SET_BANDWIDTH     2
#define SEND_DLC_SET_LIMIT_CURRENT 2
#define SEND_DLC_SET_UNDER_VOLTAGE 2
#define SEND_DLC_SET_OVER_VOLTAGE  2
#define SEND_DLC_SET_MOTOR_TEMPERATURE 2
#define SEND_DLC_SET_DRIVE_TEMPERATURE 2
#define SEND_DLC_SAVE_CONFIG       0
#define SEND_DLC_ERROR_RESET       0
#define SEND_DLC_GET_FW_VERSION    0
#define SEND_DLC_GET_STATUS_WORD   0
#define SEND_DLC_GET_CONFIG        0

// --- Receive DLC per command ---
#define RECEIVE_DLC_DISABLE_MOTOR    1
#define RECEIVE_DLC_ENABLE_MOTOR     1
#define RECEIVE_DLC_CALIBRATE_START  1
#define RECEIVE_DLC_CONTROL_MOTOR    8
#define RECEIVE_DLC_RESET_MOTOR      1
#define RECEIVE_DLC_SET_HOME         1
#define RECEIVE_DLC_SET_GEAR         2
#define RECEIVE_DLC_SET_ID           1
#define RECEIVE_DLC_SET_CAN_TIMEOUT  1
#define RECEIVE_DLC_SET_BANDWIDTH    1
#define RECEIVE_DLC_SET_LIMIT_CURRENT  1
#define RECEIVE_DLC_SET_UNDER_VOLTAGE  1
#define RECEIVE_DLC_SET_OVER_VOLTAGE   1
#define RECEIVE_DLC_SET_MOTOR_TEMPERATURE 1
#define RECEIVE_DLC_SET_DRIVE_TEMPERATURE 1
#define RECEIVE_DLC_SAVE_CONFIG      1
#define RECEIVE_DLC_ERROR_RESET      1
#define RECEIVE_DLC_GET_FW_VERSION   2
#define RECEIVE_DLC_GET_STATUS_WORD  5
#define RECEIVE_DLC_GET_CONFIG       8

// ============================================================================
//  Bit-field unions for packing/unpacking CAN data (from can_protocol.h)
// ============================================================================

#pragma pack(push, 1)
typedef union SendMotionData {
    uint8_t data[8];
    struct {
        uint32_t position : SEND_POSITION_LENGTH;     // 16 bits
        uint32_t velocity : SEND_VELOCITY_LENGTH;     // 14 bits
        uint32_t kp       : SEND_KP_LENGTH;           // 10 bits
        uint32_t kd       : SEND_KD_LENGTH;           //  8 bits
        uint32_t torque   : SEND_TORQUE_LENGTH;       // 16 bits
    };
} SendMotionData;

typedef union ReceivedMotionData {
    uint8_t data[8];
    struct {
        uint32_t position  : RECEIVE_POSITION_LENGTH;   // 20 bits
        uint32_t velocity  : RECEIVE_VELOCITY_LENGTH;   // 20 bits
        uint32_t torque    : RECEIVE_TORQUE_LENGTH;     // 16 bits
        uint32_t temp_flag : RECEIVE_TEMP_FLAG_LENGTH;  //  1 bit
        uint32_t temperature : RECEIVE_TEMP_LENGTH;     //  7 bits
    };
} ReceivedMotionData;

typedef union ReceivedErrorData {
    uint8_t data[8];
    struct {
        uint16_t error_code  : ERROR_CODE_LENGTH;       // 16 bits
        uint16_t voltage     : ERROR_VOLTAGE_LENGTH;    // 16 bits
        uint16_t current     : ERROR_CURRENT_LENGTH;    // 16 bits
        uint8_t  motor_temp  : ERROR_MOTOR_TEMP_LENGTH; //  8 bits
        uint8_t  driver_temp : ERROR_DRIVER_TEMP_LENGTH;//  8 bits
    };
} ReceivedErrorData;
#pragma pack(pop)

enum TempFlag {
    kDriverTempFlag = 0,
    kMotorTempFlag  = 1
};

// ============================================================================
//  Motor error flags (from MotorErrorType in SDK)
// ============================================================================
enum MotorErrorType : uint16_t {
    kMotorNoError   = 0,
    kOverVoltage    = (0x01 << 0),
    kUnderVoltage   = (0x01 << 1),
    kOverCurrent    = (0x01 << 2),
    kMotorOverTemp  = (0x01 << 3),
    kDriverOverTemp = (0x01 << 4),
    kCanTimeout     = (0x01 << 5),
};

// ============================================================================
//  Motor mode enum - kept for backward compatibility with motor_task
// ============================================================================
enum Motor_mode {
    Motion = 0,   // MIT-style motion control (Set_control)
    Position,     // Position control (emulated via Set_control with high Kp)
    Velocity,     // Velocity control (emulated via Set_control with Kd only)
    Current       // Current/torque control (emulated via Set_control torque only)
};

// ============================================================================
//  Default config
// ============================================================================

#define MOTOR_TRIG_PIN 15

// how many ms should CAN wait for transmission / reception
constexpr uint32_t CAN_WAIT_TIME = 10;

// ============================================================================
//  Motor_state - returned feedback from motion commands
// ============================================================================

/**
 * @brief struct to store motor state response (mapped from Deep Motor feedback)
 *
 * Field mapping (backward-compatible with old code):
 *   CAN_ID      -> extracted from received CAN frame ID
 *   error_state -> lower 6 bits = error flags, bits 7:6 = mode (2=normal, 0=off)
 *   mode        -> run mode
 *   angle       -> position in rad (from ReceivedMotionData, range +/-40 rad)
 *   angle_v     -> velocity in rad/s (range +/-40 rad/s)
 *   torque      -> torque in N.m (range +/-40 N.m)
 *   temperature -> degrees C (motor or driver depending on temp_flag)
 */
struct Motor_state
{
    uint8_t CAN_ID;      // Motor CAN ID (from received frame)
    uint8_t error_state; // Error/status packed: lower 6 bits = errors, upper 2 = mode
    uint8_t mode;        // Operating mode

    float angle;         // Position in rad
    float angle_v;       // Velocity in rad/s
    float torque;        // Torque in N.m
    float temperature;   // Temperature in degrees C

    bool temp_flag;      // false = driver temp, true = motor temp
};

/**
 * @brief struct to store error/fault status from GET_STATUS_WORD
 *
 * Mapped from ReceivedErrorData:
 *   error_code  -> 16-bit bitfield (kOverVoltage, kUnderVoltage, etc.)
 *   voltage     -> bus voltage in V (converted from uint)
 *   current     -> bus current in A
 *   motor_temp  -> motor temperature in degrees C
 *   driver_temp -> driver board temperature in degrees C
 */
struct Motor_fault_state
{
    uint16_t motor_id;
    uint16_t error_code;     // 16-bit error code bitfield (MotorErrorType)
    float bus_voltage;       // Bus voltage (V)
    float bus_current;       // Bus current (A)
    float motor_temp;        // Motor temperature (degrees C)
    float driver_temp;       // Driver temperature (degrees C)

    // Decoded individual error flags
    bool over_voltage;
    bool under_voltage;
    bool over_current;
    bool motor_over_temp;
    bool driver_over_temp;
    bool can_timeout;

    // Kept for backward compatibility
    uint8_t fault_flag;
    bool encoder_not_calibrated;
    bool phase_a_overflow;
    bool phase_b_overflow;
    bool phase_c_overflow;
    bool driver_chip_fault;
    uint8_t overload_fault;
    bool temp_warning;
    uint8_t warning_byte4;
    uint8_t warning_byte5;
    uint8_t warning_byte6;
    uint8_t warning_byte7;
};

// ============================================================================
//  Conversion helpers
// ============================================================================

uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
float uint_to_float(int x_int, float x_min, float x_max, uint8_t bits);

// ============================================================================
//  Motor class
// ============================================================================

class Motor
{
public:
    Motor();
    Motor(const uint8_t Target_ID);
    ~Motor();

    /**
     * @brief Initialize motor CAN bus
     * @param Target_ID CAN ID of the target motor (1-31)
     * @return non-zero on success (returns 1), 0 on failure
     */
    uint64_t Init(const uint8_t Target_ID);
    uint64_t Init(const uint8_t Target_ID, gpio_num_t can_tx_pin, gpio_num_t can_rx_pin);

    void Uninit();

    // ---- Basic commands ----
    Motor_state Enable();
    Motor_state Disable(const bool clear_error = false);
    Motor_state Set_zero();
    Motor_state Error_Reset();
    Motor_state Save_Config();

    // ---- ID management ----
    uint64_t Set_CAN_ID(const uint8_t New_ID);

    // ---- Motion control (MIT-style) ----
    /**
     * @brief Send motion control command
     *
     * @param target_torque  -40~40 N.m
     * @param target_angle   -40~40 rad
     * @param target_vel     -40~40 rad/s
     * @param Kp              0~1023
     * @param Kd              0~51
     * @return Motor_state feedback
     *
     * The motor computes:
     *   torque_out = Kp*(target_angle - angle) + Kd*(target_vel - vel) + target_torque
     */
    Motor_state Set_control(const float target_torque,
                            const float target_angle = 0.0f,
                            const float target_vel   = 0.0f,
                            const float Kp           = 0.0f,
                            const float Kd           = 0.0f);

    // ---- Configuration commands ----
    bool Set_gear_ratio(float ratio);
    bool Set_limit_current(float current_limit);
    bool Set_CAN_timeout(uint8_t timeout_ms);
    bool Set_bandwidth(uint16_t bandwidth);
    bool Set_under_voltage(float voltage);
    bool Set_over_voltage(float voltage);
    bool Set_motor_temperature(float temp);
    bool Set_drive_temperature(float temp);

    // ---- Status queries ----
    Motor_state Get_state();
    bool Check_Fault_Frame(Motor_fault_state* fault_out);
    uint16_t Get_fw_version();
    Motor_fault_state Get_status_word();

    // ---- Convenience wrappers (backward-compatible with old interface) ----
    /**
     * @brief Emulate position mode using Set_control with high Kp
     * @param target_angle target angle in rad (range: -40 ~ +40)
     */
    Motor_state Set_position(const float target_angle);

    /**
     * @brief Emulate velocity mode using Set_control with Kd
     * @param target_vel target velocity in rad/s (range: -40 ~ +40)
     */
    Motor_state Set_velocity(const float target_vel);

    /**
     * @brief Emulate current/torque mode using Set_control
     * @param target_current target torque in N.m (range: -40 ~ +40)
     */
    Motor_state Set_current(const float target_current);

    /**
     * @brief Set motor mode - for backward compatibility (no-op in Deep Motor,
     *        mode is implicit in how Set_control is called)
     */
    Motor_state Set_mode(const Motor_mode mode);

    float get_voltage();
    uint32_t get_error();

    bool calibrated = false;

private:
    uint8_t CAN_ID = 1;         // Motor CAN ID (default 1 for Deep Motor)
    bool CAN_inited = false;
    Motor_state last_state = {};
    Motor_mode curr_mode = Motor_mode::Motion;
    Motor_fault_state last_fault = {};
    bool fault_received = false;

    // Position/velocity control gains for convenience wrappers
    float pos_kp = 50.0f;
    float pos_kd = 2.0f;
    float vel_kd = 5.0f;
    float limit_current_val = 23.0f;

    // Form standard 11-bit CAN ID from command + motor_id
    static uint16_t FormCanId(uint8_t cmd, uint8_t motor_id) {
        return (uint16_t)(cmd << CAN_ID_SHIFT_BITS) | motor_id;
    }

    // Pack motion floats into CAN data bytes using the SDK bit-packing
    static void PackMotionData(float position, float velocity, float torque,
                               float kp, float kd, uint8_t* data);

    // Unpack received CAN data into Motor_state
    Motor_state Unpack(const twai_message_t& msg);

    // Unpack error/status word data
    void UnpackError(const twai_message_t& msg, Motor_fault_state* fault);

    // Send a simple command (no data payload) and receive response
    bool SendSimpleCommand(uint8_t cmd, uint8_t send_dlc, twai_message_t* rx_msg);

    // CAN transceive with timeout
    bool CAN_Transceive(twai_message_t* TX_msg, twai_message_t* RX_msg);
};

// ============================================================================
//  ButterworthFilter (unchanged from old library)
// ============================================================================

class ButterworthFilter {
public:
    ButterworthFilter(double cutoffFreq, double samplingRate);
    void setCutoffFrequency(double cutoffFreq, double samplingRate);
    void reset();
    double filter(double input);

private:
    double a0, a1, a2;
    double b1, b2;
    double x1, x2;
    double y1, y2;
};

#endif // MOTORCTRL_HPP__
