/**
 * @file MotorCtrl.cpp
 * @brief Motor control implementation for Deep Robotics motors via CAN (TWAI on ESP32)
 *
 * Adapted from Deep Motor SDK to work with ESP32 TWAI driver.
 * Uses standard 11-bit CAN IDs: (cmd << 5) | motor_id
 */
#include "MotorCtrl.hpp"
#include <cmath>

#define _USE_MATH_DEFINES

#define DEBUG_ENABLED 1

#if DEBUG_ENABLED
#define DEBUG_PRINT(c) Serial.print(c)
#else
#define DEBUG_PRINT(c) 0
#endif

// ============================================================================
//  Conversion helpers
// ============================================================================

uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    float clamped = x;
    if (clamped > x_max) clamped = x_max;
    if (clamped < x_min) clamped = x_min;
    return (uint32_t)((clamped - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// ============================================================================
//  Global fault storage (captured during CAN_Transceive)
// ============================================================================

static Motor_fault_state g_last_fault = {};
static bool g_fault_frame_received = false;

// ============================================================================
//  CAN Transceive
// ============================================================================

bool Motor::CAN_Transceive(twai_message_t* TX_msg, twai_message_t* RX_msg)
{
    // Clear RX buffer
    memset(RX_msg, 0, sizeof(twai_message_t));

    if (twai_transmit(TX_msg, pdMS_TO_TICKS(CAN_WAIT_TIME)) != ESP_OK)
    {
        send_led_message(LED_MSG_MOTOR_ERROR);
        DEBUG_PRINT("Failed to transmit CAN message!\n");
        calibrated = false;
        return false;
    }

    if (twai_receive(RX_msg, pdMS_TO_TICKS(CAN_WAIT_TIME)) != ESP_OK)
    {
        DEBUG_PRINT("Failed to receive CAN message\n");
        calibrated = false;
        return false;
    }

    return true;
}

// ============================================================================
//  Pack / Unpack helpers
// ============================================================================

void Motor::PackMotionData(float position, float velocity, float torque,
                           float kp, float kd, uint8_t* data)
{
    // Pack using the SDK's bit-field layout (from FloatsToUints in deep_motor_sdk.h)
    uint16_t _position = (uint16_t)float_to_uint(position, POSITION_MIN, POSITION_MAX, SEND_POSITION_LENGTH);
    uint16_t _velocity = (uint16_t)float_to_uint(velocity, VELOCITY_MIN, VELOCITY_MAX, SEND_VELOCITY_LENGTH);
    uint16_t _torque   = (uint16_t)float_to_uint(torque,   TORQUE_MIN,   TORQUE_MAX,   SEND_TORQUE_LENGTH);
    uint16_t _kp       = (uint16_t)float_to_uint(kp,       KP_MIN,       KP_MAX,       SEND_KP_LENGTH);
    uint16_t _kd       = (uint16_t)float_to_uint(kd,       KD_MIN,       KD_MAX,       SEND_KD_LENGTH);

    // Byte packing matches SDK's FloatsToUints exactly:
    // data[0] = position low byte
    // data[1] = position high byte
    // data[2] = velocity low byte
    // data[3] = velocity[13:8] (6 bits) | kp[1:0] (2 bits)
    // data[4] = kp[9:2] (8 bits)
    // data[5] = kd (8 bits)
    // data[6] = torque low byte
    // data[7] = torque high byte
    data[0] = _position & 0xFF;
    data[1] = (_position >> 8) & 0xFF;
    data[2] = _velocity & 0xFF;
    data[3] = ((_velocity >> 8) & 0x3F) | ((_kp & 0x03) << 6);
    data[4] = (_kp >> 2) & 0xFF;
    data[5] = _kd & 0xFF;
    data[6] = _torque & 0xFF;
    data[7] = (_torque >> 8) & 0xFF;
}

Motor_state Motor::Unpack(const twai_message_t& msg)
{
    Motor_state state = {};

    // Extract motor ID and command from CAN ID
    uint16_t can_id = msg.identifier & 0x7FF;  // 11-bit standard ID
    state.CAN_ID = can_id & 0x1F;              // lower 5 bits = motor_id
    uint8_t cmd = (can_id >> CAN_ID_SHIFT_BITS) & 0x3F;

    if (cmd == CMD_CONTROL_MOTOR && msg.data_length_code >= 8)
    {
        // Unpack motion data using the SDK's bit-field layout
        const ReceivedMotionData* pdata = (const ReceivedMotionData*)msg.data;

        state.angle   = uint_to_float(pdata->position, POSITION_MIN, POSITION_MAX, RECEIVE_POSITION_LENGTH);
        state.angle_v = uint_to_float(pdata->velocity, VELOCITY_MIN, VELOCITY_MAX, RECEIVE_VELOCITY_LENGTH);
        state.torque  = uint_to_float(pdata->torque,   TORQUE_MIN,   TORQUE_MAX,   RECEIVE_TORQUE_LENGTH);
        state.temp_flag = (bool)pdata->temp_flag;

        if (state.temp_flag == kMotorTempFlag) {
            state.temperature = uint_to_float(pdata->temperature, MOTOR_TEMP_MIN, MOTOR_TEMP_MAX, RECEIVE_TEMP_LENGTH);
        } else {
            state.temperature = uint_to_float(pdata->temperature, DRIVER_TEMP_MIN, DRIVER_TEMP_MAX, RECEIVE_TEMP_LENGTH);
        }

        // For backward compatibility: set mode to 2 (normal operation) when we get motion feedback
        state.mode = 2;
        state.error_state = (2 << 6);  // mode=2 in bits 7:6, no errors
    }
    else
    {
        // For non-motion responses, the first data byte is typically a status/ack byte
        if (msg.data_length_code >= 1) {
            state.error_state = msg.data[0];
        }
    }

    return state;
}

void Motor::UnpackError(const twai_message_t& msg, Motor_fault_state* fault)
{
    if (!fault) return;

    memset(fault, 0, sizeof(Motor_fault_state));

    // Extract motor ID from CAN ID
    fault->motor_id = msg.identifier & 0x1F;

    if (msg.data_length_code >= 5)
    {
        // Parse using ReceivedErrorData bit-field union (from SDK)
        const ReceivedErrorData* edata = (const ReceivedErrorData*)msg.data;

        fault->error_code = edata->error_code;

        // Voltage and current are raw uint16; convert to float
        fault->bus_voltage = uint_to_float(edata->voltage, 0.0f, 100.0f, ERROR_VOLTAGE_LENGTH);
        fault->bus_current = uint_to_float(edata->current, CURRENT_MIN, CURRENT_MAX, ERROR_CURRENT_LENGTH);
        fault->motor_temp  = uint_to_float(edata->motor_temp, MOTOR_TEMP_MIN, MOTOR_TEMP_MAX, ERROR_MOTOR_TEMP_LENGTH);
        fault->driver_temp = uint_to_float(edata->driver_temp, DRIVER_TEMP_MIN, DRIVER_TEMP_MAX, ERROR_DRIVER_TEMP_LENGTH);

        // Decode individual error flags
        fault->over_voltage    = (fault->error_code & kOverVoltage)    != 0;
        fault->under_voltage   = (fault->error_code & kUnderVoltage)   != 0;
        fault->over_current    = (fault->error_code & kOverCurrent)    != 0;
        fault->motor_over_temp = (fault->error_code & kMotorOverTemp)  != 0;
        fault->driver_over_temp= (fault->error_code & kDriverOverTemp) != 0;
        fault->can_timeout     = (fault->error_code & kCanTimeout)     != 0;

        // Set fault_flag for backward compat
        fault->fault_flag = (fault->error_code != 0) ? 1 : 0;
    }
}

// ============================================================================
//  Send a simple command (no data payload)
// ============================================================================

bool Motor::SendSimpleCommand(uint8_t cmd, uint8_t send_dlc, twai_message_t* rx_msg)
{
    twai_message_t tx_msg = {};
    tx_msg.identifier = FormCanId(cmd, CAN_ID);
    tx_msg.flags = 0;  // Standard frame (11-bit ID), NOT extended
    tx_msg.data_length_code = send_dlc;
    memset(tx_msg.data, 0, 8);

    return CAN_Transceive(&tx_msg, rx_msg);
}

// ============================================================================
//  Init / Uninit
// ============================================================================

uint64_t Motor::Init(const uint8_t Target_ID)
{
    return Init(Target_ID, GPIO_NUM_1, GPIO_NUM_2);
}

uint64_t Motor::Init(const uint8_t Target_ID, gpio_num_t can_tx_pin, gpio_num_t can_rx_pin)
{
    if (CAN_inited)
    {
        DEBUG_PRINT("Driver already installed\n");
    }
    else
    {
        // Initialize configuration structures
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(can_tx_pin, can_rx_pin, TWAI_MODE_NORMAL);
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        // Install CAN driver
        if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
        {
            DEBUG_PRINT("Driver installed\n");
        }
        else
        {
            DEBUG_PRINT("Failed to install driver\n");
            return 0;
        }

        // Start CAN driver
        if (twai_start() == ESP_OK)
        {
            DEBUG_PRINT("Driver started\n");
        }
        else
        {
            DEBUG_PRINT("Failed to start driver\n");
            return 0;
        }

        CAN_inited = true;
    }

    CAN_ID = Target_ID;

    // Return 1 to indicate success (Deep Motor doesn't have MCU ID query like CyberGear)
    return 1;
}

void Motor::Uninit()
{
    if (twai_stop() == ESP_OK)
    {
        DEBUG_PRINT("Driver stopped\n");
    }
    else
    {
        DEBUG_PRINT("Failed to stop driver\n");
        return;
    }

    if (twai_driver_uninstall() == ESP_OK)
    {
        DEBUG_PRINT("Driver uninstalled\n");
    }
    else
    {
        DEBUG_PRINT("Failed to uninstall driver\n");
        return;
    }

    CAN_inited = false;
}

// ============================================================================
//  Constructors / Destructor
// ============================================================================

Motor::Motor() {}

Motor::Motor(uint8_t Target_ID)
{
    Init(Target_ID);
}

Motor::~Motor()
{
    Uninit();
}

// ============================================================================
//  Basic commands
// ============================================================================

Motor_state Motor::Enable()
{
    twai_message_t rx_msg;

    if (SendSimpleCommand(CMD_ENABLE_MOTOR, SEND_DLC_ENABLE_MOTOR, &rx_msg))
    {
        Serial.printf("[Motor] Enable success (motor %d)\n", CAN_ID);
        last_state = Unpack(rx_msg);
        // Mark as enabled: set mode bits in error_state for backward compat
        last_state.mode = 2;
        last_state.error_state = (2 << 6);  // mode=2 (normal operation)
        return last_state;
    }
    else
    {
        Serial.printf("[Motor] Enable failed (motor %d)\n", CAN_ID);
        return Motor_state{};
    }
}

Motor_state Motor::Disable(const bool clear_error)
{
    if (clear_error)
    {
        // Send error reset first
        Error_Reset();
    }

    twai_message_t rx_msg;

    if (SendSimpleCommand(CMD_DISABLE_MOTOR, SEND_DLC_DISABLE_MOTOR, &rx_msg))
    {
        Serial.printf("[Motor] Disable success (motor %d)\n", CAN_ID);
        last_state = Unpack(rx_msg);
        last_state.mode = 0;
        last_state.error_state = 0;  // mode=0 (disabled)
        return last_state;
    }
    else
    {
        Serial.printf("[Motor] Disable failed (motor %d)\n", CAN_ID);
        return Motor_state{};
    }
}

Motor_state Motor::Set_zero()
{
    twai_message_t rx_msg;

    if (SendSimpleCommand(CMD_SET_HOME, SEND_DLC_SET_HOME, &rx_msg))
    {
        Serial.printf("[Motor] Set zero success (motor %d)\n", CAN_ID);
        return Unpack(rx_msg);
    }
    else
    {
        Serial.printf("[Motor] Set zero failed (motor %d)\n", CAN_ID);
        return Motor_state{};
    }
}

Motor_state Motor::Error_Reset()
{
    twai_message_t rx_msg;

    if (SendSimpleCommand(CMD_ERROR_RESET, SEND_DLC_ERROR_RESET, &rx_msg))
    {
        Serial.printf("[Motor] Error reset success (motor %d)\n", CAN_ID);
        return Unpack(rx_msg);
    }
    else
    {
        Serial.printf("[Motor] Error reset failed (motor %d)\n", CAN_ID);
        return Motor_state{};
    }
}

Motor_state Motor::Save_Config()
{
    twai_message_t rx_msg;

    if (SendSimpleCommand(CMD_SAVE_CONFIG, SEND_DLC_SAVE_CONFIG, &rx_msg))
    {
        Serial.printf("[Motor] Save config success (motor %d)\n", CAN_ID);
        return Unpack(rx_msg);
    }
    else
    {
        Serial.printf("[Motor] Save config failed (motor %d)\n", CAN_ID);
        return Motor_state{};
    }
}

// ============================================================================
//  Set CAN ID
// ============================================================================

uint64_t Motor::Set_CAN_ID(const uint8_t New_ID)
{
    twai_message_t tx_msg = {};
    tx_msg.identifier = FormCanId(CMD_SET_ID, CAN_ID);
    tx_msg.flags = 0;  // Standard frame
    tx_msg.data_length_code = SEND_DLC_SET_ID;
    memset(tx_msg.data, 0, 8);
    tx_msg.data[0] = New_ID;

    twai_message_t rx_msg;

    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        uint8_t confirmed_id = rx_msg.data[0];
        Serial.printf("[Motor] Set CAN ID: %d -> %d (confirmed: %d)\n", CAN_ID, New_ID, confirmed_id);
        CAN_ID = New_ID;
        return New_ID;
    }
    else
    {
        Serial.printf("[Motor] Set CAN ID failed\n");
        return 0;
    }
}

// ============================================================================
//  Motion Control (MIT-style)
// ============================================================================

Motor_state Motor::Set_control(const float target_torque, const float target_angle,
                               const float target_vel, const float Kp, const float Kd)
{
    twai_message_t tx_msg = {};
    tx_msg.identifier = FormCanId(CMD_CONTROL_MOTOR, CAN_ID);
    tx_msg.flags = 0;  // Standard frame
    tx_msg.data_length_code = SEND_DLC_CONTROL_MOTOR;

    PackMotionData(target_angle, target_vel, target_torque, Kp, Kd, tx_msg.data);

    twai_message_t rx_msg;

    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        last_state = Unpack(rx_msg);
        return last_state;
    }
    else
    {
        return Motor_state{};
    }
}

// ============================================================================
//  Configuration commands (2-byte payload)
// ============================================================================

bool Motor::Set_gear_ratio(float ratio)
{
    uint16_t val = (uint16_t)float_to_uint(ratio, GEAR_RATIO_MIN, GEAR_RATIO_MAX, SEND_GEAR_RATIO_LENGTH);

    twai_message_t tx_msg = {};
    tx_msg.identifier = FormCanId(CMD_SET_GEAR, CAN_ID);
    tx_msg.flags = 0;
    tx_msg.data_length_code = SEND_DLC_SET_GEAR;
    memset(tx_msg.data, 0, 8);
    tx_msg.data[0] = val & 0xFF;
    tx_msg.data[1] = (val >> 8) & 0xFF;

    twai_message_t rx_msg;
    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        Serial.printf("[Motor] Set gear ratio = %.2f (motor %d)\n", ratio, CAN_ID);
        return true;
    }
    return false;
}

bool Motor::Set_limit_current(float current_limit)
{
    uint16_t val = (uint16_t)float_to_uint(current_limit, CURRENT_MIN, CURRENT_MAX, SEND_LIMIT_CURRENT_LENGTH);

    twai_message_t tx_msg = {};
    tx_msg.identifier = FormCanId(CMD_SET_LIMIT_CURRENT, CAN_ID);
    tx_msg.flags = 0;
    tx_msg.data_length_code = SEND_DLC_SET_LIMIT_CURRENT;
    memset(tx_msg.data, 0, 8);
    tx_msg.data[0] = val & 0xFF;
    tx_msg.data[1] = (val >> 8) & 0xFF;

    twai_message_t rx_msg;
    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        Serial.printf("[Motor] Set limit current = %.2f A (motor %d)\n", current_limit, CAN_ID);
        limit_current_val = current_limit;
        return true;
    }
    return false;
}

bool Motor::Set_CAN_timeout(uint8_t timeout_ms)
{
    twai_message_t tx_msg = {};
    tx_msg.identifier = FormCanId(CMD_SET_CAN_TIMEOUT, CAN_ID);
    tx_msg.flags = 0;
    tx_msg.data_length_code = SEND_DLC_SET_CAN_TIMEOUT;
    memset(tx_msg.data, 0, 8);
    tx_msg.data[0] = timeout_ms;

    twai_message_t rx_msg;
    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        Serial.printf("[Motor] Set CAN timeout = %d ms (motor %d)\n", timeout_ms, CAN_ID);
        return true;
    }
    return false;
}

bool Motor::Set_bandwidth(uint16_t bandwidth)
{
    twai_message_t tx_msg = {};
    tx_msg.identifier = FormCanId(CMD_SET_BANDWIDTH, CAN_ID);
    tx_msg.flags = 0;
    tx_msg.data_length_code = SEND_DLC_SET_BANDWIDTH;
    memset(tx_msg.data, 0, 8);
    tx_msg.data[0] = bandwidth & 0xFF;
    tx_msg.data[1] = (bandwidth >> 8) & 0xFF;

    twai_message_t rx_msg;
    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        Serial.printf("[Motor] Set bandwidth = %d (motor %d)\n", bandwidth, CAN_ID);
        return true;
    }
    return false;
}

bool Motor::Set_under_voltage(float voltage)
{
    uint16_t val = (uint16_t)(voltage * 100.0f);

    twai_message_t tx_msg = {};
    tx_msg.identifier = FormCanId(CMD_SET_UNDER_VOLTAGE, CAN_ID);
    tx_msg.flags = 0;
    tx_msg.data_length_code = SEND_DLC_SET_UNDER_VOLTAGE;
    memset(tx_msg.data, 0, 8);
    tx_msg.data[0] = val & 0xFF;
    tx_msg.data[1] = (val >> 8) & 0xFF;

    twai_message_t rx_msg;
    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        Serial.printf("[Motor] Set under voltage = %.2f V (motor %d)\n", voltage, CAN_ID);
        return true;
    }
    return false;
}

bool Motor::Set_over_voltage(float voltage)
{
    uint16_t val = (uint16_t)(voltage * 100.0f);

    twai_message_t tx_msg = {};
    tx_msg.identifier = FormCanId(CMD_SET_OVER_VOLTAGE, CAN_ID);
    tx_msg.flags = 0;
    tx_msg.data_length_code = SEND_DLC_SET_OVER_VOLTAGE;
    memset(tx_msg.data, 0, 8);
    tx_msg.data[0] = val & 0xFF;
    tx_msg.data[1] = (val >> 8) & 0xFF;

    twai_message_t rx_msg;
    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        Serial.printf("[Motor] Set over voltage = %.2f V (motor %d)\n", voltage, CAN_ID);
        return true;
    }
    return false;
}

bool Motor::Set_motor_temperature(float temp)
{
    uint16_t val = (uint16_t)(temp * 10.0f);

    twai_message_t tx_msg = {};
    tx_msg.identifier = FormCanId(CMD_SET_MOTOR_TEMPERATURE, CAN_ID);
    tx_msg.flags = 0;
    tx_msg.data_length_code = SEND_DLC_SET_MOTOR_TEMPERATURE;
    memset(tx_msg.data, 0, 8);
    tx_msg.data[0] = val & 0xFF;
    tx_msg.data[1] = (val >> 8) & 0xFF;

    twai_message_t rx_msg;
    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        Serial.printf("[Motor] Set motor temp limit = %.1f C (motor %d)\n", temp, CAN_ID);
        return true;
    }
    return false;
}

bool Motor::Set_drive_temperature(float temp)
{
    uint16_t val = (uint16_t)(temp * 10.0f);

    twai_message_t tx_msg = {};
    tx_msg.identifier = FormCanId(CMD_SET_DRIVE_TEMPERATURE, CAN_ID);
    tx_msg.flags = 0;
    tx_msg.data_length_code = SEND_DLC_SET_DRIVE_TEMPERATURE;
    memset(tx_msg.data, 0, 8);
    tx_msg.data[0] = val & 0xFF;
    tx_msg.data[1] = (val >> 8) & 0xFF;

    twai_message_t rx_msg;
    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        Serial.printf("[Motor] Set drive temp limit = %.1f C (motor %d)\n", temp, CAN_ID);
        return true;
    }
    return false;
}

// ============================================================================
//  Status queries
// ============================================================================

Motor_state Motor::Get_state()
{
    // Use a zero-torque control command to get feedback without moving
    return Set_control(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

uint16_t Motor::Get_fw_version()
{
    twai_message_t rx_msg;

    if (SendSimpleCommand(CMD_GET_FW_VERSION, SEND_DLC_GET_FW_VERSION, &rx_msg))
    {
        uint16_t version = (uint16_t(rx_msg.data[1]) << 8) | rx_msg.data[0];
        Serial.printf("[Motor] FW version: %d.%d (motor %d)\n",
                      rx_msg.data[1], rx_msg.data[0], CAN_ID);
        return version;
    }
    return 0;
}

Motor_fault_state Motor::Get_status_word()
{
    Motor_fault_state fault = {};

    twai_message_t rx_msg;

    if (SendSimpleCommand(CMD_GET_STATUS_WORD, SEND_DLC_GET_STATUS_WORD, &rx_msg))
    {
        UnpackError(rx_msg, &fault);
        fault.motor_id = CAN_ID;

        // Log errors if any
        if (fault.error_code != kMotorNoError)
        {
            if (fault.over_voltage)     Serial.printf("[Motor %d] ERROR: Over voltage\n", CAN_ID);
            if (fault.under_voltage)    Serial.printf("[Motor %d] ERROR: Under voltage\n", CAN_ID);
            if (fault.over_current)     Serial.printf("[Motor %d] ERROR: Over current\n", CAN_ID);
            if (fault.motor_over_temp)  Serial.printf("[Motor %d] ERROR: Motor over temp\n", CAN_ID);
            if (fault.driver_over_temp) Serial.printf("[Motor %d] ERROR: Driver over temp\n", CAN_ID);
            if (fault.can_timeout)      Serial.printf("[Motor %d] ERROR: CAN timeout\n", CAN_ID);
        }
    }

    last_fault = fault;
    return fault;
}

bool Motor::Check_Fault_Frame(Motor_fault_state* fault_out)
{
    // Check if we captured a fault during a previous CAN_Transceive
    if (g_fault_frame_received)
    {
        if (fault_out) *fault_out = g_last_fault;
        g_fault_frame_received = false;
        return true;
    }

    // Actively query status word
    Motor_fault_state fault = Get_status_word();

    if (fault.error_code != kMotorNoError)
    {
        if (fault_out) *fault_out = fault;
        return true;
    }

    return false;
}

// ============================================================================
//  Convenience wrappers (backward-compatible)
// ============================================================================

Motor_state Motor::Set_mode(const Motor_mode mode)
{
    // Deep Motor doesn't have explicit mode switching.
    // Mode is implicit in how Set_control is called.
    // We just store the mode for reference.
    curr_mode = mode;
    return last_state;
}

Motor_state Motor::Set_position(const float target_angle)
{
    // Emulate position mode: high Kp, moderate Kd, no feedforward torque
    curr_mode = Motor_mode::Position;
    return Set_control(0.0f, target_angle, 0.0f, pos_kp, pos_kd);
}

Motor_state Motor::Set_velocity(const float target_vel)
{
    // Emulate velocity mode: only Kd, no position target
    curr_mode = Motor_mode::Velocity;
    return Set_control(0.0f, 0.0f, target_vel, 0.0f, vel_kd);
}

Motor_state Motor::Set_current(const float target_current)
{
    // Emulate current/torque mode: only torque, no position/velocity control
    curr_mode = Motor_mode::Current;
    return Set_control(target_current, 0.0f, 0.0f, 0.0f, 0.0f);
}

float Motor::get_voltage()
{
    Motor_fault_state fault = Get_status_word();
    return fault.bus_voltage;
}

uint32_t Motor::get_error()
{
    Motor_fault_state fault = Get_status_word();
    return (uint32_t)fault.error_code;
}

// ============================================================================
//  ButterworthFilter (unchanged)
// ============================================================================

ButterworthFilter::ButterworthFilter(double cutoffFreq, double samplingRate)
{
    setCutoffFrequency(cutoffFreq, samplingRate);
    reset();
}

void ButterworthFilter::setCutoffFrequency(double cutoffFreq, double samplingRate)
{
    double nyquist = 0.5 * samplingRate;
    double normalizedCutoff = cutoffFreq / nyquist;
    double K = std::tan(M_PI * normalizedCutoff);
    double K_squared = K * K;

    double norm = 1.0 / (1.0 + std::sqrt(2.0) * K + K_squared);
    a0 = K_squared * norm;
    a1 = 2.0 * a0;
    a2 = a0;
    b1 = 2.0 * (K_squared - 1.0) * norm;
    b2 = (1.0 - std::sqrt(2.0) * K + K_squared) * norm;
}

void ButterworthFilter::reset()
{
    x1 = x2 = 0.0;
    y1 = y2 = 0.0;
}

double ButterworthFilter::filter(double input)
{
    double output = a0 * input + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;
    return output;
}
