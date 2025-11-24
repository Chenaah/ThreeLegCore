/**
 * @file MotorCtrl.hpp
 * @brief Motor control functions
 *
 * @note You are responsible for keeping the state of the motor! I don't want to deal with concurrency issues!
 *
 * @warning alert messages has been disabled! we do not check for that in this library
 */
#ifndef MOTORCTRL_HPP__
#define MOTORCTRL_HPP__

#include "Arduino.h"
#include <cstdint>
#include "driver/gpio.h"
// #include "driver/can.h"
#include "hal/twai_types.h"
#include "driver/twai.h"
#include "tasks/utils.hpp"

#define MOTOR_TRIG_PIN 15

constexpr float MAX_TORQUE_NM = 36.0F; // max torque in N.m
constexpr float MAX_SPEED_RAD = 50.0F; // max speed in rad/s
constexpr float MAX_CURRENT_A = 57.0F; // max current in A

constexpr uint16_t XIAOMI_DEFAULT_MOTOR_CAN_ID = 0x7F;

// CAN ID for ESP32
constexpr uint16_t MASTER_CAN_ID = 0x23;

// how many ms should CAN wait for transmission / reception
constexpr uint32_t CAN_WAIT_TIME = 10;

/**
 * @brief struct to store motor's state response
 */
struct Motor_state
{
    uint8_t CAN_ID;      // CAN ID of the motor
    uint8_t error_state; // 0 for no error, from bit 21 to bit 16 are nothing / hall encoder error / magnetic encoder error / over temperature / over current / undervolt
    uint8_t mode;        // 0 -> reset, 1 -> calibration, 2 -> normal operation

    float angle;       // 0~65535 -> -4pi ~ 4pi
    float angle_v;     // 0~65535 -> -30 ~ 30rad/s
    float torque;      // 0~65535 -> -12 ~ 12 N.m
    float temperature; // temperature/10 = temperature in C
};

/**
 * @brief motor parameter enumeration
 *
 * @note only includes those with float type
 */
enum Motor_param
{
    iq_ref = 0x7006U,        // iq command in current mode, -23~23A, W/R
    spd_ref = 0x700AU,       // speed command in speed mode, -30~30rad/s, W/R
    imit_torque = 0x700BU,   // torque limit, 0~12N.m, W/R
    cur_kp = 0x7010U,        // current Kp, default to 0.125, W/R
    cur_ki = 0x7011U,        // current Ki, default to 0.0158, W/R
    cur_filt_gain = 0x7014U, // current filter gain, 0~1.0, default to 0.1, W/R
    loc_ref = 0x7016U,       // angle command in position mode, unit in rad, W/R
    limit_spd = 0x7017U,     // velocity limit in position mode, 0~30rad/s, W/R
    limit_cur = 0x7018U,     // current limit in position mode, 0~23A, W/R
    mech_pos = 0x7019U,      // position on load side, unit in rad, R
    mech_pos2 = 0X3016U,     // position on load side, unit in rad, R
    iqf = 0x701AU,           // iq filter value, -23~23A, R
    mech_vel = 0x701BU,      // velocity on load side, -30~30 rad/s, R
    VBUS = 0x701CU,          // bus voltage, unit in V, R
    loc_kp = 0x701EU,          // bus voltage, unit in V, R
    VBUS2 = 0x302bU,          // bus voltage, unit in V, R
    not_exist = 0x7020U,
    fault_sta = 0x3022U
};

enum Motor_mode
{
    Motion = 0, // motion control mode
    Position,   // position control mode
    Velocity,   // velocity control mode
    Current     // current(torque) control mode
};

/**
 * @brief Helper function for value parsing
 *
 * @param x value
 * @param x_min min limit
 * @param x_max max limit
 * @param bits number of bits required
 * @return uint16_t value
 */
uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

class Motor
{
public:
    /**
     * @brief Construct a new Motor object and do nothing
     */
    Motor();

    /**
     * @brief Construct a new Motor object and init with ID
     *
     * @param Target_ID CAN_ID of motor
     */
    Motor(const uint8_t Target_ID);

    /**
     * @brief Destroy the Motor object and also uninit the CAN bus if needed
     */
    ~Motor();

    /**
     * @brief Initialize motor and return the MCU ID
     *
     * @param Target_ID CAN_ID of motor
     * @return uint64_t MCU ID, 0 means init error
     *
     * @warning always init before use!
     */
    uint64_t Init(const uint8_t Target_ID);
    uint64_t Init(const uint8_t Target_ID, gpio_num_t can_tx_pin, gpio_num_t can_rx_pin);

    /**
     * @brief uninitialize motor and CAN
     */
    void Uninit();

    /**
     * @brief Enable motor operation
     *
     * @return Motor_state
     */
    Motor_state Enable();

    /**
     * @brief Disable motor operation
     *
     * @param clear_error = 1, if we should clear error in this process
     * @return Motor_state
     */
    Motor_state Disable(const bool clear_error = 0);

    /**
     * @brief Set current motor position to zero point
     *
     * @return Motor_state
     *
     * @note will reset after power off
     */
    Motor_state Set_zero();

    /**
     * @brief Set new CAN ID
     *
     * @param New_ID new CAN_ID of motor
     * @return uint64_t MCU ID, 0 means error
     */
    uint64_t Set_CAN_ID(const uint8_t New_ID);

    /**
     * @brief Read certain parameter
     *
     * @param index parameter index
     * @return float parameter value
     */
    float Read_parameter(const Motor_param index);
    float Read_parameter2(const Motor_param index, const uint8_t data_type);
    float get_voltage();
    uint32_t get_error();

    /**
     * @brief read number of rotations
     *
     * @return int16_t
     */
    int16_t Read_rotation();

    /**
     * @brief Set certain parameter
     *
     * @param index parameter index
     * @param value parameter value to be set
     * @return Motor_state
     *
     * @warning It's your duty to make sure that the parameter can be written into and the value make sense!!!
     */
    Motor_state Set_parameter(const Motor_param index, const float val);

    /**
     * @brief set motor's mode to torque, velocity, or position control
     *
     * @return Motor_state
     * @note one can also do that using Set_parameter, this is only for convenience
     */
    Motor_state Set_mode(const Motor_mode mode);

    /**
     * @brief Set to position control mode and set position
     *
     * @param target_angle -4pi~4pirad
     * @return Motor_state
     *
     * @note For more complicated operations, please use Set_control function
     * for motion control and refer to the diagram for how these targets
     * translates to the actual expected current. By calling this function you
     * basically let the motor controller determines how it accomplish this
     * goal.
     * @note This function will set the operation mode to position control mode
     */
    Motor_state Get_state();
    Motor_state Set_position(const float target_angle);

    /**
     * @brief Set to velocity control mode and set velocity
     *
     * @param target_vel -30~30rad/s
     * @return Motor_state
     *
     * @note For more complicated operations, please use Set_control function
     * for motion control and refer to the diagram for how these targets
     * translates to the actual expected current. By calling this function you
     * basically let the motor controller determines how it accomplish this
     * goal.
     * @note This function will set the operation mode to velocity control mode
     */
    Motor_state Set_velocity(const float target_vel);

    /**
     * @brief Set to current control mode and set current (torque)
     *
     * @param target_torque -23~23A
     * @return Motor_state
     *
     * @note For more complicated operations, please use Set_control function
     * for motion control and refer to the diagram for how these targets
     * translates to the actual expected current. By calling this function you
     * basically let the motor controller determines how it accomplish this
     * goal.
     * @note This function will set the operation mode to current control mode
     */
    Motor_state Set_current(const float target_current);

    /**
     * @brief Set control values in floats
     *
     * @param target_torque -12~12N.m
     * @param target_angle -4pi~4pirad, default to 0
     * @param target_vel -30~30rad/s, default to 0
     * @param Kp 0~500.0, default to 0
     * @param Kd 0~5.0, default to 0
     * @return Motor_state
     *
     * @note Refer to the diagram for how these targets translates to the actual
     * expected current. For simple position/velocity/torque control, you could
     * either use this function and set other unwanted values to 0 or directly
     * use the corresponding Set_position, Set_velocity, Set_torque functions,
     * which will call Set_parameter directly and let motor controller handle
     * the rest.
     * 
     * @note when ignoring all parameters except target_torque, i.e. Set_control(torque), this is basically torque control mode.
     */
    Motor_state Set_control(const float target_torque, const float target_angle = 0.0F, const float target_vel = 0.0F, const float Kp = 0.0F, const float Kd = 0.0F);

    /**
     * @brief Set control values in uint16_t
     *
     * @param target_torque 0~65535 -> -12~12N.m
     * @param target_angle 0~65535 -> -4pi~4pirad
     * @param target_vel 0~65535 -> -30~30rad/s
     * @param Kp 0~65535 -> 0~500.0
     * @param Kd 0~65535 -> 0~5.0
     * @return Motor_state
     *
     * @note Refer to the diagram for how these targets translates to the actual
     * expected current. For simple position/velocity/torque control, you could
     * either use this function and set other unwanted values to 0 or directly
     * use the corresponding Set_position, Set_velocity, Set_torque functions,
     * which will call Set_parameter directly and let motor controller handle
     * the rest.
     */
    Motor_state Set_control_int(const uint16_t target_torque, const uint16_t target_angle, const uint16_t target_vel, const uint16_t Kp, const uint16_t Kd);

    bool calibrated = false;

private:
    // CAN ID of this motor
    uint8_t CAN_ID = XIAOMI_DEFAULT_MOTOR_CAN_ID;

    // has the CAN bus been initialized
    bool CAN_inited = false;

    // current motor mode
    Motor_mode curr_mode = Motor_mode::Motion;

    /**
     * @brief unpack the returned data to Motor_state class
     *
     * @param msg can message
     * @return Motor_state
     */
    Motor_state Unpack(const twai_message_t msg);

    /**
     * @brief helper function for CAN message transcieving
     *
     * @param TX_msg pointer to the message to be transmitted
     * @param RX_msg pointer to the place where the message will be stored
     *
     * @return 1 if successful
     */
    bool CAN_Transceive(twai_message_t *const TX_msg_ptr, twai_message_t *const RX_msg_ptr);
};

class ButterworthFilter {
public:
    // Constructor: Initializes the filter with a cutoff frequency and sampling rate
    ButterworthFilter(double cutoffFreq, double samplingRate);

    // Set the cutoff frequency and recompute the filter coefficients
    void setCutoffFrequency(double cutoffFreq, double samplingRate);

    // Reset the filter state (clears the history of inputs and outputs)
    void reset();

    // Apply the filter to the input signal and return the filtered value
    double filter(double input);

private:
    // Coefficients for the difference equation
    double a0, a1, a2;
    double b1, b2;

    // Input and output history (used to store previous inputs and outputs)
    double x1, x2;  // Previous inputs
    double y1, y2;  // Previous outputs
};


#endif