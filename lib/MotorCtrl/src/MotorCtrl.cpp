#include "MotorCtrl.hpp"
#include <cmath>

#define _USE_MATH_DEFINES

#define DEBUG_ENABLED 1

#if DEBUG_ENABLED
#define DEBUG_PRINT(c) Serial.print(c)
#elif
#define DEBUG_PRINT(c) 0
#endif

namespace
{
constexpr float kPi = 3.14159265358979323846F;

float uint_to_float(const uint16_t x, const float x_min, const float x_max)
{
    return (float(x) / 65536.0F) * (x_max - x_min) + x_min;
}

Motor_profile get_builtin_profile(const Motor_type type)
{
    switch (type)
    {
    case Motor_type::RS03:
        return Motor_profile{
            -60.0F, 60.0F,
            -4.0F * kPi, 4.0F * kPi,
            -20.0F, 20.0F,
            0.0F, 5000.0F,
            0.0F, 100.0F,
            10.0F};
    case Motor_type::Cybergear:
    default:
        return Motor_profile{
            -12.0F, 12.0F,
            -4.0F * kPi, 4.0F * kPi,
            -30.0F, 30.0F,
            0.0F, 500.0F,
            0.0F, 5.0F,
            10.0F};
    }
}
} // namespace


uint16_t float_to_uint(const float x, const float x_min, const float x_max, const int bits)
{
    float x1 = x;
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x1 = x_max;
    else if (x < x_min)
        x1 = x_min;
    return (uint16_t)((x1 - offset) * ((float)((1 << bits) - 1)) / span);
}

// Global storage for fault frames captured during CAN communication
static Motor_fault_state g_last_fault = {};
static bool g_fault_frame_received = false;

/**
 * @brief Parse a fault frame message into Motor_fault_state
 */
static void parse_fault_frame(const twai_message_t& rx_msg, Motor_fault_state* fault_out) {
    // Debug: Print raw CAN data
    Serial.printf("[Fault] Raw CAN ID: 0x%08X\n", rx_msg.identifier);
    Serial.printf("[Fault] Raw data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                  rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3],
                  rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);
    
    // Extract motor CAN_ID from bits 23-0 of the identifier
    fault_out->motor_id = rx_msg.identifier & 0xFFFF;
    
    // Fault flag from bits 0-3 of Byte 0
    fault_out->fault_flag = rx_msg.data[0] & 0x0F;
    
    // Extract fault bits from Byte 0, Byte 1, and Byte 2 (combined as 24-bit value)
    uint32_t fault_bits = (uint32_t(rx_msg.data[2]) << 16) | 
                          (uint32_t(rx_msg.data[1]) << 8) | 
                          rx_msg.data[0];
    
    Serial.printf("[Fault] fault_bits (24-bit): 0x%06X\n", fault_bits);
    
    fault_out->phase_a_overflow = (fault_bits >> 16) & 0x01;        // bit16
    fault_out->overload_fault = (fault_bits >> 8) & 0xFF;           // bit15-8
    fault_out->encoder_not_calibrated = (fault_bits >> 7) & 0x01;   // bit7
    fault_out->phase_c_overflow = (fault_bits >> 5) & 0x01;         // bit5
    fault_out->phase_b_overflow = (fault_bits >> 4) & 0x01;         // bit4
    fault_out->over_voltage = (fault_bits >> 3) & 0x01;             // bit3
    fault_out->under_voltage = (fault_bits >> 2) & 0x01;            // bit2
    fault_out->driver_chip_fault = (fault_bits >> 1) & 0x01;        // bit1
    fault_out->motor_over_temp = fault_bits & 0x01;                 // bit0
    
    // Warning values from bytes 4-7
    fault_out->warning_byte4 = rx_msg.data[4];
    fault_out->warning_byte5 = rx_msg.data[5];
    fault_out->warning_byte6 = rx_msg.data[6];
    fault_out->warning_byte7 = rx_msg.data[7];
    
    // Temperature warning is bit0 of warning bytes
    fault_out->temp_warning = (rx_msg.data[4] & 0x01) || 
                              (rx_msg.data[5] & 0x01) || 
                              (rx_msg.data[6] & 0x01) || 
                              (rx_msg.data[7] & 0x01);
}

bool Motor::CAN_Transceive(twai_message_t *const TX_msg_ptr, twai_message_t *const RX_msg_ptr)
{
    // Clear RX buffer to avoid stale data
    memset(RX_msg_ptr, 0, sizeof(twai_message_t));

    if (twai_transmit(TX_msg_ptr, pdMS_TO_TICKS(CAN_WAIT_TIME)) != ESP_OK)
    {
        send_led_message(LED_MSG_MOTOR_ERROR);
        DEBUG_PRINT("Oh no! Failed to talk to the motor! \n");
        calibrated = false;
        return 0;
    }

    // Try to receive, but check if it's a fault frame
    // If we get a fault frame, store it and try to receive again for the actual response
    int max_retries = 3;
    for (int i = 0; i < max_retries; i++) {
        if (twai_receive(RX_msg_ptr, pdMS_TO_TICKS(CAN_WAIT_TIME)) != ESP_OK)
        {
            DEBUG_PRINT("Failed to receive message\n");
            calibrated = false;
            return 0;
        }
        
        // Check if this is a fault frame (Communication Type 21 = 0x15)
        uint8_t msg_type = (RX_msg_ptr->identifier >> 24) & 0x1F;
        if (msg_type == 0x15) {
            // This is a fault frame - store it and try to receive again
            parse_fault_frame(*RX_msg_ptr, &g_last_fault);
            g_fault_frame_received = true;
            DEBUG_PRINT("Fault frame captured during CAN_Transceive\n");
            // Continue loop to get the actual response
        } else {
            // This is the expected response
            return 1;
        }
    }
    
    // If we got here, we only received fault frames
    DEBUG_PRINT("Only received fault frames, no response\n");
    return 0;
}

uint64_t Motor::Init(const uint8_t Target_ID){
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
        // Initialize configuration structures using macro initializers
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

        // set state to initialized
        CAN_inited = true;
    }

    CAN_ID = Target_ID;

    // check devide ID
    // construct TX
    twai_message_t tx_msg;
    tx_msg.identifier = (0 << 24) + (uint32_t(MASTER_CAN_ID) << 8) + CAN_ID;
    tx_msg.flags = TWAI_MSG_FLAG_EXTD;
    tx_msg.data_length_code = 8;
    for (int i = 0; i < 8; i++)
    {
        tx_msg.data[i] = 0;
    }

    twai_message_t rx_msg;

    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        uint64_t MCU_ID = 0;
        for (int i = 0; i < 7; i++)
        {
            MCU_ID += (rx_msg.data[i] << (8 * (7 - i)));
        }
        return MCU_ID;
    }
    else
    {
        return 0;
    }
}

void Motor::Uninit()
{
    // Stop the CAN driver
    if (twai_stop() == ESP_OK)
    {
        DEBUG_PRINT("Driver stopped\n");
    }
    else
    {
        DEBUG_PRINT("Failed to stop driver\n");
        return;
    }

    // Uninstall the CAN driver
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

Motor::Motor()
    : motor_type(Motor_type::Cybergear), motor_profile(get_builtin_profile(Motor_type::Cybergear))
{
}

Motor::Motor(Motor_type type)
    : motor_type(type), motor_profile(get_builtin_profile(type))
{
}

Motor::Motor(uint8_t Target_ID)
    : motor_type(Motor_type::Cybergear), motor_profile(get_builtin_profile(Motor_type::Cybergear))
{
    Init(Target_ID);
}

Motor::Motor(uint8_t Target_ID, Motor_type type)
    : motor_type(type), motor_profile(get_builtin_profile(type))
{
    Init(Target_ID);
}

Motor::~Motor()
{
    Uninit();
}

void Motor::Set_motor_type(const Motor_type type)
{
    motor_type = type;
    motor_profile = get_builtin_profile(type);
}

void Motor::Set_motor_profile(const Motor_profile& profile)
{
    motor_profile = profile;
}

Motor_type Motor::Get_motor_type() const
{
    return motor_type;
}

const Motor_profile& Motor::Get_motor_profile() const
{
    return motor_profile;
}

Motor_state Motor::Unpack(const twai_message_t msg)
{
    Motor_state temp;

    temp.CAN_ID = (msg.identifier >> 8) & 0xFF;
    temp.error_state = (msg.identifier >> 16) & 0xFF; // cmd_data[1]
    temp.mode = (msg.identifier >> 22) & 0x03;

    temp.angle = uint_to_float((uint16_t(msg.data[0]) << 8) + msg.data[1], motor_profile.angle_min, motor_profile.angle_max);
    temp.angle_v = uint_to_float((uint16_t(msg.data[2]) << 8) + msg.data[3], motor_profile.vel_min, motor_profile.vel_max);
    temp.torque = uint_to_float((uint16_t(msg.data[4]) << 8) + msg.data[5], motor_profile.torque_min, motor_profile.torque_max);
    temp.temperature = float((uint16_t(msg.data[6]) << 8) + msg.data[7]) / motor_profile.temperature_scale;

    return temp;
}

Motor_state Motor::Enable()
{
    // construct TX
    twai_message_t tx_msg;
    tx_msg.identifier = (3 << 24) + (uint32_t(MASTER_CAN_ID) << 8) + CAN_ID;
    tx_msg.flags = TWAI_MSG_FLAG_EXTD;
    tx_msg.data_length_code = 8;
    for (int i = 0; i < 8; i++)
    {
        tx_msg.data[i] = 0;
    }

    twai_message_t rx_msg;

    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        return Unpack(rx_msg);
    }
    else
    {
        return Motor_state{};
    }
}

Motor_state Motor::Disable(const bool clear_error)
{
    // construct TX
    twai_message_t tx_msg;
    tx_msg.identifier = (4 << 24) + (uint32_t(MASTER_CAN_ID) << 8) + CAN_ID;
    tx_msg.flags = TWAI_MSG_FLAG_EXTD;
    tx_msg.data_length_code = 8;
    for (int i = 0; i < 8; i++)
    {
        tx_msg.data[i] = 0;
    }
    tx_msg.data[0] = clear_error;

    twai_message_t rx_msg;

    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        return Unpack(rx_msg);
    }
    else
    {
        return Motor_state{};
    }
}

Motor_state Motor::Set_zero()
{
    // construct TX
    twai_message_t tx_msg;
    tx_msg.identifier = (6 << 24) + (uint32_t(MASTER_CAN_ID) << 8) + CAN_ID;
    tx_msg.flags = TWAI_MSG_FLAG_EXTD;
    tx_msg.data_length_code = 8;
    for (int i = 0; i < 8; i++)
    {
        tx_msg.data[i] = 0;
    }

    tx_msg.data[0] = 1;

    twai_message_t rx_msg;

    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        return Unpack(rx_msg);
    }
    else
    {
        return Motor_state{};
    }
}

uint64_t Motor::Set_CAN_ID(const uint8_t New_ID)
{
    // construct TX
    twai_message_t tx_msg;
    tx_msg.identifier = (7 << 24) + (uint32_t(New_ID) << 16) + (uint32_t(MASTER_CAN_ID) << 8) + CAN_ID;
    tx_msg.flags = TWAI_MSG_FLAG_EXTD;
    tx_msg.data_length_code = 8;
    for (int i = 0; i < 8; i++)
    {
        tx_msg.data[i] = 0;
    }

    twai_message_t rx_msg;

    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        uint64_t MCU_ID = 0;
        for (int i = 0; i < 7; i++)
        {
            MCU_ID += (rx_msg.data[i] << (8 * (7 - i)));
        }
        return MCU_ID;
    }
    else
    {
        return 0;
    }
}

Motor_state Motor::Set_control_int(const uint16_t target_torque, const uint16_t target_angle, const uint16_t target_vel, const uint16_t Kp, const uint16_t Kd)
{
    if (curr_mode != Motor_mode::Motion)
    {
        Set_mode(Motor_mode::Motion);
    }

    // construct TX
    twai_message_t tx_msg;
    tx_msg.identifier = (1 << 24) + (uint32_t(target_torque) << 8) + CAN_ID;
    tx_msg.flags = TWAI_MSG_FLAG_EXTD;
    tx_msg.data_length_code = 8;
    tx_msg.data[0] = (target_angle >> 8) & 0xFF;
    tx_msg.data[1] = target_angle & 0xFF;
    tx_msg.data[2] = (target_vel >> 8) & 0xFF;
    tx_msg.data[3] = target_vel & 0xFF;
    tx_msg.data[4] = (Kp >> 8) & 0xFF;
    tx_msg.data[5] = Kp & 0xFF;
    tx_msg.data[6] = (Kd >> 8) & 0xFF;
    tx_msg.data[7] = Kd & 0xFF;

    twai_message_t rx_msg;

    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        return Unpack(rx_msg);
    }
    else
    {
        return Motor_state{};
    }
}

Motor_state Motor::Set_control(const float target_torque, const float target_angle, const float target_vel, const float Kp, const float Kd)
{
    return Set_control_int(
        float_to_uint(target_torque, motor_profile.torque_min, motor_profile.torque_max, 16),
        float_to_uint(target_angle, motor_profile.angle_min, motor_profile.angle_max, 16),
        float_to_uint(target_vel, motor_profile.vel_min, motor_profile.vel_max, 16),
        float_to_uint(Kp, motor_profile.kp_min, motor_profile.kp_max, 16),
        float_to_uint(Kd, motor_profile.kd_min, motor_profile.kd_max, 16));
}

float Motor::Read_parameter(const Motor_param index)
{
    // construct TX
    twai_message_t tx_msg;
    tx_msg.identifier = (17 << 24) + (uint32_t(MASTER_CAN_ID) << 8) + CAN_ID;
    tx_msg.flags = TWAI_MSG_FLAG_EXTD;
    tx_msg.data_length_code = 8;

    uint16_t indexi = uint16_t(index);
    memset(tx_msg.data, 0, 8); // Clear all data bytes
    memcpy(&tx_msg.data[0], &indexi, 2);

    twai_message_t rx_msg;
    memset(&rx_msg, 0, sizeof(rx_msg));


    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        // Validate the identifier of the response
        uint32_t expected_id = (17 << 24) + (uint32_t(CAN_ID) << 8) + MASTER_CAN_ID;
        if (rx_msg.identifier == expected_id){
            // Serial.println("Received message");
            float retval = 0;
            memcpy(&retval, &rx_msg.data[4], 4);
            return retval;
        } 
        // else {
        //     Serial.println("Received message with wrong ID");
        //     return float(rx_msg.identifier >> 24);
        // }
    }
    return 0;
}

float Motor::Read_parameter2(const Motor_param index, const uint8_t data_type)
{
    // construct TX
    twai_message_t tx_msg;
    tx_msg.identifier = (9 << 24) + (uint32_t(MASTER_CAN_ID) << 8) + CAN_ID;
    tx_msg.flags = TWAI_MSG_FLAG_EXTD;
    tx_msg.data_length_code = 8;

    uint16_t indexi = uint16_t(index);
    for (uint8_t i = 0; i < 8; i++)
    {
        tx_msg.data[i] = 0;
    }
    memcpy(&tx_msg.data[0], &indexi, 2);
    memcpy(&tx_msg.data[2], &data_type, 1);

    twai_message_t rx_msg;

    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        float retval = 0;
        memcpy(&retval, &rx_msg.data[4], 4);

        return retval;
    }
    else
    {
        return 0;
    }
}

float Motor::get_voltage(){
    return Read_parameter2(Motor_param::VBUS2, 6);
}

uint32_t Motor::get_error(){
    return Read_parameter2(Motor_param::fault_sta, 4);
}

int16_t Motor::Read_rotation()
{
    // construct TX
    twai_message_t tx_msg;
    tx_msg.identifier = (17 << 24) + (uint32_t(MASTER_CAN_ID) << 8) + CAN_ID;
    tx_msg.flags = TWAI_MSG_FLAG_EXTD;
    tx_msg.data_length_code = 8;

    uint16_t indexi = 0x701DU;
    for (uint8_t i = 0; i < 8; i++)
    {
        tx_msg.data[i] = 0;
    }
    memcpy(&tx_msg.data[0], &indexi, 2);

    twai_message_t rx_msg;

    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        int16_t retval = 0;
        memcpy(&retval, &rx_msg.data[4], 2);

        // DEBUG_PRINT("Read: index = ");
        // DEBUG_PRINT((uint32_t(rx_msg.data[1]) << 8) + rx_msg.data[0]);
        // DEBUG_PRINT(" , value = ");
        // DEBUG_PRINT(retval);
        // DEBUG_PRINT("\n");

        return retval;
    }
    else
    {
        return 0;
    }
}

Motor_state Motor::Set_parameter(const Motor_param index, const float val)
{
    // construct TX
    twai_message_t tx_msg;
    tx_msg.identifier = (18 << 24) + (uint32_t(MASTER_CAN_ID) << 8) + CAN_ID;
    tx_msg.flags = TWAI_MSG_FLAG_EXTD;
    tx_msg.data_length_code = 8;

    uint16_t indexi = uint16_t(index);
    for (uint8_t i = 0; i < 8; i++)
    {
        tx_msg.data[i] = 0;
    }
    memcpy(&tx_msg.data[0], &indexi, 2);
    memcpy(&tx_msg.data[4], &val, 4);

    // DEBUG_PRINT("Write: index = ");
    // DEBUG_PRINT(indexi);
    // DEBUG_PRINT(" , val = ");
    // DEBUG_PRINT(val);
    // DEBUG_PRINT("\n");

    twai_message_t rx_msg;

    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        return Unpack(rx_msg);
    }
    else
    {
        return Motor_state{};
    }
}

Motor_state Motor::Set_mode(const Motor_mode mode)
{
    // construct TX
    twai_message_t tx_msg;
    tx_msg.identifier = (18 << 24) + (uint32_t(MASTER_CAN_ID) << 8) + CAN_ID;
    tx_msg.flags = TWAI_MSG_FLAG_EXTD;
    tx_msg.data_length_code = 8;

    uint16_t index = 0x7005U;
    uint8_t runmode = uint8_t(mode);

    for (uint8_t i = 0; i < 8; i++)
    {
        tx_msg.data[i] = 0;
    }
    memcpy(&tx_msg.data[0], &index, 2);
    memcpy(&tx_msg.data[4], &runmode, 1);

    twai_message_t rx_msg;

    if (CAN_Transceive(&tx_msg, &rx_msg))
    {
        return Unpack(rx_msg);
    }
    else
    {
        return Motor_state{};
    }
}

Motor_state Motor::Set_position(const float target_angle)
{
    if (curr_mode != Motor_mode::Position)
    {
        Set_mode(Motor_mode::Position);
        Set_parameter(Motor_param::limit_spd, 25.0F);
        // Set_parameter(Motor_param::imit_torque, 10.0F);
    }
    return Set_parameter(Motor_param::loc_ref, target_angle);
}

Motor_state Motor::Set_velocity(const float target_vel)
{
    if (curr_mode != Motor_mode::Velocity)
    {
        Set_mode(Motor_mode::Velocity);
        Set_parameter(Motor_param::limit_cur, 23.0F);
    }
    return Set_parameter(Motor_param::spd_ref, target_vel);
}

Motor_state Motor::Set_current(const float target_current)
{
    if (curr_mode != Motor_mode::Current)
    {
        Set_mode(Motor_mode::Current);
    }
    return Set_parameter(Motor_param::iq_ref, target_current);
}

Motor_state Motor::Get_state()
{
    // if (curr_mode != Motor_mode::Position)
    // {
    //     Set_mode(Motor_mode::Position);
    //     Set_parameter(Motor_param::limit_spd, 30.0F);
    // }
    // return Set_parameter(Motor_param::not_exist, 30.0F);
    Set_parameter(Motor_param::limit_spd, 30.0F);
    Set_parameter(Motor_param::imit_torque, 10.0F);
    return Set_parameter(Motor_param::limit_cur, 27.0F);
}

bool Motor::Check_Fault_Frame(Motor_fault_state* fault_out)
{
    // First, check if a fault was captured during CAN_Transceive
    if (g_fault_frame_received) {
        if (fault_out) {
            *fault_out = g_last_fault;
        }
        g_fault_frame_received = false;  // Clear the flag
        DEBUG_PRINT("Returning captured fault frame\n");
        return true;
    }
    
    // Also do a non-blocking check for any fault frames in the buffer
    twai_message_t rx_msg;
    
    // Non-blocking receive - check if any message is available (timeout = 0)
    if (twai_receive(&rx_msg, 0) != ESP_OK) {
        return false; // No message available
    }
    
    // Check if this is a fault feedback frame (Communication Type 21 = 0x15)
    // Bits 28-24 contain the message type
    uint8_t msg_type = (rx_msg.identifier >> 24) & 0x1F;
    if (msg_type != 0x15) {
        return false; // Not a fault frame
    }
    
    // Parse and return the fault frame
    if (fault_out) {
        parse_fault_frame(rx_msg, fault_out);
    }
    
    DEBUG_PRINT("Fault frame received from motor ");
    DEBUG_PRINT(rx_msg.identifier & 0xFFFF);
    DEBUG_PRINT("\n");
    
    return true;
}


// Constructor: Initializes the filter and sets the cutoff frequency and sampling rate
ButterworthFilter::ButterworthFilter(double cutoffFreq, double samplingRate) {
    setCutoffFrequency(cutoffFreq, samplingRate);  // Calculate initial coefficients
    reset();  // Reset the filter state (history of inputs/outputs)
}

// Set the cutoff frequency and compute filter coefficients
void ButterworthFilter::setCutoffFrequency(double cutoffFreq, double samplingRate) {
    // Nyquist frequency is half the sampling rate
    double nyquist = 0.5 * samplingRate;
    
    // Normalized cutoff frequency (0 to 1, where 1 corresponds to the Nyquist frequency)
    double normalizedCutoff = cutoffFreq / nyquist;

    // Prewarp the frequency for bilinear transform
    double K = std::tan(M_PI * normalizedCutoff);
    double K_squared = K * K;

    // Compute the filter coefficients based on the normalized frequency
    double norm = 1 / (1 + std::sqrt(2) * K + K_squared);
    a0 = K_squared * norm;
    a1 = 2 * a0;
    a2 = a0;
    b1 = 2 * (K_squared - 1) * norm;
    b2 = (1 - std::sqrt(2) * K + K_squared) * norm;


}

// Reset the filter (clear input/output history)
void ButterworthFilter::reset() {
    x1 = x2 = 0.0;  // Clear previous inputs
    y1 = y2 = 0.0;  // Clear previous outputs
}

// Apply the filter to the input value and return the filtered output
double ButterworthFilter::filter(double input) {
    // Apply the difference equation for a second-order low-pass Butterworth filter
    double output = a0 * input + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;

    // Update the input/output history for the next call
    x2 = x1;  // Shift previous input x[n-1] to x[n-2]
    x1 = input;  // Store current input as x[n-1]
    
    y2 = y1;  // Shift previous output y[n-1] to y[n-2]
    y1 = output;  // Store current output as y[n-1]

    return output;
}
