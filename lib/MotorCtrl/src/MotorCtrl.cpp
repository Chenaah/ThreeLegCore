#include "MotorCtrl.hpp"
#include <cmath>

#define _USE_MATH_DEFINES

#define DEBUG_ENABLED 1

#if DEBUG_ENABLED
#define DEBUG_PRINT(c) Serial.print(c)
#elif
#define DEBUG_PRINT(c) 0
#endif


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

    if (twai_receive(RX_msg_ptr, pdMS_TO_TICKS(CAN_WAIT_TIME)) != ESP_OK)
    {
        DEBUG_PRINT("Failed to receive message\n");
        calibrated = false;
        return 0;
    }

    return 1;
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
{
}

Motor::Motor(uint8_t Target_ID)
{
    Init(Target_ID);
}

Motor::~Motor()
{
    Uninit();
}

Motor_state Motor::Unpack(const twai_message_t msg)
{
    Motor_state temp;

    temp.CAN_ID = (msg.identifier >> 8) & 0xFF;
    temp.error_state = (msg.identifier >> 16) & 0xFF; // cmd_data[1]
    temp.mode = (msg.identifier >> 22) & 0x03;

    temp.angle = (float((uint16_t(msg.data[0]) << 8) + msg.data[1]) / 65536.0F - 0.5F) * 8.0F * M_PI;
    temp.angle_v = (float((uint16_t(msg.data[2]) << 8) + msg.data[3]) / 65536.0F - 0.5F) * MAX_SPEED_RAD * 2.0F;
    temp.torque = (float((uint16_t(msg.data[4]) << 8) + msg.data[5]) / 65536.0F - 0.5F) * MAX_TORQUE_NM * 2.0F;
    temp.temperature = float((uint16_t(msg.data[6]) << 8) + msg.data[7]) / 10.0F;

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
        float_to_uint(target_torque, -MAX_TORQUE_NM, MAX_TORQUE_NM, 16),
        float_to_uint(target_angle, -4.0F * M_PI, 4.0F * M_PI, 16),
        float_to_uint(target_vel, -MAX_SPEED_RAD, MAX_SPEED_RAD, 16),
        float_to_uint(Kp, 0.0F, 5000.0F, 16),
        float_to_uint(Kd, 0.0F, 100.0F, 16));
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
        Set_parameter(Motor_param::limit_spd, MAX_SPEED_RAD);
        // Set_parameter(Motor_param::imit_torque, 10.0F);
    }
    return Set_parameter(Motor_param::loc_ref, target_angle);
}

Motor_state Motor::Set_velocity(const float target_vel)
{
    if (curr_mode != Motor_mode::Velocity)
    {
        Set_mode(Motor_mode::Velocity);
        Set_parameter(Motor_param::limit_cur, MAX_CURRENT_A);
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
    Set_parameter(Motor_param::limit_spd, MAX_SPEED_RAD);
    Set_parameter(Motor_param::imit_torque, MAX_TORQUE_NM);
    return Set_parameter(Motor_param::limit_cur, MAX_CURRENT_A);
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
