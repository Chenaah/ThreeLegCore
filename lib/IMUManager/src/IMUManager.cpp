/**
 * @file IMUManager.cpp
 * @brief Zero-cost abstraction layer for BNO08x IMU
 *
 * This is a ~0 cost simplification layer that:
 * - Provides customized initialization for the BNO08x IMU
 * - Reads and packs data directly from BNO08x when GetData() is called
 * - Extrapolates data using timestamps from BNO08x library
 *
 * No callbacks, no extra tasks, no overhead - just direct reads.
 */
#include <IMUManager.hpp>
extern "C"
{
#include <bno08x_driver.h>
}
#include <esp_timer.h>
#include <cmath>

/* Debug macros - disabled by default */
#define DEBUG_IMU 0

#if DEBUG_IMU
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(fmt, ...)
#endif

using namespace Math3D;

namespace IMUManager
{

    // ===== PRIVATE VARIABLES =====
    static BNO08x imu;
    static bool is_initialized = false;

    // ===== PRIVATE FUNCTION DECLARATIONS =====
    static IMUData ExtrapolateIMUData(const IMUData &raw_data, int64_t current_time_us);

    // ===== PRIVATE FUNCTIONS =====

    /**
     * @brief Extrapolate IMU data to current time
     *
     * Uses angular velocity to extrapolate quaternion and acceleration
     * from the time they were measured to the current time. Each sensor
     * component uses its own timestamp for more accurate extrapolation.
     *
     * @param raw_data The raw IMU data to extrapolate
     * @param current_time_us Current time in microseconds
     * @return Extrapolated IMU data
     */
    static IMUData ExtrapolateIMUData(const IMUData &raw_data, int64_t current_time_us)
    {
        IMUData extrapolated = raw_data;

        // Calculate time delays for each component from their individual timestamps
        float quaternion_dt_s = (current_time_us - raw_data.quaternion_timestamp_us) / 1000000.0f;
        float acceleration_dt_s = (current_time_us - raw_data.acceleration_timestamp_us) / 1000000.0f;

        // Use latest angular velocity for all extrapolations
        Vector3D latest_angular_velocity = raw_data.angular_velocity;
        float angv_magnitude = latest_angular_velocity.magnitude();

        // Extrapolate quaternion using its specific time delay
        if (quaternion_dt_s > 0 && angv_magnitude > 0.001f)
        {
            Vector3D angv_axis = latest_angular_velocity / angv_magnitude;
            float angle = angv_magnitude * quaternion_dt_s;

            // Create rotation quaternion from body-frame angular velocity
            Quaternion rot_quat(angv_axis, angle);

            // Multiply quaternions: result = w_quat * rot_quat (for body-frame angular velocity)
            extrapolated.quaternion = raw_data.quaternion * rot_quat;
            extrapolated.quaternion.normalize();
        }

        // Extrapolate acceleration using its specific time delay
        if (acceleration_dt_s > 0 && angv_magnitude > 0.001f)
        {
            Vector3D angv_axis = latest_angular_velocity / angv_magnitude;
            float angle = angv_magnitude * acceleration_dt_s;

            // Create rotation quaternion
            Quaternion rot_quat(angv_axis, angle);

            // Apply inverse body rotation to acceleration (including gravity)
            extrapolated.acceleration = rotate(rot_quat.conjugate(), raw_data.acceleration);
        }

        // Angular velocity doesn't need rotation extrapolation as it's the reference

        // set timestamp of all components to current time
        // but do not set last_data_time_us here
        extrapolated.quaternion_timestamp_us = current_time_us;
        extrapolated.acceleration_timestamp_us = current_time_us;
        extrapolated.angular_velocity_timestamp_us = current_time_us;

        return extrapolated;
    }

    // ===== PUBLIC FUNCTIONS =====

    bool Initialize(uint8_t cs_pin, uint8_t int_pin, uint8_t rst_pin, uint32_t sample_interval_ms, uint32_t task_priority)
    {
        DEBUG_PRINTLN("IMUManager: Initializing...");

        // Convert sample interval from ms to us
        uint32_t sample_interval_us = sample_interval_ms * 1000;

        // Configure IMU with hardware pins
        BNO08x_config_t cfg = {
            .spi_peripheral = SPI2_HOST,
            .io_cs = (gpio_num_t)cs_pin,
            .io_int = (gpio_num_t)int_pin,
            .io_rst = (gpio_num_t)rst_pin,
            .sclk_speed = 3000000, // 3 MHz SPI clock
            .task_priority = task_priority};

        // Initialize SPI peripheral and GPIO
        BNO08x_init(&imu, &cfg);
        DEBUG_PRINTLN("IMUManager: SPI and GPIO initialized");

        // Initialize BNO08x (this creates the internal spi_task)
        if (!BNO08x_initialize(&imu))
        {
            DEBUG_PRINTLN("IMUManager: Failed to initialize BNO08x IMU");
            return false;
        }
        DEBUG_PRINTLN("IMUManager: BNO08x detected and initialized");

        // NO CALLBACK REGISTRATION - we read directly in GetData()

        // Enable sensor reports with specified interval (in microseconds)
        // Rotation vector for orientation
        BNO08x_enable_rotation_vector(&imu, sample_interval_us);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Gyroscope for angular velocity
        BNO08x_enable_gyro(&imu, sample_interval_us);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Accelerometer for acceleration
        BNO08x_enable_accelerometer(&imu, sample_interval_us);
        vTaskDelay(pdMS_TO_TICKS(100));

        DEBUG_PRINTLN("IMUManager: Sensor reports enabled");
        DEBUG_PRINTLN("IMUManager: Initialization complete (direct read mode)");
        is_initialized = true;

        return true;
    }

    IMUData GetData()
    {
        // Get raw data then extrapolate
        return ExtrapolateIMUData(GetRawData(), esp_timer_get_time());
    }

    IMUData GetRawData()
    {
        IMUData data;
        uint32_t count;

        // Read quaternion with consistency check
        do
        {
            count = BNO08x_get_quat_update_count(&imu);
            data.quaternion.w = BNO08x_get_quat_real(&imu);
            data.quaternion.x = BNO08x_get_quat_I(&imu);
            data.quaternion.y = BNO08x_get_quat_J(&imu);
            data.quaternion.z = BNO08x_get_quat_K(&imu);
            data.quaternion_timestamp_us = BNO08x_get_quat_timestamp_us(&imu);
        } while (count != BNO08x_get_quat_update_count(&imu));

        // Read gyro with consistency check
        do
        {
            count = BNO08x_get_gyro_update_count(&imu);
            data.angular_velocity.x = BNO08x_get_gyro_calibrated_velocity_X(&imu);
            data.angular_velocity.y = BNO08x_get_gyro_calibrated_velocity_Y(&imu);
            data.angular_velocity.z = BNO08x_get_gyro_calibrated_velocity_Z(&imu);
            data.angular_velocity_timestamp_us = BNO08x_get_gyro_timestamp_us(&imu);
        } while (count != BNO08x_get_gyro_update_count(&imu));

        // Read accelerometer with consistency check
        do
        {
            count = BNO08x_get_accel_update_count(&imu);
            data.acceleration.x = BNO08x_get_accel_X(&imu);
            data.acceleration.y = BNO08x_get_accel_Y(&imu);
            data.acceleration.z = BNO08x_get_accel_Z(&imu);
            data.acceleration_timestamp_us = BNO08x_get_accel_timestamp_us(&imu);
        } while (count != BNO08x_get_accel_update_count(&imu));

        data.last_data_time_us = std::max({data.quaternion_timestamp_us, data.acceleration_timestamp_us, data.angular_velocity_timestamp_us});

        return data;
    }

    bool InitializedQ()
    {
        return is_initialized;
    }

    void* GetIMUDevice()
    {
        return is_initialized ? (void*)&imu : NULL;
    }

} // namespace IMUManager
