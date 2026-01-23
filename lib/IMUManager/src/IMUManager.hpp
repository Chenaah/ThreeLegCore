/**
 * @file IMUManager.hpp
 * @brief Simplified thread-safe IMU data management layer
 * 
 * This is an intermediate layer providing:
 * - Customized initialization for the BNO08x IMU
 * - Data packing after reading from BNO08x
 * - Data extrapolation using timestamps from BNO08x library
 * 
 * No extra tasks are needed since timing information is obtained
 * directly from the BNO08x library.
 */
#ifndef IMUMANAGER_HPP
#define IMUMANAGER_HPP

#include "Arduino.h"
#include <Quaternion.hpp>
#include <Vector3D.hpp>

using namespace Math3D;

namespace IMUManager
{
    /**
     * @brief Simple IMU data structure with timestamps
     */
    struct IMUData
    {
        // Quaternion - orientation (could drift in Yaw)
        Quaternion quaternion;

        // Acceleration in m/s² - total acceleration including gravity (body frame, accurate)
        Vector3D acceleration;

        // Angular velocity in rad/s (body frame, accurate)
        Vector3D angular_velocity;

        // Individual timestamps for each sensor component (microseconds, from esp_timer)
        int64_t quaternion_timestamp_us;
        int64_t acceleration_timestamp_us;
        int64_t angular_velocity_timestamp_us;
        int64_t last_data_time_us; // Timestamp of the last data update (microseconds)
    };

    /**
     * @brief Initialize IMU with specified configuration
     * 
     * No spi_mutex needed anymore - ESP32 handles it internally.
     * No extra tasks needed - timing is handled in BNO08x library.
     * 
     * @param cs_pin SPI chip select pin
     * @param int_pin Interrupt pin
     * @param rst_pin Reset pin
     * @param sample_interval_ms Sample interval in milliseconds (default 10ms)
     * @param task_priority Priority of the BNO08x SPI task (default 6)
     * @return true if initialization successful
     * 
     * @warning You need to initialise the SPI2_HOST bus before calling this function.
     */
    bool Initialize(uint8_t cs_pin, uint8_t int_pin, uint8_t rst_pin, uint32_t sample_interval_ms = 10, uint32_t task_priority = 6);

    /**
     * @brief Get current IMU data with extrapolation to current time
     * 
     * Extrapolates the stored sensor data to the current time using
     * angular velocity for more accurate real-time readings.
     * 
     * @return IMUData structure with extrapolated sensor readings
     */
    IMUData GetData();

    /**
     * @brief Get raw IMU data without extrapolation
     * 
     * Returns the most recent sensor data exactly as received,
     * without any time-based extrapolation.
     * 
     * @return IMUData structure with raw sensor readings
     */
    IMUData GetRawData();

    /**
     * @brief Check if IMU has been initialized
     * @return true if initialized, false otherwise
     */
    bool InitializedQ();

    /**
     * @brief Get pointer to BNO08x device for direct access
     * @return Pointer to BNO08x device, or NULL if not initialized
     * @warning Use with caution - direct manipulation of the device can lead to undefined behavior.
     */
    void* GetIMUDevice();

} // namespace IMUManager

#endif // IMUMANAGER_HPP
