/**
 * @file NewRollbot_IMU_Test.cpp
 * @brief Test program for IMU functionality
 *
 * This test initializes the BNO08x IMU and prints sensor data
 * every 500ms to verify proper operation.
 */

#include <Arduino.h>
#include <IMUManager.hpp>
#include <HardwareDefs.hpp>
#include <Blink.hpp>
#include <driver/spi_master.h>

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);
    vTaskDelay(1000);
    Blink(500, 3);

    Serial.println("\n=== IMU Test Starting ===");

    Serial.println("Initializing SPI...");

    // this is only necessary on my test board where another UWB is present.
    pinMode(UWB_CS_PIN, OUTPUT);
    digitalWrite(UWB_CS_PIN, HIGH);

    // Initialize SPI bus before initializing any other thing
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = (gpio_num_t)SPI_MOSI_PIN,
        .miso_io_num = (gpio_num_t)SPI_MISO_PIN,
        .sclk_io_num = (gpio_num_t)SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
        .flags = 0,
        .intr_flags = 0};

    if (spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO) != ESP_OK)
    {
        Serial.println("ERROR: Failed to initialize SPI bus!");
        while (1)
        {
            vTaskDelay(1000);
        }
    }

    Serial.println("Initializing BNO08x IMU...");

    // Initialize IMU with 10ms sample interval (100Hz)
    bool success = IMUManager::Initialize(
        IMU_CS_PIN,
        IMU_INTN_PIN,
        IMU_RST_PIN,
        10 // 10ms sample interval
    );

    if (!success)
    {
        Serial.println("ERROR: Failed to initialize IMU!");
        Serial.println("System halted.");
        while (1)
        {
            vTaskDelay(1000);
        }
    }

    Serial.println("IMU initialized successfully!");
    Serial.println("Starting data acquisition...\n");

    // Wait a bit for sensor to stabilize
    vTaskDelay(100);
}

void loop()
{
    // Check if IMU is initialized
    if (!IMUManager::InitializedQ())
    {
        Serial.println("ERROR: IMU not initialized!");
        vTaskDelay(500);
        return;
    }

    // Get extrapolated IMU data
    IMUManager::IMUData data = IMUManager::GetData();

    // Print timestamp information
    Serial.println("=== IMU Data ===");

    // Print quaternion (orientation)
    Serial.printf("Quaternion [timestamp: %lld us]:\n", data.quaternion_timestamp_us);
    Serial.printf("  w=%.4f, x=%.4f, y=%.4f, z=%.4f\n",
                  data.quaternion.w,
                  data.quaternion.x,
                  data.quaternion.y,
                  data.quaternion.z);

    // Print acceleration (m/s²)
    Serial.printf("Acceleration [timestamp: %lld us]:\n", data.acceleration_timestamp_us);
    Serial.printf("  x=%.3f, y=%.3f, z=%.3f m/s²\n",
                  data.acceleration.x,
                  data.acceleration.y,
                  data.acceleration.z);

    // Print magnitude
    float accel_mag = data.acceleration.magnitude();
    Serial.printf("  Magnitude: %.3f m/s² (should be ~9.81 when stationary)\n", accel_mag);

    // Print angular velocity (rad/s)
    Serial.printf("Angular Velocity [timestamp: %lld us]:\n", data.angular_velocity_timestamp_us);
    Serial.printf("  x=%.3f, y=%.3f, z=%.3f rad/s\n",
                  data.angular_velocity.x,
                  data.angular_velocity.y,
                  data.angular_velocity.z);

    // Convert to degrees/s for easier interpretation
    Serial.printf("  x=%.2f, y=%.2f, z=%.2f °/s\n",
                  data.angular_velocity.x * 180.0f / PI,
                  data.angular_velocity.y * 180.0f / PI,
                  data.angular_velocity.z * 180.0f / PI);

    Serial.println();

    // Wait 500ms before next reading
    vTaskDelay(500);
}
