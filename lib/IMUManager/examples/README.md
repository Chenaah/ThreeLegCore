# IMUManager Examples

> **⚠️ WARNING: This documentation is AI-Generated Content (AIGC)**  
> Please verify all technical specifications, code implementations, and system designs independently. Do not rely solely on this documentation for critical system implementations.

This directory contains example applications demonstrating IMUManager library usage.

## Important Setup Requirements

### SPI Bus Pre-initialization

**CRITICAL**: Before using any IMUManager functions, you **must** initialize the SPI bus (SPI2_HOST):

```cpp
// Step 1: Initialize SPI bus
spi_bus_config_t spi_cfg = {
    .mosi_io_num = SPI_MOSI_PIN,
    .miso_io_num = SPI_MISO_PIN,
    .sclk_io_num = SPI_CLK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 1024,
    .flags = 0,
    .intr_flags = 0
};
spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);

// Step 2: Create SPI mutex for bus protection
SemaphoreHandle_t spi_mutex = xSemaphoreCreateMutex();

// Step 3: Initialize IMU device
IMUManager::Initialize(IMU_CS_PIN, IMU_INTN_PIN, IMU_RST_PIN, spi_mutex);
```

### Initialize() Parameters

The `Initialize()` function requires:
- **cs_pin**: SPI chip select for IMU
- **int_pin**: Interrupt pin from IMU  
- **rst_pin**: Reset pin for IMU
- **spi_mutex**: FreeRTOS mutex for SPI bus protection
- **sample_interval_ms** (optional): Background update interval in milliseconds (default 10ms)
- **task_priority** (optional): Priority for the background task (default 6)

**Note**: SPI bus pins (MOSI, MISO, SCLK) are NOT needed since the bus is already initialized.

## Available Examples

### 1. NewRollbot_IMU_Test.cpp

**Location**: `/lib/IMUManager/examples/NewRollbot_IMU_Test.cpp`

**Purpose**: Demonstrates IMU initialization and continuous data acquisition with formatted output

**Features**:
- Initializes SPI bus and IMU sensor
- Prints quaternion orientation data
- Displays acceleration with magnitude calculation
- Shows angular velocity in both rad/s and °/s
- Includes individual component timestamps
- Runs at 2Hz display rate (500ms intervals)

**Code**:
```cpp
#include <Arduino.h>
#include <IMUManager.hpp>
#include <HardwareDefs.hpp>
#include <Blink.hpp>
#include <driver/spi_master.h>

void setup() {
    Serial.begin(115200);
    vTaskDelay(1000);
    
    Serial.println("\n=== IMU Test Starting ===");
    Serial.println("Initializing SPI...");
    
    // Initialize SPI bus
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = (gpio_num_t)SPI_MOSI_PIN,
        .miso_io_num = (gpio_num_t)SPI_MISO_PIN,
        .sclk_io_num = (gpio_num_t)SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
        .flags = 0,
        .intr_flags = 0
    };
    
    if (spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO) != ESP_OK) {
        Serial.println("ERROR: Failed to initialize SPI bus!");
        while (1) vTaskDelay(1000);
    }
    
    // Create SPI mutex
    SemaphoreHandle_t spi_mutex = xSemaphoreCreateMutex();
    
    Serial.println("Initializing BNO08x IMU...");
    
    // Initialize IMU
    if (!IMUManager::Initialize(IMU_CS_PIN, IMU_INTN_PIN, IMU_RST_PIN, spi_mutex)) {
        Serial.println("ERROR: Failed to initialize IMU!");
        while (1) vTaskDelay(1000);
    }
    
    Serial.println("IMU initialized successfully!");
    Serial.println("Starting data acquisition...\n");
}

void loop() {
    if (!IMUManager::InitializedQ()) {
        Serial.println("ERROR: IMU not initialized!");
        vTaskDelay(500);
        return;
    }
    
    IMUManager::IMUData data = IMUManager::GetData();
    
    Serial.println("=== IMU Data ===");
    
    // Print quaternion
    Serial.printf("Quaternion [timestamp: %lld us]:\n", data.quaternion_timestamp_us);
    Serial.printf("  w=%.4f, x=%.4f, y=%.4f, z=%.4f\n",
                  data.quaternion.w, data.quaternion.x, 
                  data.quaternion.y, data.quaternion.z);
    
    // Print acceleration
    Serial.printf("Acceleration [timestamp: %lld us]:\n", data.acceleration_timestamp_us);
    Serial.printf("  x=%.3f, y=%.3f, z=%.3f m/s²\n",
                  data.acceleration.x, data.acceleration.y, data.acceleration.z);
    
    float accel_mag = data.acceleration.magnitude();
    Serial.printf("  Magnitude: %.3f m/s²\n", accel_mag);
    
    // Print angular velocity
    Serial.printf("Angular Velocity [timestamp: %lld us]:\n", data.angular_velocity_timestamp_us);
    Serial.printf("  x=%.3f, y=%.3f, z=%.3f rad/s\n",
                  data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z);
    
    Serial.printf("  x=%.2f, y=%.2f, z=%.2f °/s\n",
                  data.angular_velocity.x * 180.0f / PI,
                  data.angular_velocity.y * 180.0f / PI,
                  data.angular_velocity.z * 180.0f / PI);
    
    Serial.println();
    vTaskDelay(500);
}
```

**Expected Behavior**:
1. Prints initialization messages for SPI and IMU
2. Displays IMU data every 500ms with timestamps
3. Shows quaternion, acceleration (with magnitude), and angular velocity
4. Acceleration magnitude should be ~9.81 m/s² when stationary
5. Angular velocity should be near zero when not rotating

**Troubleshooting**:
- If SPI initialization fails, check SPI pin definitions in HardwareDefs.hpp
- If IMU initialization fails, verify IMU connections and power supply
- Check for SPI bus conflicts with other devices
- Ensure SPI mutex is created before IMU initialization
- Verify IMU_CS_PIN is not held low by another device during init.
