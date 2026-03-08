# IMUManager Library

> **⚠️ WARNING: This documentation is AI-Generated Content (AIGC)**  
> Please verify all technical specifications, code implementations, and system designs independently. Do not rely solely on this documentation for critical system implementations.

## Overview

IMUManager is a thread-safe, high-performance library for managing IMU (Inertial Measurement Unit) data from the BNO08x sensor on ESP32 platforms. It provides a simplified interface for accessing quaternion orientation, acceleration, angular velocity, and gravity vector data at 500Hz update rates.

## Features

- **High-frequency data acquisition**: 500Hz (2ms) update rate
- **Thread-safe operation**: Uses FreeRTOS semaphores for concurrent access
- **Automatic sensor management**: Background task handles all sensor communication
- **Simple API**: Easy-to-use interface similar to other sensor libraries
- **Robust error handling**: Automatic reset detection and reconfiguration
- **Memory efficient**: Minimal memory footprint with double-buffering

## Hardware Requirements

- **IMU Sensor**: BNO08x series (BNO080, BNO085, BNO086)
- **Platform**: ESP32 (S2, S3, or standard ESP32)
- **Interface**: SPI communication
- **Pins Required**:
  - CS (Chip Select)
  - INT (Interrupt) 
  - RST (Reset)
  - Standard SPI pins (MOSI, MISO, SCK)

## Dependencies

- **SparkFun BNO08x Arduino Library**: Core sensor communication
- **FreeRTOS**: Task management and synchronization
- **Arduino ESP32 Core**: Platform-specific functions

## Installation

1. Copy the `IMUManager` folder to your PlatformIO `lib/` directory
2. Include the header in your code: `#include <IMUManager.hpp>`
3. Ensure BNO08x library is available in your project

## API Reference

### Data Structures

#### `IMUData`
```cpp
struct IMUData {
    float quaternion[4];        // [w, x, y, z] - Orientation quaternion (normalized)
    float acceleration[3];      // [ax, ay, az] - Total acceleration in m/s² (body frame)
    float angular_velocity[3];  // [wx, wy, wz] - Angular velocity in rad/s (body frame)  
    float gravity[3];          // [gx, gy, gz] - Gravity vector in m/s² (body frame)
    int64_t timestamp_us;      // Timestamp when data was acquired (microseconds)
    bool data_valid;           // Whether the data is valid and fresh
};
```

**Coordinate System**: All data is in the sensor's body frame
- **X-axis**: Forward (sensor marking direction)
- **Y-axis**: Right (90° clockwise from X when viewed from above)
- **Z-axis**: Down (completing right-hand coordinate system)

### Functions

#### `bool Initialize(uint8_t cs_pin, uint8_t int_pin, uint8_t rst_pin, SemaphoreHandle_t spi_mutex, uint32_t sample_interval_ms = 10, uint32_t task_priority = 6)`
Initializes the IMU manager and starts background data acquisition.

**Parameters:**
- `cs_pin`: SPI chip select pin number
- `int_pin`: Interrupt pin number
- `rst_pin`: Reset pin number
- `spi_mutex`: FreeRTOS mutex handle for SPI bus protection (required)
- `sample_interval_ms`: Background update interval in milliseconds (default 10ms, do not set lower than this for rot/accel/gyro measurements!)
- `task_priority`: Priority for the background task (default 6)

**Returns:** `true` if initialization successful, `false` otherwise

**Important:** The `spi_mutex` parameter is required to protect the SPI bus from concurrent access by multiple devices. You must create and provide a valid mutex before calling `Initialize()`.

**Note:** Ensure SPI2_HOST is initialized before calling `Initialize()`.

**Example:**
```cpp
// Create SPI mutex (do this once, typically in setup)
SemaphoreHandle_t spi_mutex = xSemaphoreCreateMutex();

if (spi_mutex == nullptr) {
    Serial.println("Failed to create SPI mutex!");
    return;
}

// Initialize IMU with mutex protection
if (!IMUManager::Initialize(5, 4, 2, spi_mutex)) {
    Serial.println("IMU initialization failed!");
    return;
}
```

#### `IMUData GetData()`
Retrieves the most recent IMU data in a thread-safe manner.

**Returns:** `IMUData` structure with current sensor readings

**Thread Safety:** Yes - uses semaphore protection

**Example:**
```cpp
IMUManager::IMUData data = IMUManager::GetData();
if (data.data_valid) {
    Serial.printf("Quaternion: [%.3f, %.3f, %.3f, %.3f]\n", 
                  data.quaternion[0], data.quaternion[1], 
                  data.quaternion[2], data.quaternion[3]);
}
```

#### `IMUData GetRawData()`
Retrieves raw sensor data (identical to `GetData()` in this simplified implementation).

**Returns:** `IMUData` structure with current sensor readings

**Note:** In this version, raw data is the same as processed data. Future versions may add filtering differences.

## Configuration

### Update Rate
```cpp
constexpr uint32_t UPDATE_RATE_MS = 2; // 500Hz update rate
```

The update rate is fixed at 2ms (500Hz) for optimal performance. This provides smooth data for high-frequency control applications while maintaining system stability.

## Usage Examples

### Basic Usage
```cpp
#include <IMUManager.hpp>
#include <HardwareDefs.hpp>

// Global SPI mutex (shared across all SPI devices)
SemaphoreHandle_t spi_mutex = nullptr;

void setup() {
    Serial.begin(115200);
    
    // Create SPI mutex for bus protection
    spi_mutex = xSemaphoreCreateMutex();
    if (spi_mutex == nullptr) {
        Serial.println("Failed to create SPI mutex!");
        while(1) vTaskDelay(1000);
    }
    
    // Initialize IMU with mutex protection
    if (!IMUManager::Initialize(IMU_CS_PIN, IMU_INTN_PIN, IMU_RST_PIN, spi_mutex)) {
        Serial.println("Failed to initialize IMU!");
        while(1) vTaskDelay(1000);
    }
    
    Serial.println("IMU initialized successfully");
}

void loop() {
    IMUManager::IMUData imu_data = IMUManager::GetData();
    
    if (imu_data.data_valid) {
        // Print orientation (quaternion)
        Serial.printf("Orientation: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
                      imu_data.quaternion[0], imu_data.quaternion[1],
                      imu_data.quaternion[2], imu_data.quaternion[3]);
        
        // Print angular velocity
        Serial.printf("Angular Velocity: wx=%.3f, wy=%.3f, wz=%.3f rad/s\n",
                      imu_data.angular_velocity[0], imu_data.angular_velocity[1],
                      imu_data.angular_velocity[2]);
        
        // Print acceleration
        Serial.printf("Acceleration: ax=%.3f, ay=%.3f, az=%.3f m/s²\n",
                      imu_data.acceleration[0], imu_data.acceleration[1],
                      imu_data.acceleration[2]);
        
        // Data age check
        int64_t data_age_ms = (esp_timer_get_time() - imu_data.timestamp_us) / 1000;
        Serial.printf("Data age: %lld ms\n", data_age_ms);
    } else {
        Serial.println("IMU data not valid");
    }
    
    vTaskDelay(100); // 10Hz display rate
}
```

### Integration with Control Systems
```cpp
void controlLoop() {
    IMUManager::IMUData imu_data = IMUManager::GetData();
    
    // Check data freshness (should be < 50ms for real-time control)
    int64_t data_age_us = esp_timer_get_time() - imu_data.timestamp_us;
    
    if (imu_data.data_valid && data_age_us < 50000) { // 50ms timeout
        // Use IMU data for control
        processOrientationControl(imu_data.quaternion);
        processStabilization(imu_data.angular_velocity);
        processAccelerationFeedback(imu_data.acceleration);
    } else {
        // Handle stale or invalid data
        enterSafeMode();
    }
}
```

### Quaternion to Euler Conversion
```cpp
void quaternionToEuler(const float q[4], float& roll, float& pitch, float& yaw) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2 * (w * y - z * x);
    if (abs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

// Usage:
IMUManager::IMUData data = IMUManager::GetData();
if (data.data_valid) {
    float roll, pitch, yaw;
    quaternionToEuler(data.quaternion, roll, pitch, yaw);
    Serial.printf("Euler angles: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\n",
                  roll * 180/M_PI, pitch * 180/M_PI, yaw * 180/M_PI);
}
```

## Performance Characteristics

### Timing
- **Update Rate**: 500Hz (2ms intervals)
- **Data Latency**: < 3ms from sensor to application
- **API Call Time**: < 100μs for `GetData()`
- **Memory Usage**: ~1KB RAM for buffers and task stack

### Thread Safety
- **Concurrent Access**: Safe for multiple threads (data access protected by lock-free reading)
- **SPI Bus Protection**: Uses FreeRTOS mutex to prevent concurrent SPI access
- **Background Task**: Runs on Core 1 with configurable priority (default 6)
- **Stack Size**: 4KB for update task
- **Mutex Timeout**: Waits indefinitely for SPI mutex (portMAX_DELAY)

## Troubleshooting

### Common Issues

#### "Failed to initialize BNO08x IMU"
- Check SPI wiring connections
- Verify pin definitions in HardwareDefs.hpp
- Ensure proper power supply (3.3V)
- Verify SPI mutex was created and passed correctly
- Check for conflicting SPI device usage
- Ensure no other device is holding the SPI mutex during initialization

#### "IMU data not valid"
- Check if Initialize() was called successfully
- Verify IMU is not in reset state
- Check for I2C address conflicts (if mixed interface usage)

#### Data appears stale or old
- Check data_age using timestamp comparison
- Verify background task is running (check Serial output)
- Ensure SPI bus is not being blocked by other devices

#### Erratic readings
- Check for electromagnetic interference
- Verify stable power supply
- Allow adequate warm-up time (30 seconds)
- Check mounting and vibration isolation

### Debug Information

The library can output debug information to Serial for troubleshooting. To enable debug output:

1. Open `lib/IMUManager/src/IMUManager.cpp`
2. Change `#define DEBUG_IMU 0` to `#define DEBUG_IMU 1`
3. Rebuild the project

When enabled, you'll see messages like:
```
IMUManager: Initializing...
IMUManager: SPI and GPIO initialized
IMUManager: BNO08x detected and initialized
IMUManager: Sensor reports enabled
IMUManager: Initialization complete (direct read mode)
```

**Note**: Debug output is disabled by default to minimize performance impact. Enable only when troubleshooting initialization issues.

### Performance Monitoring
```cpp
// Monitor data freshness
IMUManager::IMUData data = IMUManager::GetData();
int64_t age_ms = (esp_timer_get_time() - data.timestamp_us) / 1000;
if (age_ms > 10) {
    Serial.printf("Warning: IMU data is %lld ms old\n", age_ms);
}
```

## Implementation Notes

### Architecture
- **Background Task**: Dedicated FreeRTOS task handles all sensor communication
- **Lock-Free Data Reading**: Uses atomic update counter for thread-safe data access without blocking
- **SPI Bus Protection**: External mutex ensures only one device accesses SPI bus at a time
- **Interrupt Driven**: Uses hardware interrupt for efficient data acquisition
- **Data Extrapolation**: Compensates for read latency using angular velocity

### Sensor Configuration
The library automatically configures the BNO08x for:
- Rotation Vector (quaternion): 500Hz
- Accelerometer: 500Hz  
- Gyroscope (calibrated): 500Hz
- Gravity Vector: 500Hz

### Memory Management
- Static allocation for all buffers
- No dynamic memory allocation during operation
- Minimal heap usage (only during initialization)

## License

This library is part of the NewRollbot project. See project license for details.

## Version History

- **v2.0**: Enhanced thread safety and configurability
    - Added SPI bus mutex protection for multi-device systems
    - Configurable sample interval and task priority
    - Lock-free data reading with extrapolation
    - Individual sensor component timestamps
    - Improved robustness for concurrent SPI access

- **v1.0**: Initial simplified implementation
    - Basic sensor data acquisition
    - Thread-safe API
    - 500Hz update rate
    - Background task management
