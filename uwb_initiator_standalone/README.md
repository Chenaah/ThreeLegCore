# UWB Initiator Standalone Project

This is a standalone PlatformIO project for the UWB DS-TWR (Double-Sided Two-Way Ranging) Initiator using the DW1000 module.

## Hardware Requirements

- ESP32 development board (ESP32, ESP32-S3, etc.)
- DW1000 UWB module
- (Optional) RGB LED for status indication
- (Optional) IMU module if you have one on the same SPI bus

## Pin Configuration

Default pin configuration is defined in `include/HardwareDefs.hpp`. **Please modify these pins according to your actual hardware wiring:**

### SPI Pins
- MOSI: GPIO 23
- MISO: GPIO 19
- CLK: GPIO 18

### UWB DW1000 Pins
- CS (Chip Select): GPIO 5
- IRQ (Interrupt): GPIO 4
- RST (Reset): GPIO 17

### LED Pins (Optional)
- Red LED: GPIO 25
- Green LED: GPIO 26
- Blue LED: GPIO 27

Note: LEDs are configured as active LOW (LOW = ON, HIGH = OFF)

### IMU Pins (If Present)
- IMU CS: GPIO 15

## Building and Flashing

### 1. Install PlatformIO

If you haven't installed PlatformIO yet:
```bash
# Using pip
pip install platformio

# Or using the PlatformIO IDE extension in VSCode
```

### 2. Configure for Your Board

Edit `platformio.ini` if you're using a different ESP32 board:

For ESP32-S3:
```ini
[env:esp32s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
lib_ldf_mode = deep
monitor_speed = 115200
build_flags = 
    -I include
    -std=gnu++11
```

For other boards, check available boards at: https://docs.platformio.org/en/latest/boards/index.html#espressif-32

### 3. Update Pin Definitions

Edit `include/HardwareDefs.hpp` to match your hardware wiring.

### 4. Build the Project

```bash
cd uwb_initiator_standalone
pio run
```

### 5. Flash to ESP32

```bash
pio run --target upload
```

Or specify a port:
```bash
pio run --target upload --upload-port /dev/ttyUSB0
```

### 6. Monitor Serial Output

```bash
pio device monitor
```

Or combined upload and monitor:
```bash
pio run --target upload && pio device monitor
```

## Expected Output

When the initiator is running correctly, you should see:

```
=== UWB DS-TWR Initiator Test ===
Initializing...
UWB Initiator initialized successfully
Ranging started with 20ms interval
Initiator is now sending ranging polls...

Initiator active and ranging...
Initiator active and ranging...
...
```

The initiator will continuously send ranging polls every 20ms. You'll need a corresponding responder device to complete the ranging operation and get distance measurements.

## Troubleshooting

### SPI Initialization Failed
- Check that your SPI pin definitions in `HardwareDefs.hpp` are correct
- Ensure no other device is using the same SPI bus without proper CS control
- If you have an IMU on the same bus, make sure `IMU_CS_PIN` is set correctly

### UWB Initialization Failed
- Verify UWB_CS_PIN, UWB_IRQ_PIN, and UWB_RST_PIN connections
- Check power supply to the DW1000 module (3.3V)
- Ensure the DW1000 module is properly seated/connected
- Try adding a small delay before initialization

### No Output on Serial Monitor
- Check baud rate is set to 115200
- Verify the correct USB port is selected
- Try pressing the reset button on the ESP32

## Project Structure

```
uwb_initiator_standalone/
├── platformio.ini          # PlatformIO configuration
├── include/
│   ├── HardwareDefs.hpp   # Pin definitions (MODIFY THIS)
│   └── Blink.hpp          # LED blink utility
├── src/
│   └── main.cpp           # Main application code
└── lib/
    ├── DW1000Manager/     # UWB ranging management library
    └── DWM1000_ESP32/     # DW1000 hardware driver
```

## Modifying the Code

### Change Ranging Interval

In `src/main.cpp`, modify the interval parameter (in milliseconds):

```cpp
// Start ranging with 20ms interval
UWBRanging::Initiator::Begin(20);  // Change 20 to desired interval
```

### Disable LED Blinking

Comment out the Blink line in `setup()`:

```cpp
// Blink(500, 3, true, true, true);
```

### Disable IMU CS Control

If you don't have an IMU, comment out these lines:

```cpp
// pinMode(IMU_CS_PIN, OUTPUT);
// digitalWrite(IMU_CS_PIN, HIGH);
```

## Next Steps

To create a complete ranging system, you'll need:
1. A responder device (see DW1000Manager examples)
2. Proper antenna configuration on both devices
3. Distance calculation implementation

## License

This project uses libraries from the parent metamachine_esp32 project. Please refer to individual library licenses.

## Support

For issues related to:
- DW1000 hardware: Check DW1000Manager library documentation
- PlatformIO: Visit https://docs.platformio.org
- ESP32: Visit https://docs.espressif.com
