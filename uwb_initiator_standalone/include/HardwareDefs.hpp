/**
 * @file HardwareDefs.hpp
 * @brief Hardware pin definitions for UWB Initiator
 */

#ifndef HARDWARE_DEFS_HPP
#define HARDWARE_DEFS_HPP

// ========================================
// SPI Pin Definitions
// ========================================
// Modify these pins according to your ESP32 board and wiring
#define SPI_MOSI_PIN    11
#define SPI_MISO_PIN    13
#define SPI_CLK_PIN     12

// ========================================
// UWB DW1000 Pin Definitions
// ========================================
#define UWB_CS_PIN      9
#define UWB_IRQ_PIN     14
#define UWB_RST_PIN     15

// ========================================
// IMU Pin Definitions (if IMU is present)
// ========================================
#define IMU_CS_PIN      10

// ========================================
// LED Pin Definitions (for Blink function)
// ========================================
// RGB LED pins (active LOW)
#define LED_R_PIN       18
#define LED_G_PIN       17
#define LED_B_PIN       16

#endif // HARDWARE_DEFS_HPP
