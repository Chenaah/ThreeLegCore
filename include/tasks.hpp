#pragma once
#include "tasks/motor_task.hpp"
#include "tasks/imu_task.hpp"
#include "tasks/comm_task.hpp"
#include "tasks/monitor_task.hpp"
#include "tasks/utils.hpp"
#include "identity.hpp"
#include <esp_task_wdt.h>

#define SPI_SCK GPIO_NUM_12
#define SPI_MISO GPIO_NUM_13
#define SPI_MOSI GPIO_NUM_11

#define BNO08X_CS   GPIO_NUM_10
#define BNO08X_INT  GPIO_NUM_4
#define BNO08X_RST  GPIO_NUM_5
#define BNO086_WAKE GPIO_NUM_6

#define DWM1000_CS GPIO_NUM_9
#define DWM1000_RST GPIO_NUM_15  // reset pin
#define DWM1000_IRQ GPIO_NUM_14  // irq pin

#define SDA GPIO_NUM_8
#define SCL GPIO_NUM_7

#define CAN_TX_PIN GPIO_NUM_1
#define CAN_RX_PIN GPIO_NUM_2
#define MOTOR_SWITCH GPIO_NUM_38

#define DELAY_PERIOD 2 //10 // 2
#define DEBUG_ENABLED 0

#if DEBUG_ENABLED
#define DEBUG_PRINT(c) Serial.println(c)
#else
#define DEBUG_PRINT(c) 0
#endif