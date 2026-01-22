#include "tasks.hpp"
#include "identity.hpp"
#include <driver/spi_master.h>

void setup() {
    Serial.begin(115200);
    Serial.println("------ Hello! ------");
    send_led_message(LED_MSG_POWER_ON);

    // Initialize SPI bus using ESP-IDF for proper bus sharing
    // This enables hardware locking between UWB and IMU
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
        .flags = 0,
        .intr_flags = 0
    };
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.printf("SPI bus initialization failed: %d\n", ret);
        while (1)
        {
            vTaskDelay(1000);
        }
    } else {
        Serial.println("SPI bus initialized successfully!");
    }

    // Deselect both SPI devices initially
    pinMode(BNO08X_CS, OUTPUT);
    digitalWrite(BNO08X_CS, HIGH);   // Deselect BNO08x
    pinMode(DWM1000_CS, OUTPUT);
    digitalWrite(DWM1000_CS, HIGH);  // Deselect DW1000
    pinMode(MOTOR_SWITCH, OUTPUT);
    digitalWrite(MOTOR_SWITCH, LOW); // Turn on motor

    set_module_id();

    // Initialize UWB as Responder (before creating tasks)
    // Parameters: CS, IRQ, RST
    if (!Task::UWBTask::initialize(DWM1000_CS, DWM1000_IRQ, DWM1000_RST)) {
        Serial.println("ERROR: UWB initialization failed!");
    }
    
    TaskHandle_t xHandle;

    // Create tasks with appropriate priorities
    // UWB has higher priority for timing sensitivity (priority 2)
    xTaskCreatePinnedToCore(Task::UWBTask::run, "UWB_Task", 8192, nullptr, 2, &xHandle, 1);
    xTaskCreatePinnedToCore(Task::IMUTask::run, "IMU_Task", 10000, nullptr, 1, &xHandle, 1);
    xTaskCreatePinnedToCore(Task::MotorTask::run, "Motor_Task", 10000, nullptr, 1, &xHandle, 1);
    xTaskCreatePinnedToCore(Task::CommTask::run, "Comm_Task", 10000, nullptr, 0, &xHandle, 0);
    xTaskCreatePinnedToCore(Task::MonitorTask::run, "Monitor_Task", 10000, nullptr, 0, &xHandle, 0);

    vTaskDelete(NULL);
}

void loop() {
    vTaskDelete(NULL);
}