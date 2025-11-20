#include "tasks.hpp"
#include "identity.hpp"

void setup() {
    Serial.begin(115200);
    Serial.println("------ Hello! ------");
    send_led_message(LED_MSG_POWER_ON);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    pinMode(BNO08X_CS, OUTPUT);
    digitalWrite(BNO08X_CS, HIGH);   // Deselect BNO08x
    pinMode(DWM1000_CS, OUTPUT);
    digitalWrite(DWM1000_CS, HIGH);  // Deselect DW1000
    pinMode(MOTOR_SWITCH, OUTPUT);
    digitalWrite(MOTOR_SWITCH, LOW); // Turn on motor

    set_module_id();
    
    TaskHandle_t xHandle;

    xTaskCreatePinnedToCore(Task::IMUTask::run, "IMU_Task", 10000, nullptr, 0, &xHandle, 1);
    xTaskCreatePinnedToCore(Task::MotorTask::run, "Motor_Task", 10000, nullptr, 0, &xHandle, 1);
    xTaskCreatePinnedToCore(Task::CommTask::run, "Comm_Task", 10000, nullptr, 0, &xHandle, 0);
    xTaskCreatePinnedToCore(Task::MonitorTask::run, "Monitor_Task", 10000, nullptr, 0, &xHandle, 0);

    vTaskDelete(NULL);
}

void loop() {
    vTaskDelete(NULL);
}