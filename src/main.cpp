#include "tasks.hpp"
#include "identity.hpp"
#include "LocalPolicy.hpp"
#include <driver/spi_master.h>

// Global onboard-model instance (accessible from other tasks if needed)
LocalPolicy onboard_model;

void setup() {
    Serial.begin(115200);
    Serial.println("------ Hello! ------");
    send_led_message(LED_MSG_POWER_ON);

    // Initialize SPI bus using ESP-IDF API (required by the new BNO08x driver)
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = (gpio_num_t)SPI_MOSI,
        .miso_io_num = (gpio_num_t)SPI_MISO,
        .sclk_io_num = (gpio_num_t)SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
        .flags = 0,
        .intr_flags = 0
    };
    esp_err_t spi_ret = spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO);
    if (spi_ret != ESP_OK && spi_ret != ESP_ERR_INVALID_STATE) {
        Serial.printf("ERROR: Failed to initialize SPI bus: %d\n", spi_ret);
    }

    pinMode(BNO08X_CS, OUTPUT);
    digitalWrite(BNO08X_CS, HIGH);   // Deselect BNO08x
    pinMode(DWM1000_CS, OUTPUT);
    digitalWrite(DWM1000_CS, HIGH);  // Deselect DW1000
    pinMode(MOTOR_SWITCH, OUTPUT);
    digitalWrite(MOTOR_SWITCH, LOW); // Turn on motor

    set_module_id();

    // --- Load onboard model from LittleFS ---
    Serial.println("\n====== Loading Onboard Model ======");

    // Fallback only for sanity checks and offline debugging.
    // During live hierarchical control, the PC sends the joint offset explicitly
    // in every MotorCommand, so the ESP32 does not depend on hardcoded mapping.
    onboard_model.set_module_index(0);

    if (onboard_model.load_from_littlefs()) {
        // Run sanity check with reference values generated alongside the deploy headers.
        bool sanity_ok = onboard_model.run_sanity_check(
            DEPLOY_SANITY_EXPECTED_MEAN,
            DEPLOY_SANITY_EXPECTED_ACTION
        );
        Task::onboard_model_loaded = true;
        if (!onboard_model.is_build_hash_match()) {
            Task::policy_error_code = Task::POLICY_ERROR_HASH_MISMATCH;
            send_led_message(LED_MSG_POLICY_ERROR);
            Serial.println("[Setup] Onboard-model hash mismatch against build header.");
        } else if (sanity_ok) {
            Serial.println("[Setup] Onboard model loaded: waiting for PC control_mode to activate NN.");
        } else {
            Task::policy_error_code = Task::POLICY_ERROR_SANITY_FAILED;
            send_led_message(LED_MSG_POLICY_ERROR);
            Serial.println("[Setup] Onboard-model sanity check FAILED.");
        }
    } else {
        Task::policy_error_code = Task::POLICY_ERROR_NOT_LOADED;
        Serial.println("WARNING: Onboard model not loaded, running without NN");
    }
    Serial.println("==================================\n");

    TaskHandle_t xHandle;

    xTaskCreatePinnedToCore(Task::IMUTask::run, "IMU_Task", 10000, nullptr, 1, &xHandle, 1);
    xTaskCreatePinnedToCore(Task::MotorTask::run, "Motor_Task", 10000, nullptr, 3, &xHandle, 1);   // Highest priority for stable motor control
    xTaskCreatePinnedToCore(Task::CommTask::run, "Comm_Task", 10000, nullptr, 2, &xHandle, 0);     // Medium priority for timely command delivery
    xTaskCreatePinnedToCore(Task::MonitorTask::run, "Monitor_Task", 10000, nullptr, 0, &xHandle, 0);
    xTaskCreatePinnedToCore(Task::OTATask::run, "OTA_Task", 6000, nullptr, 3, &xHandle, 0);

    vTaskDelete(NULL);
}

void loop() {
    vTaskDelete(NULL);
    // if (!onboard_model.is_loaded()) {
    //     delay(5000);
    //     return;
    // }

    // // Use the latest received command context from comm_task if available,
    // // otherwise fall back to hardcoded demo values.
    // std::array<float, Task::COMMAND_CONTEXT_DIM> command_context;
    // bool context_from_network = false;

    // // Task::received_joint_id >= 0 means a real command with joint_id has arrived
    // if (Task::received_joint_id >= 0) {
    //     for (size_t i = 0; i < Task::COMMAND_CONTEXT_DIM; i++) {
    //         command_context[i] = Task::received_command_context[i];
    //     }
    //     context_from_network = true;
    // } else {
    //     // Demo: hardcoded command context when no network command has been received
    //     command_context = {0.5f, -0.3f, 0.1f, 0.0f, 0.2f, -0.1f, 0.4f, -0.2f};
    // }

    // std::array<float, LOCAL_OBS_DIM> obs = {};  // zeros = placeholder sensor data

    // // forward_nn: raw NN output in [DEPLOY_ACTION_LOW, DEPLOY_ACTION_HIGH], no offset
    // float nn_action = onboard_model.forward_nn(command_context, obs);

    // // select_action: NN output + default_dof_pos[module_idx] = motor target
    // float motor_target = onboard_model.select_action(command_context, obs);

    // if (context_from_network) {
    //     Serial.printf("[Loop] Using network context (joint_id=%d): [%.3f, %.3f, %.3f, ...]\n",
    //                   Task::received_joint_id, command_context[0], command_context[1], command_context[2]);
    // } else {
    //     Serial.println("[Loop] Running model with hardcoded demo context...");
    // }
    // Serial.printf("[Loop] NN action:     %.6f (in [%.3f, %.3f])\n",
    //               nn_action, DEPLOY_ACTION_LOW, DEPLOY_ACTION_HIGH);
    // Serial.printf("[Loop] Motor target:  %.6f (NN + default_dof_pos[%d]=%.4f)\n",
    //               motor_target, onboard_model.get_module_index(),
    //               DEPLOY_DEFAULT_DOF_POS[onboard_model.get_module_index()]);

    // delay(5000);  // Print every 5 seconds
}
