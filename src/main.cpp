#include "tasks.hpp"
#include "identity.hpp"
#include "LocalPolicy.hpp"
#include <driver/spi_master.h>

// Global policy instance (accessible from other tasks if needed)
LocalPolicy local_policy;

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

    // --- Load local policy from LittleFS ---
    Serial.println("\n====== Loading Local Policy ======");

    // Fallback only for sanity checks and offline debugging.
    // During live hierarchical control, the PC sends the joint offset explicitly
    // in every MotorCommand, so the ESP32 does not depend on hardcoded mapping.
    local_policy.set_module_index(0);

    if (local_policy.load_from_littlefs()) {
        // Run sanity check with reference values generated alongside the deploy headers.
        local_policy.run_sanity_check(
            DEPLOY_SANITY_EXPECTED_MEAN,
            DEPLOY_SANITY_EXPECTED_ACTION
        );
        Task::local_policy_loaded = true;
        Serial.println("[Setup] Local policy loaded: waiting for PC control_mode to activate NN.");
    } else {
        Serial.println("WARNING: Local policy not loaded, running without NN");
    }
    Serial.println("==================================\n");

    TaskHandle_t xHandle;

    xTaskCreatePinnedToCore(Task::IMUTask::run, "IMU_Task", 10000, nullptr, 1, &xHandle, 1);
    xTaskCreatePinnedToCore(Task::MotorTask::run, "Motor_Task", 10000, nullptr, 3, &xHandle, 1);   // Highest priority for stable 100 Hz
    xTaskCreatePinnedToCore(Task::CommTask::run, "Comm_Task", 10000, nullptr, 2, &xHandle, 0);     // Medium priority for timely command delivery
    xTaskCreatePinnedToCore(Task::MonitorTask::run, "Monitor_Task", 10000, nullptr, 0, &xHandle, 0);

    vTaskDelete(NULL);
}

void loop() {
    vTaskDelete(NULL);
    // if (!local_policy.is_loaded()) {
    //     delay(5000);
    //     return;
    // }

    // // Use the latest received latent from comm_task if available,
    // // otherwise fall back to hardcoded demo values.
    // std::array<float, LOCAL_LATENT_DIM> latent;
    // bool latent_from_network = false;

    // // Task::received_joint_id >= 0 means a real command with joint_id has arrived
    // if (Task::received_joint_id >= 0) {
    //     for (size_t i = 0; i < LOCAL_LATENT_DIM; i++) {
    //         latent[i] = Task::received_latent[i];
    //     }
    //     latent_from_network = true;
    // } else {
    //     // Demo: hardcoded latent when no network command has been received
    //     latent = {0.5f, -0.3f, 0.1f, 0.0f, 0.2f, -0.1f, 0.4f, -0.2f};
    // }

    // std::array<float, LOCAL_OBS_DIM> obs = {};  // zeros = placeholder sensor data

    // // forward_nn: raw NN output in [-0.8, 0.8], no offset
    // float nn_action = local_policy.forward_nn(latent, obs);

    // // select_action: NN output + default_dof_pos[module_idx] = motor target
    // float motor_target = local_policy.select_action(latent, obs);

    // if (latent_from_network) {
    //     Serial.printf("[Loop] Using network latent (joint_id=%d): [%.3f, %.3f, %.3f, ...]\n",
    //                   Task::received_joint_id, latent[0], latent[1], latent[2]);
    // } else {
    //     Serial.println("[Loop] Running policy with hardcoded demo latent...");
    // }
    // Serial.printf("[Loop] NN action:     %.6f (in [-0.8, 0.8])\n", nn_action);
    // Serial.printf("[Loop] Motor target:  %.6f (NN + default_dof_pos[%d]=%.4f)\n",
    //               motor_target, local_policy.get_module_index(),
    //               DEPLOY_DEFAULT_DOF_POS[local_policy.get_module_index()]);

    // delay(5000);  // Print every 5 seconds
}
