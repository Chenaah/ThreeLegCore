#include "tasks.hpp"
#include "identity.hpp"
#include "LocalPolicy.hpp"

// Global policy instance (accessible from other tasks if needed)
LocalPolicy local_policy;

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

    // --- Load local policy from LittleFS ---
    Serial.println("\n====== Loading Local Policy ======");

    // TODO: Set module index based on hardware identity.
    // Each ESP32 controls one module (leg). The module index determines
    // which default_dof_pos offset is applied after the NN output.
    //   Module 0: default_dof_pos = 0.0 rad
    //   Module 1: default_dof_pos = 0.5 rad
    //   Module 2: default_dof_pos = -0.5 rad
    local_policy.set_module_index(0);  // <-- Change per module

    if (local_policy.load_from_littlefs()) {
        // Run sanity check with reference values from Python conversion script:
        //   Input: latent[0]=1.0, local_obs[0]=0.5, rest zeros
        //   Expected raw mean: -1.80628085
        //   Expected NN action (before default_dof_pos): -0.75796205
        local_policy.run_sanity_check(-1.80628085f, -0.75796205f);
        Task::local_policy_active = true;   // Enable local-policy mode in motor task
        Serial.println("[Setup] Local policy active: motor task will use NN at 100 Hz.");
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
