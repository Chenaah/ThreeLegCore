#include "tasks.hpp"
#include <IMUManager.hpp>

// Helper functions for quaternion rotation (kept for backward compatibility)
void normalize(float& w, float& x, float& y, float& z) {
    float norm = std::sqrt(w*w + x*x + y*y + z*z);
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
}

void multiplyQuaternions(const float q1[4], const float q2[4], float result[4]) {
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

std::vector<float> rotateQuaternion(float x, float y, float z, float w) {
    // Normalize the quaternion
    normalize(w, x, y, z);

    // Rotation quaternion (60 degrees around z-axis)
    float theta = -60.0 * M_PI / 180.0; // Convert degrees to radians
    float r[4] = {std::cos(theta / 2), 0, 0, std::sin(theta / 2)}; // (w, x, y, z)

    // Conjugate of r
    float rConjugate[4] = {r[0], -r[1], -r[2], -r[3]};

    // Original quaternion
    float q[4] = {w, x, y, z};

    // Temporary quaternion to hold intermediate result
    float temp[4];
    multiplyQuaternions(r, q, temp);
    multiplyQuaternions(temp, rConjugate, q);

    std::vector<float> q_vec = {q[1], q[2], q[3], q[0]}; // xyzw

    return q_vec;
}

// Function to rotate an angular velocity vector
std::vector<float> rotateAngularVelocity(float wx, float wy, float wz) {
    // Rotation quaternion (60 degrees around z-axis)
    float theta = -60.0 * M_PI / 180.0; // Convert degrees to radians
    float r[4] = {std::cos(theta / 2), 0, 0, std::sin(theta / 2)}; // (w, x, y, z)

    // Conjugate of r
    float rConjugate[4] = {r[0], -r[1], -r[2], -r[3]};

    // Angular velocity quaternion
    float omega[4] = {0, wx, wy, wz}; // (w, x, y, z)

    // Temporary quaternion to hold intermediate result
    float temp[4];
    multiplyQuaternions(r, omega, temp);
    multiplyQuaternions(temp, rConjugate, omega);

    // The result will be a pure quaternion representing the rotated angular velocity vector
    std::vector<float> ang_vel_vec = {omega[1], omega[2], omega[3]}; // x, y, z

    return ang_vel_vec;
}


namespace Task {

    std::vector<float> euler_imu(3), quat_imu(4), acc_imu(3), ang_vel_imu(3);

    namespace IMUTask {

        static bool imu_initialized = false;

        bool initialize(uint8_t cs_pin, uint8_t int_pin, uint8_t rst_pin) {
            Serial.println("[IMU] Initializing via IMUManager...");
            
            // Initialize IMU using NewRollbot's IMUManager
            // Parameters: cs_pin, int_pin, rst_pin, sample_interval_ms, task_priority
            if (!IMUManager::Initialize(cs_pin, int_pin, rst_pin, 10, 6)) {
                Serial.println("[IMU] ERROR: Failed to initialize IMU!");
                enqueue(info_queue, 200);
                return false;
            }
            
            Serial.println("[IMU] IMU initialized successfully!");
            enqueue(info_queue, 201);
            imu_initialized = true;
            return true;
        }

        void run(void *pvParameters) {
            // Wait for system to stabilize
            vTaskDelay(pdMS_TO_TICKS(500));

            if (!imu_initialized) {
                Serial.println("[IMU] ERROR: IMU not initialized! Task exiting.");
                vTaskDelete(NULL);
                return;
            }

            Serial.println("[IMU] Task started, reading data from IMUManager...");

            float count = 0;
            uint32_t last_stats_time = millis();

            while (true) {
                vTaskDelay(pdMS_TO_TICKS(DELAY_PERIOD));

                // Get IMU data from IMUManager
                IMUManager::IMUData imu_data = IMUManager::GetData();

                // Check if we have valid data (timestamp > 0)
                if (imu_data.last_data_time_us > 0) {
                    // Update quaternion (apply rotation for backward compatibility)
                    // IMUManager returns w, x, y, z
                    quat_imu = rotateQuaternion(
                        imu_data.quaternion.x,
                        imu_data.quaternion.y,
                        imu_data.quaternion.z,
                        imu_data.quaternion.w
                    ); // output: x, y, z, w
                    // Serial.printf("[IMU] Quat (xyzw): %.4f, %.4f, %.4f, %.4f\n", quat_imu[0], quat_imu[1], quat_imu[2], quat_imu[3]);

                    // Update acceleration
                    acc_imu[0] = imu_data.acceleration.x;
                    acc_imu[1] = imu_data.acceleration.y;
                    acc_imu[2] = imu_data.acceleration.z;

                    // Update angular velocity (apply rotation for backward compatibility)
                    ang_vel_imu = rotateAngularVelocity(
                        imu_data.angular_velocity.x,
                        imu_data.angular_velocity.y,
                        imu_data.angular_velocity.z
                    );

                    count++;
                }

                // Print statistics every second
                uint32_t now = millis();
                if (now - last_stats_time >= 1000) {
                    Serial.printf("[IMU] Rate: %.0f Hz\n", count);
                    count = 0;
                    last_stats_time = now;

                    // Debug print
                    DEBUG_PRINT("Quat: ");
                    DEBUG_PRINT(quat_imu[0]);
                }
            }
        }

    }
}