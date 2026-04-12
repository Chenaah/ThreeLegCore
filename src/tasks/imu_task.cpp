#include "tasks.hpp"
#include "deploy_config.h"

namespace {

constexpr float DEGREES_TO_RADIANS = static_cast<float>(M_PI) / 180.0f;

void getImuFrameRotation(float rotation[4], float rotation_conjugate[4]) {
    const float half_theta = 0.5f * DEPLOY_IMU_FRAME_ROTATION_DEG * DEGREES_TO_RADIANS;
    rotation[0] = std::cos(half_theta);
    rotation[1] = 0.0f;
    rotation[2] = 0.0f;
    rotation[3] = std::sin(half_theta);

    rotation_conjugate[0] = rotation[0];
    rotation_conjugate[1] = -rotation[1];
    rotation_conjugate[2] = -rotation[2];
    rotation_conjugate[3] = -rotation[3];
}

}

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

    float r[4];
    float rConjugate[4];
    getImuFrameRotation(r, rConjugate);

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
    float r[4];
    float rConjugate[4];
    getImuFrameRotation(r, rConjugate);

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

        // IMU sampling rate
        static constexpr double IMU_SAMPLE_HZ = 200.0;

        // Butterworth low-pass filters for each axis
        // Quaternion: 20 Hz cutoff — preserves fast body rotations, removes high-freq noise
        ButterworthFilter quat_filter_x(20, IMU_SAMPLE_HZ);
        ButterworthFilter quat_filter_y(20, IMU_SAMPLE_HZ);
        ButterworthFilter quat_filter_z(20, IMU_SAMPLE_HZ);
        ButterworthFilter quat_filter_w(20, IMU_SAMPLE_HZ);

        // Acceleration: 15 Hz cutoff — accelerometer is noisy, aggressive filtering is fine
        ButterworthFilter acc_filter_x(15, IMU_SAMPLE_HZ);
        ButterworthFilter acc_filter_y(15, IMU_SAMPLE_HZ);
        ButterworthFilter acc_filter_z(15, IMU_SAMPLE_HZ);

        // Angular velocity: 25 Hz cutoff — used for control, keep more bandwidth
        ButterworthFilter gyro_filter_x(25, IMU_SAMPLE_HZ);
        ButterworthFilter gyro_filter_y(25, IMU_SAMPLE_HZ);
        ButterworthFilter gyro_filter_z(25, IMU_SAMPLE_HZ);

        void run(void *pvParameters) {

            vTaskDelay(1000);

            // Initialize IMU with 5ms sample interval (200Hz)
            bool success = IMUManager::Initialize(
                BNO08X_CS,
                BNO08X_INT,
                BNO08X_RST,
                5 // 5ms sample interval = 200Hz
            );

            if (!success) {
                Serial.println("No BNO08x detected (IMUManager init failed)");
                enqueue(info_queue, 200);
                vTaskDelete(NULL);
                return;
            }

            Serial.println("BNO08x found! (IMUManager initialized at 200Hz)");
            enqueue(info_queue, 201);

            // Wait for sensor to stabilize
            vTaskDelay(pdMS_TO_TICKS(100));

            uint32_t last_quat_count = 0;
            uint32_t last_accel_count = 0;
            uint32_t last_gyro_count = 0;

            float rate_count[3] = {0, 0, 0};

            while (true) {
                vTaskDelay(pdMS_TO_TICKS(5)); // 5ms = 200Hz, matching IMU sample rate

                static long last_print = millis();
                if (millis() - last_print > 1000) {
                    Serial.print("IMU Rates (Hz) - Quat: ");
                    Serial.print(rate_count[0]);
                    Serial.print(", Acc: ");
                    Serial.print(rate_count[1]);
                    Serial.print(", Gyro: ");
                    Serial.println(rate_count[2]);
                    rate_count[0] = 0;
                    rate_count[1] = 0;
                    rate_count[2] = 0;
                    last_print = millis();
                }

                // Read raw data from IMUManager (no extrapolation needed here,
                // the new driver runs its own SPI task at the configured rate)
                IMUManager::IMUData data = IMUManager::GetRawData();

                // Check for new quaternion data
                {
                    // Use the raw data we already fetched
                    float qi = data.quaternion.x;
                    float qj = data.quaternion.y;
                    float qk = data.quaternion.z;
                    float qw = data.quaternion.w;

                    // Check if data actually changed by comparing timestamp
                    if (data.quaternion_timestamp_us != 0) {
                        std::vector<float> rotated = rotateQuaternion(qi, qj, qk, qw); // input: x,y,z,w

                        // Apply Butterworth filter to each quaternion component
                        quat_imu[0] = quat_filter_x.filter(rotated[0]);
                        quat_imu[1] = quat_filter_y.filter(rotated[1]);
                        quat_imu[2] = quat_filter_z.filter(rotated[2]);
                        quat_imu[3] = quat_filter_w.filter(rotated[3]);

                        // Re-normalize after filtering to keep unit quaternion
                        float norm = std::sqrt(quat_imu[0]*quat_imu[0] + quat_imu[1]*quat_imu[1]
                                             + quat_imu[2]*quat_imu[2] + quat_imu[3]*quat_imu[3]);
                        if (norm > 1e-6f) {
                            quat_imu[0] /= norm;
                            quat_imu[1] /= norm;
                            quat_imu[2] /= norm;
                            quat_imu[3] /= norm;
                        }

                        rate_count[0]++;

                        DEBUG_PRINT("Quat: ");
                        DEBUG_PRINT(quat_imu[0]);
                    }
                }

                // Check for new accelerometer data
                {
                    if (data.acceleration_timestamp_us != 0) {
                        acc_imu[0] = acc_filter_x.filter(data.acceleration.x);
                        acc_imu[1] = acc_filter_y.filter(data.acceleration.y);
                        acc_imu[2] = acc_filter_z.filter(data.acceleration.z);
                        rate_count[1]++;
                    }
                }

                // Check for new gyro data
                {
                    if (data.angular_velocity_timestamp_us != 0) {
                        std::vector<float> rotated_gyro = rotateAngularVelocity(
                            data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z);

                        ang_vel_imu[0] = gyro_filter_x.filter(rotated_gyro[0]);
                        ang_vel_imu[1] = gyro_filter_y.filter(rotated_gyro[1]);
                        ang_vel_imu[2] = gyro_filter_z.filter(rotated_gyro[2]);
                        rate_count[2]++;

                        DEBUG_PRINT("AngVel: ");
                        DEBUG_PRINT(ang_vel_imu[0]);
                        DEBUG_PRINT(", ");
                        DEBUG_PRINT(ang_vel_imu[1]);
                        DEBUG_PRINT(", ");
                        DEBUG_PRINT(ang_vel_imu[2]);
                    }
                }

            }
        }

    }
}
