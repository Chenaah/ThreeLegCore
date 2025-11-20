#include "tasks.hpp"

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

// std::vector<float> rotateQuaternion(float x, float y, float z, float w) {
//     // rotates a quaternion (x, y, z, w) by 180 degrees around the z-axis
//     // Normalize the quaternion
//     normalize(w, x, y, z);

//     // Rotation quaternion (180 degrees around z-axis)
//     float r[4] = {0, 0, 0, -1}; // cos(90°) + sin(90°) * k

//     // Conjugate of r
//     float rConjugate[4] = {r[0], -r[1], -r[2], -r[3]};

//     // Original quaternion
//     float q[4] = {w, x, y, z};

//     // Temporary quaternion to hold intermediate result
//     float temp[4];
//     multiplyQuaternions(r, q, temp);
//     multiplyQuaternions(temp, rConjugate, q);

//     std::vector<float> q_vec = {q[1], q[2], q[3], q[0]}; //xyzw

//     return q_vec;

// }

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

// // Function to rotate an angular velocity vector
// std::vector<float> rotateAngularVelocity(float wx, float wy, float wz) {

//     // Rotation quaternion
//     float r[4] = {0, 0, 0, -1}; // cos(90°) + sin(90°) * k

//     // Conjugate of r
//     float rConjugate[4] = {r[0], -r[1], -r[2], -r[3]};

//     // Angular velocity quaternion
//     float omega[4] = {0, wx, wy, wz};

//     // Temporary quaternion to hold intermediate result
//     float temp[4];
//     multiplyQuaternions(r, omega, temp);
//     multiplyQuaternions(temp, rConjugate, omega);

//     std::vector<float> ang_vel_vec = {omega[1], omega[2], omega[3]}; //xyzw

//     return ang_vel_vec;
// }


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

        BNO08x myIMU;

        void set_reports() {
            // Here is where you define the sensor outputs you want to receive
            // TODO: Set data output rate
            Serial.println("Setting desired reports");
            if (myIMU.enableRotationVector() == true) {
                Serial.println(F("Rotation vector enabled"));
                Serial.println(F("Output in form i, j, k, real, accuracy"));
            } else {
                Serial.println("Could not enable rotation vector");
            }
            delay(100);
            if (myIMU.enableAccelerometer() == true) {
                Serial.println(F("Accelerometer enabled"));
                Serial.println(F("Output in form x, y, z, in m/s^2"));
            } else {
                Serial.println("Could not enable accelerometer");
            }
            delay(100);
            if (myIMU.enableGyro() == true) {
                Serial.println(F("Gyro enabled"));
                Serial.println(F("Output in form x, y, z, in radians per second"));
            } else {
                Serial.println("Could not enable gyro");
            }
            delay(100); // This delay allows enough time for the BNO086 to accept the new 
                        // configuration and clear its reset status
        }

        void run(void *pvParameters) {

            vTaskDelay(1000);

            //if (myIMU.begin() == false) {  
            if (myIMU.beginSPI(BNO08X_CS, BNO08X_INT, BNO08X_RST) == false) {
                Serial.print("No BNO08x detected");
                enqueue(info_queue, 200);
            } else {
                Serial.println("BNO08x found!");
                enqueue(info_queue, 201);
            }

            // Configeration
            set_reports();

            float count[3] = {0, 0, 0};

            while (true) {
                // esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(DELAY_PERIOD));

                if (myIMU.wasReset()) {
                    Serial.print("sensor was reset ");
                    set_reports();
                }

                static long last_print=millis();
                if (millis()-last_print>1000){
                    Serial.print("IMU Rates (Hz) - Quat: ");
                    Serial.print(count[0]);
                    Serial.print(", Acc: ");
                    Serial.print(count[1]);
                    Serial.print(", Gyro: ");
                    Serial.println(count[2]);
                    count[0]=0;
                    count[1]=0;
                    count[2]=0;
                    last_print=millis();
                }

                // Has a new event come in on the Sensor Hub Bus?
                if (myIMU.getSensorEvent() == true) {
                    // Serial.print("Event ID: ");
                    // Serial.println(myIMU.getSensorEventID());

                    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
                        float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
                        quat_imu[0] = myIMU.getQuatI(); // x
                        quat_imu[1] = myIMU.getQuatJ(); // y
                        quat_imu[2] = myIMU.getQuatK(); // z
                        quat_imu[3] = myIMU.getQuatReal(); // w

                        quat_imu = rotateQuaternion(quat_imu[0], quat_imu[1], quat_imu[2], quat_imu[3]); // input: x,y,z,w

                        count[0] ++;

                        DEBUG_PRINT("Quat: ");
                        DEBUG_PRINT(quat_imu[0]);
                    }

                    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
                        acc_imu[0] = myIMU.getAccelX();
                        acc_imu[1] = myIMU.getAccelY();
                        acc_imu[2] = myIMU.getAccelZ();
                        count[1] ++;
                        // Serial.print("Acc: ");
                        // Serial.println(acc_imu[0]);
                    }

                    if (myIMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
                        ang_vel_imu[0] = myIMU.getGyroX();
                        ang_vel_imu[1] = myIMU.getGyroY();
                        ang_vel_imu[2] = myIMU.getGyroZ();
                        ang_vel_imu = rotateAngularVelocity(ang_vel_imu[0], ang_vel_imu[1], ang_vel_imu[2]);
                        count[2] ++;
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