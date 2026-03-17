#include <vector>
#include <queue>
#include <Arduino.h>
#include <tasks.hpp>



void enqueue(std::queue<int> &queue, int value){
    if (queue.size() >= MAX_QUEUE_SIZE) {
        queue.pop();
    }
    queue.push(value);
}

int dequeue(std::queue<int> &queue){
    int value = 0;
    if (not queue.empty()){
        value = queue.front();
        queue.pop();
    }
    return value;
}


void set_led_color(int red, int green, int blue) {
    analogWrite(LED_RED_PIN, 255-red);
    analogWrite(LED_GREEN_PIN, 255-green);
    analogWrite(LED_BLUE_PIN, 255-blue);
}

bool motor_error_flag = false;
unsigned long lastTime = 0;

void send_led_message(int msg) {
    unsigned long currentTime = millis();

    if (msg == 0){
        // Yellow: power on
        set_led_color(240, 255, 0);
    } else if (msg == LED_MSG_WIFI){
        // Green: WiFi connection
        set_led_color(40, 240, 0);
    } else if (msg == LED_MSG_NORMAL){
        // Green: Waiting for calibration (normal)
        set_led_color(40, 240, 0);
    } else if (msg == LED_MSG_MOTOR_ERROR){
        // Red: Motor error
        if (motor_error_flag == false){
            set_led_color(240, 20, 20);
        } else {
            set_led_color(0, 0, 0);
        }
        if (currentTime - lastTime >= 200) {
            motor_error_flag = !motor_error_flag;
            lastTime = currentTime;
        }
    } else if (msg == LED_MSG_CALI){ // LED_MSG_CALI
        // Orange: Calibration
        set_led_color(253, 133, 13);
    } else if (msg == LED_MSG_MOTOR_ON){ // LED_MSG_ON
        // White: LED on
        set_led_color(240, 240, 255);
    } else if (msg == LED_MSG_MOTOR_OFF){ // LED_MSG_OFF
        // Off: blue
        set_led_color(0, 0, 255);
    } else if (msg == LED_MSG_WAIT_CALI){
        if (motor_error_flag == false){
            set_led_color(255, 255, 0);
        } else {
            set_led_color(0, 0, 0);
        }
        if (currentTime - lastTime >= 200) {
            motor_error_flag = !motor_error_flag;
            lastTime = currentTime;
        }
    } else if (msg == LED_MSG_UNSAFE){
        // Red: Unsafe
        if (motor_error_flag == false){
            set_led_color(255, 0, 0);
        } else {
            set_led_color(0, 0, 0);
        }
        if (currentTime - lastTime >= 200) {
            motor_error_flag = !motor_error_flag;
            lastTime = currentTime;
        }
    } else if (msg == LED_MSG_WAIT_HELP){
        // Blue: Waiting for help
        if (motor_error_flag == false){
            set_led_color(255, 255, 255);
        } else {
            set_led_color(255, 0, 0);
        }
        if (currentTime - lastTime >= 200) {
            motor_error_flag = !motor_error_flag;
            lastTime = currentTime;
        }
    } else if (msg == LED_MSG_POLICY_ERROR){
        // Purple blink: policy hash/sanity validation failed
        if (motor_error_flag == false){
            set_led_color(255, 0, 255);
        } else {
            set_led_color(0, 0, 0);
        }
        if (currentTime - lastTime >= 200) {
            motor_error_flag = !motor_error_flag;
            lastTime = currentTime;
        }
    } else if (msg == LED_MSG_POLICY_ACTIVE){
        // Cyan: onboard policy is validated and actively running
        set_led_color(0, 220, 255);
    }
}
