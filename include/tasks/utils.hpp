#pragma once

#include <Arduino.h>
#include <vector>
#include <queue>

#define MAX_QUEUE_SIZE 5 // Define maximum queue size
#define LED_RED_PIN GPIO_NUM_18
#define LED_GREEN_PIN GPIO_NUM_17
#define LED_BLUE_PIN GPIO_NUM_16


#define LED_MSG_POWER_ON 0
#define LED_MSG_WIFI 1
#define LED_MSG_NORMAL 2
#define LED_MSG_MOTOR_ERROR 3
#define LED_MSG_CALI 4
#define LED_MSG_MOTOR_ON 5
#define LED_MSG_MOTOR_OFF 6
#define LED_MSG_WAIT_CALI 7
#define LED_MSG_UNSAFE 8
#define LED_MSG_WAIT_HELP 9

void enqueue(std::queue<int> &queue, int value);

int dequeue(std::queue<int> &queue);

void set_led_color(int red, int green, int blue);

void send_led_message(int msg);