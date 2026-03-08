/**
 * @file Blink.hpp
 * @brief Simple LED blinking function using hardware definitions
 */

#ifndef BLINK_HPP
#define BLINK_HPP

#include "Arduino.h"
#include <HardwareDefs.hpp>

/**
 * @brief Blinks the RGB LED with specified color for a specified number of times
 * @param timePerBlink Time in milliseconds for each blink cycle (on + off)
 * @param timesOfBlink Number of times to blink the LED
 * @param red True to enable red LED
 * @param green True to enable green LED
 * @param blue True to enable blue LED
 */
void Blink(uint32_t timePerBlink, uint16_t timesOfBlink, bool red = true, bool green = true, bool blue = true)
{
    // Initialize LED pins as output
    pinMode(LED_R_PIN, OUTPUT);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_B_PIN, OUTPUT);

    // Calculate on/off time (half of total blink time each)
    uint32_t halfTime = timePerBlink / 2;

    // Blink the LEDs for the specified number of times
    for (uint16_t i = 0; i < timesOfBlink; i++)
    {
        // Turn selected LEDs on
        digitalWrite(LED_R_PIN, red ? LOW : HIGH);
        digitalWrite(LED_G_PIN, green ? LOW : HIGH);
        digitalWrite(LED_B_PIN, blue ? LOW : HIGH);
        delay(halfTime);

        // Turn all LEDs off
        digitalWrite(LED_R_PIN, HIGH);
        digitalWrite(LED_G_PIN, HIGH);
        digitalWrite(LED_B_PIN, HIGH);
        delay(halfTime);
    }
}

#endif