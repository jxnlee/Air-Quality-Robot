/**
 * @file fan.c
 * @brief Fan control driver.
 * 
 * This file contains the implementation of functions to initialize and control
 * the fan which is actuated via a relay module.
 */

#include "head.h"
#include "fan.h"

/// @brief Initializes the fan by setting the relay pin mode and turning it off.
void init_fan()
{
    pinMode(FAN_PIN, OUTPUT);
    turn_off(FAN_PIN);
}

/// @brief Starts the fan by turning the relay on.
void start_fan()
{
    turn_on(FAN_PIN);
}

/// @brief Stops the fan by turning the relay off.
void stop_fan()
{
    turn_off(FAN_PIN);
}