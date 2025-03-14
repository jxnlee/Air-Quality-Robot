/**
 * @file nion_gen.c
 * @brief Negative ion generator control functions.
 * 
 * This file contains the implementation of functions to initialize and control
 * the negative ion generator which is actuated via a relay module.
 */
#include "head.h"
#include "nion_gen.h"

/// @brief Initializes the negative ion generator by setting the relay pin mode and turning it off.
void init_nion_gen()
{
    pinMode(NION_GEN_PIN, OUTPUT);
    turn_off(NION_GEN_PIN);
}

/// @brief Starts the negative ion generator by turning the relay on.
void start_nion_gen()
{
    turn_on(NION_GEN_PIN);
}

/// @brief Stops the negative ion generator by turning the relay off.
void stop_nion_gen()
{
    turn_off(NION_GEN_PIN);
}