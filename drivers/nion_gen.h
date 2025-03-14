/**
 * @file nion_gen.h
 * @brief Header file for negative ion generator control functions.
 * 
 * This file contains the function declarations for initializing and controlling
 * the negative ion generator which is actuated via a relay module.
 */

#ifndef _NION_GEN_
#define _NION_GEN_
/// @brief Initializes the negative ion generator by setting the relay pin mode and turning it off.
void init_nion_gen();

/// @brief Starts the negative ion generator by turning the relay on.
void start_nion_gen();

/// @brief Stops the negative ion generator by turning the relay off.
void stop_nion_gen();

#endif