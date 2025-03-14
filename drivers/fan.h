/**
 * @file fan.h
 * @brief Header file for fan control driver.
 * 
 * This file contains the implementation of functions to initialize and control
 * the fan which is actuated via a relay module.
 */

 #ifndef _FAN_H_
 #define _FAN_H_
 
/// @brief Initializes the fan by setting the relay pin mode and turning it off.
 void init_fan();
 
/// @brief Starts the fan by turning the relay on.
 void start_fan();
 
/// @brief Stops the fan by turning the relay off.
 void stop_fan();
 
 #endif