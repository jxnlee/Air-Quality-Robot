/**
 * @file ultrasonic.h
 * @brief Header file for ultrasonic sensor driver functions.
 * 
 * This file contains the function declarations for initializing and reading data
 * from the ultrasonic sensor used in the Air Quality Robot.
 */

#ifndef _ULTRASONIC_
#define _ULTRASONIC_
/// @brief Initializes the ultrasonic sensor by setting the pin modes.
void init_ultrasonic();

/// @brief Reads the distance measured by the ultrasonic sensor.
/// @param distance Pointer to store the measured distance.
void read_ultrasonic(long* distance);

#endif
