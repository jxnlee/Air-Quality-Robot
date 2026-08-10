/**
 * @file l298n.h
 * @brief Header file for L298N motor driver control functions.
 * 
 * This file contains the function declarations for initializing and controlling
 * the L298N motor driver used in the Air Quality Robot.
 */
#ifndef _L298N_
#define	_L298N_
// library for pulse width modulation
#include <softPwm.h>

/// @brief Initializes the L298N motor driver by setting the pin modes and creating PWM channels.
void init_l298n();

/// @brief Sets the speed of both motors.
/// @param speed The speed value to set (0-255).
void set_motors_speed(uint8_t speed);

/// @brief Turns off both motors.
void motors_off();

/// @brief Drives the left motor forward at the specified speed.
/// @param speed The speed value to set (0-255).
void drive_left_forward(uint8_t speed);

/// @brief Drives the left motor backward at the specified speed.
/// @param speed The speed value to set (0-255).
void drive_left_backward(uint8_t speed);

/// @brief Drives the right motor forward at the specified speed.
/// @param speed The speed value to set (0-255).
void drive_right_forward(uint8_t speed);

/// @brief Drives the right motor backward at the specified speed.
/// @param speed The speed value to set (0-255).
void drive_right_backward(uint8_t speed);

/// @brief Drives both motors forward at the specified speed.
/// @param speed The speed value to set (0-255).
void drive_forward(uint8_t speed);

/// @brief Drives both motors backward at the specified speed.
/// @param speed The speed value to set (0-255).
void drive_backward(uint8_t speed);
#endif
