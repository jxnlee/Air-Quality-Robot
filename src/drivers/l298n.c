/**
 * @file l298n.c
 * @brief L298N motor driver control functions.
 * 
 * This file contains the implementation of functions to initialize and control
 * the L298N motor driver used in the Air Quality Robot.
 */
#include "head.h"
#include "l298n.h"

/// @brief Initializes the L298N motor driver by setting the pin modes and creating PWM channels.
void init_l298n()
{
	pinMode(L_MOTORS1_PIN, OUTPUT);
	pinMode(L_MOTORS2_PIN, OUTPUT);
	pinMode(L_MOTORS_SPD_PIN, OUTPUT);
	softPwmCreate(L_MOTORS_SPD_PIN, 0, 0xff);

	pinMode(R_MOTORS1_PIN, OUTPUT);
	pinMode(R_MOTORS2_PIN, OUTPUT);
	pinMode(R_MOTORS_SPD_PIN, OUTPUT);
	softPwmCreate(R_MOTORS_SPD_PIN, 0, 0xff);
}

/// @brief Sets the speed of both motors.
/// @param speed The speed value to set (0-255).
void set_motors_speed(uint8_t speed)
{
	softPwmWrite(L_MOTORS_SPD_PIN, speed);
	softPwmWrite(R_MOTORS_SPD_PIN, speed);
}

/// @brief Turns off both motors.
void motors_off()
{
	turn_off(L_MOTORS1_PIN);
	turn_off(L_MOTORS2_PIN);
	turn_off(R_MOTORS1_PIN);
	turn_off(R_MOTORS2_PIN);
}

/// @brief Drives the left motor forward at the specified speed.
/// @param speed The speed value to set (0-255).
void drive_left_forward(uint8_t speed)
{
	softPwmWrite(L_MOTORS_SPD_PIN, speed);
	turn_on(L_MOTORS1_PIN);
	turn_off(L_MOTORS2_PIN);
}

/// @brief Drives the left motor backward at the specified speed.
/// @param speed The speed value to set (0-255).
void drive_left_backward(uint8_t speed)
{
	softPwmWrite(L_MOTORS_SPD_PIN, speed);
	turn_on(L_MOTORS2_PIN);
	turn_off(L_MOTORS1_PIN);
}

/// @brief Drives the right motor forward at the specified speed.
/// @param speed The speed value to set (0-255).
void drive_right_forward(uint8_t speed)
{
	softPwmWrite(R_MOTORS_SPD_PIN, speed);
	turn_on(R_MOTORS1_PIN);
	turn_off(R_MOTORS2_PIN);
}

/// @brief Drives the right motor backward at the specified speed.
/// @param speed The speed value to set (0-255).
void drive_right_backward(uint8_t speed)
{
	softPwmWrite(R_MOTORS_SPD_PIN, speed);
	turn_on(R_MOTORS2_PIN);
	turn_off(R_MOTORS1_PIN);
}

/// @brief Drives both motors forward at the specified speed.
/// @param speed The speed value to set (0-255).
void drive_forward(uint8_t speed)
{
	drive_left_forward(speed);
	drive_right_forward(speed);
}

/// @brief Drives both motors backward at the specified speed.
/// @param speed The speed value to set (0-255).
void drive_backward(uint8_t speed)
{
	drive_left_backward(speed);
	drive_right_backward(speed);
}