/**
 * @file head.h
 * @brief Header file containing common definitions and macros for hardware control.
 * 
 * This file includes common definitions, macros, and pin configurations used
 * across various hardware components of the Air Quality Robot.
 */
#ifndef _HEAD_
#define _HEAD_

#include <stdio.h>
#include <wiringPi.h>
#include <stdint.h>
#include <stdlib.h>

// Macros for controlling pins
#define turn_on(pin)	digitalWrite(pin, 1)
#define turn_off(pin)	digitalWrite(pin, 0)
#define read(pin)	digitalRead(pin)
#define write(pin, o)	digitalWrite(pin, output)

#define LOW	0
#define HIGH	1

// Pinout definitions
#define DHT_PIN			7
#define TRIGGER_PIN		0
#define ECHO_PIN		2
#define FAN_PIN         1
#define NION_GEN_PIN    5
#define L_MOTORS_SPD_PIN	23
#define L_MOTORS1_PIN		25
#define L_MOTORS2_PIN		24
#define R_MOTORS1_PIN		28
#define R_MOTORS2_PIN		27
#define R_MOTORS_SPD_PIN	29

// Helper Functions / Variables / Structs

/// @brief Measures the duration of a pulse on the specified pin.
/// @param pin The pin number to read the pulse from.
/// @param level The level to measure (HIGH or LOW).
/// @return The duration of the pulse in microseconds.
long long pulseIn(int pin, int level);

#endif
