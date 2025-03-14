/**
 * @file ultrasonic.c
 * @brief Ultrasonic sensor driver functions.
 * 
 * This file contains the implementation of functions to initialize and read data
 * from the ultrasonic sensor used in the Air Quality Robot.
 * 
 * References:
 * 	37 in 1 Ultrasonic module datasheet
 */

#include "ultrasonic.h"
#include "head.h"

/// @brief Initializes the ultrasonic sensor by setting the pin modes.
void init_ultrasonic()
{
	pinMode(ECHO_PIN, INPUT);
	pinMode(TRIGGER_PIN, OUTPUT);
	turn_off(TRIGGER_PIN);
}

/// @brief Reads the distance measured by the ultrasonic sensor.
/// @param distance Pointer to store the measured distance in cm.
void read_ultrasonic(long* distance)
{
	long long duration;
	turn_on(TRIGGER_PIN);
	delayMicroseconds(10);
	turn_off(TRIGGER_PIN);
	duration = pulseIn(ECHO_PIN, HIGH);
	if (duration < 0) *distance = -1;
	else *distance = duration / 2 / 7.6; // distance conversion to cm
}


