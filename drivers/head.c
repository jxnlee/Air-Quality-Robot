/**
 * @file head.c
 * @brief Implementation file for common hardware control functions.
 * 
 * This file contains the implementation of helper functions used for
 * controlling and interfacing with various hardware components of the
 * Air Quality Robot.
 * 
 * References:
 * 	For PulseIn Function: referenced this pull to WiringPi https://github.com/WiringPi/WiringPi/pull/319/files
 */
#include "head.h"

// set timeout to 1 sec (or 1000000 microseconds)
#define PULSE_TIMEOUT	1000000

/// @brief Measures the duration of a pulse on the specified pin.
/// @param pin The pin number to read the pulse from.
/// @param level The level to measure (HIGH or LOW).
/// @return The duration of the pulse in microseconds.
long long pulseIn(int pin, int level)
{
	long long start_time = micros();

	// wait for for pulse to start
	while (read(pin) != level)
	{
		if (micros() - start_time > PULSE_TIMEOUT) 
		{
			printf("FAILED TO START PULSE!!!\n");
			return 0;
		}
	}

	long long pulse_start = micros();

	// wait for pulse to end
	while (read(pin) == level)
	{
		if (micros() - start_time > PULSE_TIMEOUT) {
			printf("FIALED TO END PULSE!!!\n");
			return 0;
		}
	}

	long long pulse_end = micros();

	// return pulse duration
	return pulse_end - pulse_start;
}
