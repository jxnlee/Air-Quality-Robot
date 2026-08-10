/**
 * @file body.c
 * @brief Main setup file for initializing hardware components.
 * 
 * This file contains the setup function which initializes various hardware
 * components required for the Air Quality Robot.
 */
#include "head.h"
#include "ultrasonic.h"
#include "l298n.h"
#include "pms.h"
#include "fan.h"
#include "nion_gen.h"

/// @brief Sets up wiringPi and initializes hardware component drivers.
/// @return 1 if the setup is successful, 0 otherwise.
int setup()
{
	if (wiringPiSetup() == -1)
	{
		printf("Failed to setup wiringPi.\n");
		return 0;
	}
	init_ultrasonic();
	init_l298n();
	if (!init_pms()) return 0;
	init_fan();
	init_nion_gen();
	return 1;
}
