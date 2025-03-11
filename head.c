#include "head.h"

#define PULSE_TIMEOUT	1000000
long long pulseIn(int pin, int level)
{
	long long start_time = micros();

	// wait for for pulse to start
	while (read(pin) != level)
	{
		if (micros() - start_time > PULSE_TIMEOUT) 
		{
			//printf("Current: %lu, Start: %lu\n");
			printf("FAILED PULSE IN!!!\n");
			return -1;
		}
	}

	long long pulse_start = micros();

	// wait for pulse to end
	while (read(pin) == level)
	{
		if (micros() - start_time > PULSE_TIMEOUT) {
			printf("FAILED PULSE IN!!!\n");
			return 0;
		}
	}

	long long pulse_end = micros();

	return pulse_end - pulse_start;
}
