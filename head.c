#include "head.h"

#define PULSE_TIMEOUT	1000000
unsigned long pulseIn(int pin, int level)
{
	unsigned long start_time = micros();

	// wait for for pulse to start
	while (read(pin) != level)
	{
		if (micros() - start_time > PULSE_TIMEOUT) 
		{
			printf("FAILED PULSE IN!!!\n");
			return 0;
		}
	}

	unsigned long pulse_start = micros();

	// wait for pulse to end
	while (read(pin) == level)
	{
		if (micros() - start_time > PULSE_TIMEOUT) {
			printf("FAILED PULSE IN!!!\n");
			return 0;
		}
	}

	unsigned long pulse_end = micros();

	unsigned long pulse_dur = pulse_end - pulse_start;

	//printf("PULSE DURATION: %lu\n", pulse_dur);

	return pulse_dur;
}
