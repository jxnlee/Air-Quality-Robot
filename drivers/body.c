#include "head.h"
#include "ultrasonic.h"
#include "l298n.h"
#include "pms.h"
#include "fan.h"
#include "nion_gen.h"

int setup()
{
	if (wiringPiSetup() == -1)
	{
		printf("Failed to setup wiringPi.\n");
		return 0;
	}
	init_ultrasonic();
	init_l298n();
	//if (!init_pms()) return 0;
	init_fan();
	init_nion_gen();
	return 1;
}
