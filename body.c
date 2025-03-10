#include "head.h"
#include "dht.h"
#include "ultrasonic.h"
#include "pms.h"
#include "l298n.h"

int setup()
{
	if (wiringPiSetup() == -1)
	{
		printf("Failed to setup wriingPi.\n");
		return 0;
	}
	init_ultrasonic();
	init_l298n();
	set_motors_speed(255);
	return 1;
}
