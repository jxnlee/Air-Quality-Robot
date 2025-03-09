#include "ultrasonic.h"
#include "head.h"



	void init_ultrasonic()
{
	pinMode(ECHO_PIN, INPUT);
	pinMode(TRIGGER_PIN, OUTPUT);
	turn_off(TRIGGER_PIN);
}

void read_ultrasonic(unsigned long* distance)
{
	unsigned long duration;
	turn_on(TRIGGER_PIN);
	delayMicroseconds(10);
	turn_off(TRIGGER_PIN);
	duration = pulseIn(ECHO_PIN, HIGH);
	*distance = duration / 2 / 7.6;
}


