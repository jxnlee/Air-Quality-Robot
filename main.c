#include <stdio.h>
#include <pthread.h>
#include <wiringPi.h>
#include "head.h"
#include "dht.h"
#include "ultrasonic.h"
#include "pms.h"
#include "l298n.h"

int main(int argc, char* argv[])
{
	// Setup
	if (wiringPiSetup() == -1)
	{
		printf("Failed to setup wiringPi.\n");
		return 1;
	}
	init_ultrasonic();
	//if (!init_pms()) return 0;
	float temperature;
	float humidity;
	long distance;
	//PM25_Data pms5003_data;
	init_l298n();
	//set_motors_speed(255);

	int running = 1;

	// Main program loop
	while (running)
	{
		/*if (read_dht(&temperature, &humidity))
		{
			printf("temperature: %.1f deg C, humidity: %.1f\%\n", temperature, humidity);	
			delay(1000);
		}
		else delay(1000);*/
		//read_ultrasonic(&distance);
		//printf("Ultrasonic Distance Measurement: %ldcm\n", distance);
		/*
		if (read_pms(&pms5003_data))
		{
			print_pms_readings(&pms5003_data);
		}
		else
		{
			printf("ERROR READING PM2.5!!!\n");
		}
		delay(1000);*/
		
		drive_right_forward(160);
		drive_left_backward(40);
		delay(500);
		motors_off();
		delay(5000);

	}
	close_pms();
	motors_off();
	return 0;
}
