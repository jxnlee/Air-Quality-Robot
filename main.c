#include <stdio.h>
#include <pthread.h>
#include <wiringPi.h>
#include "head.h"
#include "dht.h"
#include "ultrasonic.h"
#include "pms.h"
#include "l298n.h"
#include "fan.h"
#include "mpu6050.h"
#include "nion_gen.h"

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
	init_fan();
	init_nion_gen();
	//set_motors_speed(255);

	int running = 1;

	//float Ax, Ay, Az, Gx, Gy, Gz;

	// Main program loop
	int counter = 0;
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
		//read_accelerometer(&Ax, &Ay, &Az);
		//read_gyroscope(&Gx, &Gy, &Gz);
		//printf("\n Gx=%.3f °/s\tGy=%.3f °/s\tGz=%.3f °/s\tAx=%.3f g\tAy=%.3f g\tAz=%.3f g\n",Gx,Gy,Gz,Ax,Ay,Az);
		//delay(500);
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
		
		//drive_right_forward(160);
		//drive_left_backward(40);
		//delay(500);
		//motors_off();
		//delay(5000);
		start_nion_gen();
		delay(1000);
		stop_nion_gen();
		delay(5000);
		//counter++;
	}
	//close_pms();
	//motors_off();
	return 0;
}
