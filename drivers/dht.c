/**
 * @file dht.c
 * @brief DHT11 sensor driver for reading temperature and humidity.
 * 
 * This file contains the implementation of the functions required to
 * interface with the DHT11 sensor and read temperature and humidity data.
 * 
 * References: 
 * 	37 in 1 DHT11 Sensor Module Datasheet
 * 	https://www.circuitbasics.com/how-to-set-up-the-dht11-humidity-sensor-on-the-raspberry-pi/
 */
#include <stdio.h>
#include <stdint.h>
#include "dht.h"
#include "head.h"

// number of times to read
#define MAXTIMINGS	100

// data buffer
int data[5] = { 0, 0, 0, 0, 0 };

__attribute__((visibility("default")))

/// @brief Reads temperature and humidity from the DHT sensor.
/// @param temperature Pointer to store the read temperature.
/// @param humidity Pointer to store the read humidity.
/// @return 1 if the data is read successfully, 0 otherwise.
int read_dht(float* temperature, float* humidity)
{
	uint8_t laststate	= HIGH;
	uint8_t counter		= 0;
	uint8_t j		= 0, i;
 
	// Initialize data buffer
	data[0] = data[1] = data[2] = data[3] = data[4] = 0;
 
	// Send start signal to DHT sensor (specifications according to its datasheet)
	pinMode( DHT_PIN, OUTPUT );
	turn_off(DHT_PIN);
	delay( 18 );
	turn_on(DHT_PIN);
	delayMicroseconds( 40 );
	pinMode( DHT_PIN, INPUT );
 
	// Read data from DHT sensor
	for (i = 0; i < MAXTIMINGS; i++)
	{
		counter = 0;
		while ((digitalRead(DHT_PIN)) == laststate)
		{
			counter++;
			delayMicroseconds(1);
			if (counter == 255)
				break;
		}
		laststate = digitalRead( DHT_PIN );
 
		if (counter == 255)
			break;
		// Ignore first 3 transitions
		if ((i >= 4) && (i % 2 == 0))
		{
			// Shift data and store the bit
			data[j / 8] <<= 1;
			if (counter > 16)
				data[j / 8] |= 1;
			j++;
		}
	}

	// Check if we read 40 bits and if the checksum is correct
	if ((j >= 40) && (data[4] == ( (data[0] + data[1] + data[2] + data[3]) & 0xFF)))
	{
		// Convert and store humidity
		float hum = (float) data[0];
		float hum_dec = (float) data[1];
		*humidity = hum + hum_dec / 10.0;

		// Convert and store temperature
		float temp = (float) data[2];
		float temp_dec = (float) data[3];
		*temperature = temp + temp_dec / 10.0;

		return 1;
	}
	else
	{
		printf( "Data not good, skip\n" );
		return 0;
	}
}
