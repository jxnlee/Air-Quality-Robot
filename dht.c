#include <stdio.h>
#include <stdint.h>
#include "dht.h"
#include "head.h"

#define MAXTIMINGS	100


int data[5] = { 0, 0, 0, 0, 0 };

__attribute__((visibility("default")))

int read_dht(float* temperature, float* humidity)
{
	uint8_t laststate	= HIGH;
	uint8_t counter		= 0;
	uint8_t j		= 0, i;
	// float	f; 
 
	data[0] = data[1] = data[2] = data[3] = data[4] = 0;
 
	pinMode( DHT_PIN, OUTPUT );
	digitalWrite( DHT_PIN, LOW );
	delay( 18 );
	digitalWrite( DHT_PIN, HIGH );
	delayMicroseconds( 40 );
	pinMode( DHT_PIN, INPUT );
 
	for ( i = 0; i < MAXTIMINGS; i++ )
	{
		counter = 0;
		while ( digitalRead( DHT_PIN ) == laststate )
		{
			counter++;
			delayMicroseconds( 1 );
			if ( counter == 255 )
			{
				break;
			}
		}
		laststate = digitalRead( DHT_PIN );
 
		if ( counter == 255 )
			break;
 
		if ( (i >= 4) && (i % 2 == 0) )
		{
			data[j / 8] <<= 1;
			if ( counter > 16 )
				data[j / 8] |= 1;
			j++;
		}
	}
 
	if ( (j >= 40) &&
	     (data[4] == ( (data[0] + data[1] + data[2] + data[3]) & 0xFF) ) )
	{
		// f = data[2] * 9. / 5. + 32;
		//printf( "Humidity = %d.%d %% Temperature = %d.%d C (%.1f F)\n",data[0], data[1], data[2], data[3], f );
		//data[0] = data[1] = data[2] = data[3] = 5;
		float hum = (float) data[0];
		float hum_dec = (float) data[1];
		*humidity = hum + hum_dec / 10.0;
		float temp = (float) data[2];
		float temp_dec = (float) data[3];
		*temperature = temp + temp_dec / 10.0;
		return 1;
	}else  {
		//printf( "Data not good, skip\n" );
		return 0;
	}
}
