#ifndef	_PMS_
#define _PMS_

#include <stdint.h>
#include <stdio.h>

typedef struct {
	uint16_t framelen;		// how long data chunk is
	uint16_t pm10_standard,		// Standard PM1.0
		 pm25_standard,		// Standard PM2.5
		 pm100_standard;	// Standard PM10.0
	uint16_t pm10_env,		// Environmental PM1.0
		 pm25_env,		// Environmental PM2.5
		 pm100_env;		// Environmental PM10.0
	uint16_t particles_03um,	// 0.3um Particle Count
		 particles_05um,	// 0.5um Particle Count
		 particles_10um,	// 1.0um Particle Count
		 particles_25um,	// 2.5um Paritcle Count
		 particles_50um,	// 5.0um Particle Count
		 particles_100um;	// 10.0um Particle Count
	uint16_t unused;		// Unused (version + error code)
	uint16_t checksum;		// Packet checksum
	
	// AQI Conversion Results
	uint16_t aqi_pm25_us;		// pm2.5 AQI of United States
	uint16_t aqi_pm100_us;		// pm10 AQI of United States
} PM25_Data;

int init_pms();

int read_pms(uint16_t* particles);

void print_pms_readings();

uint16_t pm25_aqi_us(float concentration);
uint16_t pm100_aqi_us(float concentration);
float linear(uint16_t aqi_high, uint16_t aqi_low, float conc_high, float conc_low, float concentration);

void close_pms();


#endif
