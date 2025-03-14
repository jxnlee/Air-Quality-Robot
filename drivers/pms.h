/**
 * @file pms.h
 * @brief Header file for PMS5003 PM2.5 sensor driver functions.
 * 
 * This file contains the function declarations for initializing and reading data
 * from the PMS5003 PM2.5 sensor used in the Air Quality Robot.
 */
#ifndef	_PMS_
#define _PMS_

#include <stdint.h>
#include <stdio.h>

/// @brief Data structure to hold PMS5003 sensor readings.
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

/// @brief Initializes the PMS5003 sensor by opening the serial port.
/// @return 1 if the initialization is successful, 0 otherwise.
int init_pms();

/// @brief Reads data from the PMS5003 sensor.
/// @param particles Pointer to store the number of particles > 0.3um per 0.1L of air.
/// @return 1 if the data is read successfully, 0 otherwise.
int read_pms(uint16_t* particles);

/// @brief Prints the PMS5003 sensor readings to the console.
void print_pms_readings();

/// @brief Calculates the AQI for PM2.5 based on US standards.
/// @param concentration The PM2.5 concentration.
/// @return The AQI value.
uint16_t pm25_aqi_us(float concentration);

/// @brief Calculates the AQI for PM10 based on US standards.
/// @param concentration The PM10 concentration.
/// @return The AQI value.
uint16_t pm100_aqi_us(float concentration);

/// @brief Helper function to calculate linear AQI values.
/// @param aqi_high The high AQI value.
/// @param aqi_low The low AQI value.
/// @param conc_high The high concentration value.
/// @param conc_low The low concentration value.
/// @param concentration The current concentration.
/// @return The calculated AQI value.
float linear(uint16_t aqi_high, uint16_t aqi_low, float conc_high, float conc_low, float concentration);

/// @brief Closes the serial port used by the PMS5003 sensor.
void close_pms();


#endif
