/**
 * @file pms.c
 * @brief PMS5003 PM2.5 sensor driver functions.
 * 
 * This file contains the implementation of functions to initialize and read data
 * from the PMS5003 PM2.5 sensor used in the Air Quality Robot.
 * 
 * Note: Most of the Code references Adafruit's PM2.5 Air Quality Sensor library which is 
 * ported over except for the read_pms function which has been modified for our needs
 *  (the remaining functions that are unused are kept in for the sake of future use and implementation)
 *
 * References: https://github.com/adafruit/Adafruit_PM25AQI
 */
#include "pms.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <wiringSerial.h>

// defined baud rate
#define PMS_BAUDRATE	9600
// defined serial port path (to usb tty converter)
const char PMS_SERIAL_PORT[] = "/dev/ttyUSB0";

// Serial port number (fd)
static int serial_port;

// PM25_Data to store PMS Reading Data
PM25_Data pms5003_data; 

/// @brief Initializes the PMS5003 sensor by opening the serial port.
/// @return 1 if the initialization is successful, 0 otherwise.
int init_pms()
{
	if ((serial_port = serialOpen(PMS_SERIAL_PORT, PMS_BAUDRATE)) < 0)
	{
		fprintf(stderr, "Failed to open serial port %s.\n", PMS_SERIAL_PORT);
		return 0;
	}
	else
	{
		printf("SUCCESSFULLY OPENED SERIAL PORT %s WITH BAUDRATE %d!\n", PMS_SERIAL_PORT, PMS_BAUDRATE);
		return 1;
	}
}

/// @brief Reads data from the PMS5003 sensor.
/// @param particles Pointer to store the number of particles > 0.3um per 0.1L of air.
/// @return 1 if the data is read successfully, 0 otherwise.
int read_pms(uint16_t* particles)
{
	PM25_Data* data = &pms5003_data;
	uint8_t buffer[32];
	uint16_t sum = 0;

	// make sure serial data is available from serial port
	if (serialDataAvail(serial_port) < 32) return 0;

	// get bytes and put them into data buffer
	for (uint8_t i = 0; i < 32; i++) buffer[i] = serialGetchar(serial_port);

	// ensure that the data buffer has the appropriate starting bytes
	if (buffer[0] != 0x42 || buffer[1] != 0x4d) return 0;

	// calculate checksum
	for (uint8_t i = 0; i < 30; i++) sum += buffer[i];

	// Parse buffer to extract PM2.5 data
	uint16_t buffer_u16[15];
	for (uint8_t i = 0; i < 15; i++)
	{
		buffer_u16[i] = buffer[2 + i * 2 + 1];
		buffer_u16[i] += (buffer[2 + i * 2] << 8);
	}

	// copy to PM2.5 Data Structure
	memcpy((void*) data, (void*) buffer_u16, 30);
	
	// compare calculated sum with actual checksum
	if (sum != data->checksum) return 0;

	// calculate and store AQI values for US
	data->aqi_pm25_us = pm25_aqi_us(data->pm25_env);
	data->aqi_pm100_us = pm100_aqi_us(data->pm100_env);

	// store concentration of 0.3um or more particles measured
	*particles = data->particles_03um;

	return 1;
}

/// @brief Calculates the AQI for PM2.5 based on US standards.
/// @param concentration The PM2.5 concentration.
/// @return The AQI value.
uint16_t pm25_aqi_us(float concentration)
{
  float c;
  float AQI;
  c = ((uint16_t) (10 * concentration)) / 10.0;
  if (c < 0)
    AQI = 0.0;
  else if (c >= 0 && c < 12.1f) {
    AQI = linear(50, 0, 12, 0, c);
  } else if (c >= 12.1f && c < 35.5f) {
    AQI = linear(100, 51, 35.4f, 12.1f, c);
  } else if (c >= 35.5f && c < 55.5f) {
    AQI = linear(150, 101, 55.4f, 35.5f, c);
  } else if (c >= 55.5f && c < 150.5f) {
    AQI = linear(200, 151, 150.4f, 55.5f, c);
  } else if (c >= 150.5f && c < 250.5f) {
    AQI = linear(300, 201, 250.4f, 150.5f, c);
  } else if (c >= 250.5f && c < 350.5f) {
    AQI = linear(400, 301, 350.4f, 250.5f, c);
  } else if (c >= 350.5f && c < 500.5f) {
    AQI = linear(500, 401, 500.4f, 350.5f, c);
  } else {
    AQI = 99999.0; //
  }
  return (uint16_t) (AQI+0.5);
}

/// @brief Calculates the AQI for PM10 based on US standards.
/// @param concentration The PM10 concentration.
/// @return The AQI value.
uint16_t pm100_aqi_us(float concentration) 
{
  float c;
  float AQI;
  c = concentration;
  if (c < 0)
    AQI = 0.0;
  else if (c < 55) {
    AQI = linear(50, 0, 55, 0, c);
  } else if (c < 155) {
    AQI = linear(100, 51, 155, 55, c);
  } else if (c < 255) {
    AQI = linear(150, 101, 255, 155, c);
  } else if (c < 355) {
    AQI = linear(200, 151, 355, 255, c);
  } else if (c < 425) {
    AQI = linear(300, 201, 425, 355, c);
  } else if (c < 505) {
    AQI = linear(400, 301, 505, 425, c);
  } else if (c < 605) {
    AQI = linear(500, 401, 605, 505, c);
  } else {
    AQI = 99999.0; //
  }
  return (uint16_t) (AQI+0.5);
}

/// @brief Helper function to calculate linear AQI values.
/// @param aqi_high The high AQI value.
/// @param aqi_low The low AQI value.
/// @param conc_high The high concentration value.
/// @param conc_low The low concentration value.
/// @param concentration The current concentration.
/// @return The calculated AQI value.
float linear(uint16_t aqi_high, uint16_t aqi_low, float conc_high, float conc_low, float concentration) 
{
	 return ((concentration - conc_low) / (conc_high - conc_low))
		* (aqi_high - aqi_low)
		+ aqi_low;
}

/// @brief Prints the PMS5003 sensor readings to the console.
void print_pms_readings()
{
	PM25_Data* data = &pms5003_data;
	printf("---------------------------------------\n");
  	printf("Concentration Units (standard)\n");
	printf("PM 1.0: %hu\t\tPM 2.5: %hu\t\tPM 10: %hu\n", data->pm10_standard, data->pm25_standard, data->pm100_standard);
  	printf("---------------------------------------\n");
  	printf("Concentration Units (environmental)\n");
  	printf("PM 1.0: %hu\t\tPM 2.5: %hu\t\tPM 10: %hu\n", data->pm10_env, data->pm25_env, data->pm100_env);
  	printf("---------------------------------------\n");
  	printf("Particles > 0.3um / 0.1L air: %hu\n", data->particles_03um);
  	printf("Particles > 0.5um / 0.1L air: %hu\n", data->particles_05um);
  	printf("Particles > 1.0um / 0.1L air: %hu\n", data->particles_10um);
  	printf("Particles > 2.5um / 0.1L air: %hu\n", data->particles_25um);
  	printf("Particles > 5.0um / 0.1L air: %hu\n", data->particles_50um);
  	printf("Particles > 10 um / 0.1L air: %hu\n", data->particles_100um);
  	printf("---------------------------------------\n");
  	printf("AQI\n");
  	printf("PM2.5 AQI US: %hu\tPM10  AQI US: %hu\n", data->aqi_pm25_us, data->aqi_pm100_us);
 	printf("---------------------------------------\n");
}

/// @brief Closes the serial port used by the PMS5003 sensor.
void close_pms()
{
	serialClose(serial_port);
}
