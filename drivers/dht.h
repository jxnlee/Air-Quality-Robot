/**
 * @file dht.h
 * @brief Header file for DHT11 sensor driver.
 */
#ifndef	_DHT_
#define _DHT_

/// @brief Reads temperature and humidity from the DHT sensor.
/// @param temperature Pointer to store the read temperature.
/// @param humidity Pointer to store the read humidity.
/// @return 1 if the data is read successfully, 0 otherwise.
int read_dht(float* temperature, float* humidity);

#endif
