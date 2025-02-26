#ifndef _HEAD_

#include <stdio.h>
#include <wiringPi.h>
#include <stdint.h>
#include <stdlib.h>

// Macros
#define turn_on(pin)	digitalWrite(pin, 1)
#define turn_off(pin)	digitalWrite(pin, 0)
#define read(pin)	digitalRead(pin)
#define write(pin, o)	digitalWrite(pin, output)


#define LOW	0
#define HIGH	1

// Pinout
#define DHT_PIN	7
#define TRIGGER_PIN	4
#define ECHO_PIN	5

// Helper Functions / Variables / Structs
unsigned long pulseIn(int pin, int level);

// Defined Data Struct (Adafruit_PM25AQI)



#endif
