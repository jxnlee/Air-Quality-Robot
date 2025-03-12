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
#define DHT_PIN			7
#define TRIGGER_PIN		0
#define ECHO_PIN		2
#define FAN_PIN         1
#define NION_GEN_PIN    5
#define L_MOTORS_SPD_PIN	23
#define L_MOTORS1_PIN		25
#define L_MOTORS2_PIN		24
#define R_MOTORS1_PIN		28
#define R_MOTORS2_PIN		27
#define R_MOTORS_SPD_PIN	29

// Helper Functions / Variables / Structs
long long pulseIn(int pin, int level);

// Defined Data Struct (Adafruit_PM25AQI)



#endif
