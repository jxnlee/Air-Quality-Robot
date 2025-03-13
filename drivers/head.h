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
/*
 +-----+-----+---------+------+---+---Pi 5---+---+------+---------+-----+-----+
 | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
 |   2 |   8 |   SDA.1 | ALT3 | 1 |  3 || 4  |   |      | 5v      |     |     |
 |   3 |   9 |   SCL.1 | ALT3 | 1 |  5 || 6  |   |      | 0v      |     |     |
 |   4 |   7 | GPIO. 7 |   -  | 0 |  7 || 8  | 0 |  -   | TxD     | 15  | 14  |
 |     |     |      0v |      |   |  9 || 10 | 0 |  -   | RxD     | 16  | 15  |
 |  17 |   0 | GPIO. 0 |   -  | 0 | 11 || 12 | 0 |  -   | GPIO. 1 | 1   | 18  |
 |  27 |   2 | GPIO. 2 |   -  | 0 | 13 || 14 |   |      | 0v      |     |     |
 |  22 |   3 | GPIO. 3 |   -  | 0 | 15 || 16 | 0 |  -   | GPIO. 4 | 4   | 23  |
 |     |     |    3.3v |      |   | 17 || 18 | 0 |  -   | GPIO. 5 | 5   | 24  |
 |  10 |  12 |    MOSI |   -  | 0 | 19 || 20 |   |      | 0v      |     |     |
 |   9 |  13 |    MISO |   -  | 0 | 21 || 22 | 0 |  -   | GPIO. 6 | 6   | 25  |
 |  11 |  14 |    SCLK |   -  | 0 | 23 || 24 | 0 |  -   | CE0     | 10  | 8   |
 |     |     |      0v |      |   | 25 || 26 | 0 |  -   | CE1     | 11  | 7   |
 |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
 |   5 |  21 | GPIO.21 |   -  | 0 | 29 || 30 |   |      | 0v      |     |     |
 |   6 |  22 | GPIO.22 |   -  | 0 | 31 || 32 | 0 |  -   | GPIO.26 | 26  | 12  |
 |  13 |  23 | GPIO.23 |   -  | 0 | 33 || 34 |   |      | 0v      |     |     |
 |  19 |  24 | GPIO.24 |   -  | 0 | 35 || 36 | 0 |  -   | GPIO.27 | 27  | 16  |
 |  26 |  25 | GPIO.25 |   -  | 0 | 37 || 38 | 0 |  -   | GPIO.28 | 28  | 20  |
 |     |     |      0v |      |   | 39 || 40 | 0 |  -   | GPIO.29 | 29  | 21  |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 +-----+-----+---------+------+---+---Pi 5---+---+------+---------+-----+-----+
*/

// Helper Functions / Variables / Structs
long long pulseIn(int pin, int level);

// Defined Data Struct (Adafruit_PM25AQI)



#endif
