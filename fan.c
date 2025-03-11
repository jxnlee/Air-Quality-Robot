#include "head.h"
#include "fan.h"

void init_fan()
{
    pinMode(FAN_PIN, OUTPUT);
    turn_off(FAN_PIN);
}

void start_fan()
{
    turn_on(FAN_PIN);
}

void stop_fan()
{
    turn_off(FAN_PIN);
}