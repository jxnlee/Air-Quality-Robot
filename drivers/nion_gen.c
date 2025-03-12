#include "head.h"
#include "nion_gen.h"

void init_nion_gen()
{
    pinMode(NION_GEN_PIN, OUTPUT);
    turn_off(NION_GEN_PIN);
}

void start_nion_gen()
{
    turn_on(NION_GEN_PIN);
}

void stop_nion_gen()
{
    turn_off(NION_GEN_PIN);
}