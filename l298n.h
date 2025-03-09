#ifndef _L298N_
#define	_L298N_
#include <softPwm.h>
void init_l298n();
void set_motors_speed(uint8_t speed);
void motors_off();
void drive_left(int duration);
void drive_right(int duration);
void drive_forward(int duration);
void drive_backward(int duration);
#endif
