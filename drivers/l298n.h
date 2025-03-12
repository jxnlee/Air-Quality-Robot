#ifndef _L298N_
#define	_L298N_
#include <softPwm.h>
void init_l298n();
void set_motors_speed(uint8_t speed);
void motors_off();
void drive_left_forward(uint8_t speed);
void drive_left_backward(uint8_t speed);
void drive_right_forward(uint8_t speed);
void drive_right_backward(uint8_t speed);
void drive_forward(uint8_t speed);
void drive_backward(uint8_t speed);
#endif
