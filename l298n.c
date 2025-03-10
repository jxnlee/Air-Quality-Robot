#include "head.h"
#include "l298n.h"

void init_l298n()
{
	pinMode(L_MOTORS1_PIN, OUTPUT);
	pinMode(L_MOTORS2_PIN, OUTPUT);
	pinMode(L_MOTORS_SPD_PIN, OUTPUT);
	softPwmCreate(L_MOTORS_SPD_PIN, 0, 0xff);

	pinMode(R_MOTORS1_PIN, OUTPUT);
	pinMode(R_MOTORS2_PIN, OUTPUT);
	pinMode(R_MOTORS_SPD_PIN, OUTPUT);
	softPwmCreate(R_MOTORS_SPD_PIN, 0, 0xff);
}

void set_motors_speed(uint8_t speed)
{
	softPwmWrite(L_MOTORS_SPD_PIN, speed);
	softPwmWrite(R_MOTORS_SPD_PIN, speed);
}

void motors_off()
{
	turn_off(L_MOTORS1_PIN);
	turn_off(L_MOTORS2_PIN);
	turn_off(R_MOTORS1_PIN);
	turn_off(R_MOTORS2_PIN);
}

void drive_left_forward(uint8_t speed)
{
	softPwmWrite(L_MOTORS_SPD_PIN, speed);
	turn_on(L_MOTORS1_PIN);
	turn_off(L_MOTORS2_PIN);
}
void drive_left_backward(uint8_t speed)
{
	softPwmWrite(L_MOTORS_SPD_PIN, speed);
	turn_on(L_MOTORS2_PIN);
	turn_off(L_MOTORS1_PIN);
}
void drive_right_forward(uint8_t speed)
{
	softPwmWrite(R_MOTORS_SPD_PIN, speed);
	turn_on(R_MOTORS1_PIN);
	turn_off(R_MOTORS2_PIN);
}
void drive_right_backward(uint8_t speed)
{
	softPwmWrite(R_MOTORS_SPD_PIN, speed);
	turn_on(R_MOTORS2_PIN);
	turn_off(R_MOTORS1_PIN);
}

void drive_forward(uint8_t speed)
{
	drive_left_forward(speed);
	drive_right_forward(speed);
}

void drive_backward(uint8_t speed)
{
	drive_left_backward(speed);
	drive_right_backward(speed);
}