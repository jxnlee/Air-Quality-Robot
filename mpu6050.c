/*
	MPU6050 Interfacing with Raspberry Pi
	http://www.electronicwings.com
*/
#include "head.h"
#include <wiringPiI2C.h>

#define Device_Address 0x68	/*Device Address/Identifier for MPU6050*/

#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

int fd;

void init_mpu6050()
{
	fd = wiringPiI2CSetup(Device_Address);   /*Initializes I2C with device Address*/
	wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
	wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x00);	/* Write to power management register */
	wiringPiI2CWriteReg8 (fd, CONFIG, 0);		/* Write to Configuration register */
	wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 24);	/* Write to Gyro Configuration register */
	wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);	/*Write to interrupt enable register */
}
short read_raw_data(int addr)
{
	short high_byte,low_byte,value;
	high_byte = wiringPiI2CReadReg8(fd, addr);
	low_byte = wiringPiI2CReadReg8(fd, addr+1);
	value = (high_byte << 8) | low_byte;
	return value;
}

/*void ms_delay(int val){
	int i,j;
	for(i=0;i<=val;i++)
		for(j=0;j<1200;j++);
}*/

void read_accelerometer(float* x, float* y, float* z)
{
    *x = read_raw_data(ACCEL_XOUT_H) / 16384.0;
    printf("%.3f", *x);
    *y = read_raw_data(ACCEL_YOUT_H) / 16384.0;
    *z = read_raw_data(ACCEL_ZOUT_H) / 16384.0;
}

void read_gyroscope(float* x, float* y, float* z)
{
    *x = read_raw_data(GYRO_XOUT_H) / 131;
    *y = read_raw_data(GYRO_YOUT_H) / 131;
    *z = read_raw_data(GYRO_ZOUT_H) / 131;
}