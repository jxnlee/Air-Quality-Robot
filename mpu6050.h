#ifndef _MPU6050_
#define _MPU6050_

void init_mpu6050();
short read_raw_data(int addr);
void read_accelerometer(float* x, float* y, float* z);
void read_gyroscope(float* x, float* y, float* z);

#endif