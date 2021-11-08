#ifndef GYROSCOPE_H
#define	GYROSCOPE_H

#include "stm32f1xx_hal.h"
#include "lcd.h"

#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

void MPU6050_Init (I2C_HandleTypeDef hi2c2);
void MPU6050_Read_Accel (I2C_HandleTypeDef hi2c2, float* Ax, float* Ay, float* Az);
void MPU6050_Read_Gyro (I2C_HandleTypeDef hi2c2, float* Gx, float* Gy, float* Gz);
void print_Accel_Gyro_OnLCD(float* Ax, float* Ay, float* Az, float* Gx, float* Gy, float* Gz );

#endif 
