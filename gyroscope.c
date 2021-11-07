#include "gyroscope.h"

void MPU6050_Init (I2C_HandleTypeDef hi2c2)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ?2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ?250 ?s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}


void MPU6050_Read_Accel (I2C_HandleTypeDef hi2c2, float* Ax, float* Ay, float* Az)
{
	uint8_t Rec_Data[6];
	int16_t Accel_X_RAW = 0;
	int16_t Accel_Y_RAW = 0;
	int16_t Accel_Z_RAW = 0;
	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	//convert the RAW values into acceleration in 'g'

	*Ax = Accel_X_RAW/16384.0;
	*Ay = Accel_Y_RAW/16384.0;
	*Az = Accel_Z_RAW/16384.0;
}


void MPU6050_Read_Gyro (I2C_HandleTypeDef hi2c2, float* Gx, float* Gy, float* Gz)
{
	uint8_t Rec_Data[6];
	int16_t Gyro_X_RAW = 0;
	int16_t Gyro_Y_RAW = 0;
	int16_t Gyro_Z_RAW = 0;
	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	// convert the RAW values into degree/second

	*Gx = Gyro_X_RAW/131.0;
	*Gy = Gyro_Y_RAW/131.0;
	*Gz = Gyro_Z_RAW/131.0;
}


void print_Accel_Gyro_OnLCD(float* Ax, float* Ay, float* Az, float* Gx, float* Gy, float* Gz )
{
	//print the formal 
	LCD_DrawString(5 ,5 ,"Gx : ");
	LCD_DrawString(5 ,5 + (HEIGHT_EN_CHAR+2) ,"Gy : ");
	LCD_DrawString(5 ,5 + 2*(HEIGHT_EN_CHAR+2),"Gz : ");
	LCD_DrawString(5 ,5 + 3*(HEIGHT_EN_CHAR+2),"Ax : ");
	LCD_DrawString(5 ,5 + 4*(HEIGHT_EN_CHAR+2) ,"Ay : ");
	LCD_DrawString(5 ,5 + 5*(HEIGHT_EN_CHAR+2),"Az : ");
	
	char buf [10] = "";
	
	sprintf(buf, "% 06.1f", *Gx );
	LCD_DrawString(5 + WIDTH_EN_CHAR*5,5,buf);
	sprintf(buf, "% 06.1f", *Gy );
	LCD_DrawString(5 + WIDTH_EN_CHAR*5,5+ (HEIGHT_EN_CHAR+2),buf);
	sprintf(buf, "% 06.1f", *Gz );
	LCD_DrawString(5 + WIDTH_EN_CHAR*5,5+ 2*(HEIGHT_EN_CHAR+2),buf);
		
	sprintf(buf, "% .3f", *Ax );
	LCD_DrawString(5 + WIDTH_EN_CHAR*5,5+ 3*(HEIGHT_EN_CHAR+2),buf);
	sprintf(buf, "% .3f", *Ay );
	LCD_DrawString(5 + WIDTH_EN_CHAR*5,5+ 4*(HEIGHT_EN_CHAR+2),buf);
	sprintf(buf, "% .3f", *Az );
	LCD_DrawString(5 + WIDTH_EN_CHAR*5,5+ 5*(HEIGHT_EN_CHAR+2),buf);
}
