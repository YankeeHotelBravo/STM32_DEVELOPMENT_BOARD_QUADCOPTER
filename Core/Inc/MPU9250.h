/*
 * MPU9250.h
 *
 *  Created on: Dec 12, 2021
 *      Author: wesle
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include <main.h>
#define D2R 0.01745329252



// MPU6500 Register
#define MPU6500_ADDR 0xD0 //Already Left Shifted

#define MPU6500_SMPLRT_DIV 0x19
#define MPU6500_CONFIG 0x1A
#define MPU6500_GYRO_CONFIG 0x1B
#define MPU6500_ACCEL_CONFIG 0x1C
#define MPU6500_ACCEL_CONFIG2  0x1D

#define MPU6500_I2C_MST_CTRL 0x24

#define MPU6500_I2C_SLV0_ADDR 0x25
#define MPU6500_I2C_SLV0_REG 0x26
#define MPU6500_I2C_SLV0_CTRL 0x27

#define MPU6500_I2C_MST_STATUS 0x36
#define MPU6500_INT_PIN_CFG 0x37
#define MPU6500_INT_ENABLE 0x38
#define MPU6500_INT_STATUS 0x3A

#define MPU6500_ACCEL_XOUT_H 0x3B

#define MPU6500_USER_CTRL 0x6A
#define MPU6500_PWR_MGMT_1 0x6B
#define MPU6500_PWR_MGMT_2 0x6C
#define MPU6500_WHO_AM_I 0x75

//AK8963 Register
#define AK8963_ADDR 0x18 //Already Left Shifted
#define AK8963_WIA 0x00
#define AK8963_HXL 0x03
#define AK8963_CNTL1 0x0A
#define AK8963_CNTL2 0x0B

#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12

//AK8963 Settings
#define AK8963_CNTL1_Continous2 0b00010110

//Datastruct
typedef struct MPU9250_t
{
	int16_t Ax_Raw;
	int16_t Ay_Raw;
	int16_t Az_Raw;

	float Ax;
	float Ay;
	float Az;

	int16_t Gx_Raw;
	int16_t Gy_Raw;
	int16_t Gz_Raw;

	int16_t Gx_Offset;
	int16_t Gy_Offset;
	int16_t Gz_Offset;

	float Gx;
	float Gy;
	float Gz;

	int16_t Mx_Raw;
	int16_t My_Raw;
	int16_t Mz_Raw;

	uint8_t ASAX;
	uint8_t ASAY;
	uint8_t ASAZ;

	float Mx_Min;
	float My_Min;
	float Mz_Min;

	float Mx_Max;
	float My_Max;
	float Mz_Max;

	float Mx_Offset;
	float My_Offset;
	float Mz_Offset;

	float Mx;
	float My;
	float Mz;

}MPU9250_t;

MPU9250_t MPU9250;

//Functions
uint8_t MPU9250_Init(I2C_HandleTypeDef *I2Cx, uint8_t Gyro_FS, uint8_t Acc_FS, uint8_t DLPF_CFG, uint8_t A_DLPF_CFG);
void MPU9250_Bypass(I2C_HandleTypeDef *I2Cx);
void MPU9250_Master(I2C_HandleTypeDef *I2Cx);
uint8_t MPU9250_AK8963_Setup(I2C_HandleTypeDef *I2Cx, MPU9250_t *Datastruct);
void MPU9250_Slave0_Enable(I2C_HandleTypeDef *I2Cx);
void MPU9250_Read_All(I2C_HandleTypeDef *I2Cx);
void MPU9250_Parsing(MPU9250_t *DataStruct);
void MPU9250_Parsing_NoOffset(MPU9250_t *DataStruct);

#endif /* INC_MPU9250_H_ */
