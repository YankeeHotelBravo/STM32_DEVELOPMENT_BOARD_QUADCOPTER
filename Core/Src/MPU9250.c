/*
 * MPU9250.c
 *
 *  Created on: Dec 12, 2021
 *      Author: wesle
 */
#include "MPU9250.h"

//Variables
uint8_t MPU6500_WAI;
uint8_t AK8963_WAI;
uint8_t MPU9250_DRDY = 0;

unsigned char MPU9250_tx;
unsigned char MPU9250_rx;
unsigned char MPU9250_rx_buf[20];

float MPU9250_Gyro_LSB = 131.0;
float MPU9250_Acc_LSB = 16384.0;
float MPU9250_Mag_LSB = 0.15f;

//Functions
uint8_t MPU9250_Init(I2C_HandleTypeDef *I2Cx, uint8_t Gyro_FS, uint8_t Acc_FS, uint8_t DLPF_CFG, uint8_t A_DLPF_CFG)
{
	//Save LSB/Unit for both gyro and acc in order to use them later
	switch(Gyro_FS)
	{
	case 0: //250dps
		MPU9250_Gyro_LSB = 131.0;
		break;
	case 1: //500dps
		MPU9250_Gyro_LSB = 65.5;
		break;
	case 2: //1000dps
		MPU9250_Gyro_LSB = 32.8;
		break;
	case 3: //2000dps
		MPU9250_Gyro_LSB = 16.4;
		break;
	default:
		break;
	}

	switch(Acc_FS)
	{
	case 0: //2g
		MPU9250_Acc_LSB = 16384.0;
		break;
	case 1: //4g
		MPU9250_Acc_LSB = 8192.0;
		break;
	case 2: //8g
		MPU9250_Acc_LSB = 4096.0;
		break;
	case 3: //16g
		MPU9250_Acc_LSB = 2048.0;
		break;
	default:
		break;
	}

	HAL_I2C_Mem_Read(I2Cx, MPU6500_ADDR, MPU6500_WHO_AM_I, 1, &MPU9250_rx, 1, 100);
	if (MPU9250_rx == 0x71)
	{
		MPU9250_tx = 0x01; // Set Sampling by 2 = 500Hz
		HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_SMPLRT_DIV, 1, &MPU9250_tx, 1, 100);
		HAL_Delay(10);

		MPU9250_tx = DLPF_CFG; // Digital Low Pass Filter Setting
		HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_CONFIG, 1, &MPU9250_tx, 1, 100);
		HAL_Delay(10);

		MPU9250_tx = Gyro_FS << 3;
		HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_GYRO_CONFIG, 1, &MPU9250_tx, 1, 100);
		HAL_Delay(10);

		MPU9250_tx = Acc_FS << 3;
		HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_ACCEL_CONFIG, 1, &MPU9250_tx, 1, 100);
		HAL_Delay(10);

		MPU9250_tx = A_DLPF_CFG;
		HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_ACCEL_CONFIG2, 1, &MPU9250_tx, 1, 100);
		HAL_Delay(10);

		MPU9250_tx = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_PWR_MGMT_1, 1, &MPU9250_tx, 1, 100);
		HAL_Delay(10);

		return 1;
	}
	else
	{
		return 0;
	}
}

void MPU9250_Bypass(I2C_HandleTypeDef *I2Cx)
{
	MPU9250_tx = 0x00; //Disable Master Mode
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_USER_CTRL, 1, &MPU9250_tx, 1, 100); //Master Disable
	HAL_Delay(10);

	MPU9250_tx = 0b00000010; //Enable Bypass Mode
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_INT_PIN_CFG, 1, &MPU9250_tx, 1, 100); //Bypass Enable
	HAL_Delay(10);
}

void MPU9250_Master(I2C_HandleTypeDef *I2Cx)
{
	MPU9250_tx = 0x00; //Disable Bypass Mode
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_INT_PIN_CFG, 1, &MPU9250_tx, 1, 100); //Disable Bypass
	HAL_Delay(10);

	MPU9250_tx = 0b00100000; //Enable Master Mode
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_USER_CTRL, 1, &MPU9250_tx, 1, 100); //Master Enable
	HAL_Delay(10);

	MPU9250_tx = 0b00001101; //Set I2C Clock to 400kHz
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_I2C_MST_CTRL, 1, &MPU9250_tx, 1, 100); //Master Clock to 400kHz
	HAL_Delay(10);

	MPU9250_tx = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_PWR_MGMT_1, 1, &MPU9250_tx, 1, 100);
	HAL_Delay(10);
}

uint8_t MPU9250_AK8963_Setup(I2C_HandleTypeDef *I2Cx, MPU9250_t *Datastruct)
{
	MPU9250_tx = AK8963_CNTL1_Continous2; //Continuous Mode 2 (100Hz) + 16Bit Output
	HAL_I2C_Mem_Write(I2Cx, AK8963_ADDR, AK8963_CNTL1, 1, &MPU9250_tx, 1, 100);
	HAL_Delay(10);

	HAL_I2C_Mem_Read(I2Cx, AK8963_ADDR, AK8963_ASAX, 1, &MPU9250_rx_buf[0], 3, 100);
	HAL_Delay(10);

	Datastruct->ASAX = MPU9250_rx_buf[0];
	Datastruct->ASAY = MPU9250_rx_buf[1];
	Datastruct->ASAZ = MPU9250_rx_buf[2];

	HAL_I2C_Mem_Read(I2Cx, AK8963_ADDR, AK8963_WIA, 1, &AK8963_WAI, 3, 100);
	HAL_Delay(10);

	if(AK8963_WAI == 0b01001000) return 1;
	else return 0;
}

void MPU9250_Slave0_Enable(I2C_HandleTypeDef *I2Cx)
{
	MPU9250_tx = (AK8963_ADDR >> 1) | 0x80; //Access Slave into read mode
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_I2C_SLV0_ADDR, 1, &MPU9250_tx, 1, 100);
	HAL_Delay(10);

	MPU9250_tx = AK8963_HXL; //Slave REG for reading to take place
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_I2C_SLV0_REG, 1, &MPU9250_tx, 1, 100);
	HAL_Delay(10);

	MPU9250_tx = 0x80 | 0x07; //Number of data bytes
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, MPU6500_I2C_SLV0_CTRL, 1, &MPU9250_tx, 1, 100);
	HAL_Delay(10);
}

void MPU9250_Read_All(I2C_HandleTypeDef *I2Cx)
{
	HAL_I2C_Mem_Read_DMA(I2Cx, MPU6500_ADDR, MPU6500_ACCEL_XOUT_H, 1, &MPU9250_rx_buf[0], 20);
}

void MPU9250_Parsing(MPU9250_t *DataStruct)
{
	DataStruct->Ax_Raw = (MPU9250_rx_buf[0] << 8 | MPU9250_rx_buf[1]);
	DataStruct->Ay_Raw = -(MPU9250_rx_buf[2] << 8 | MPU9250_rx_buf[3]);
	DataStruct->Az_Raw = -(MPU9250_rx_buf[4] << 8 | MPU9250_rx_buf[5]);
	// Didn't Save Temp Value
	DataStruct->Gx_Raw = (MPU9250_rx_buf[8] << 8 | MPU9250_rx_buf[9]);
	DataStruct->Gy_Raw = -(MPU9250_rx_buf[10] << 8 | MPU9250_rx_buf[11]);
	DataStruct->Gz_Raw = -(MPU9250_rx_buf[12] << 8 | MPU9250_rx_buf[13]);

	DataStruct->My_Raw = -(MPU9250_rx_buf[15] << 8 | MPU9250_rx_buf[14]);
	DataStruct->Mx_Raw = (MPU9250_rx_buf[17] << 8 | MPU9250_rx_buf[16]);
	DataStruct->Mz_Raw = (MPU9250_rx_buf[19] << 8 | MPU9250_rx_buf[18]);

	DataStruct->Gx = DataStruct->Gx_Raw / MPU9250_Gyro_LSB;
	DataStruct->Gy = DataStruct->Gy_Raw / MPU9250_Gyro_LSB;
	DataStruct->Gz = DataStruct->Gz_Raw / MPU9250_Gyro_LSB;
	DataStruct->Ax = DataStruct->Ax_Raw / MPU9250_Acc_LSB;
	DataStruct->Ay = DataStruct->Ay_Raw / MPU9250_Acc_LSB;
	DataStruct->Az = DataStruct->Az_Raw / MPU9250_Acc_LSB;
	DataStruct->Mx = (DataStruct->Mx_Raw * ((DataStruct->ASAX - 128) / 256 + 1)) / MPU9250_Mag_LSB;
	DataStruct->My = (DataStruct->My_Raw * ((DataStruct->ASAY - 128) / 256 + 1)) / MPU9250_Mag_LSB;
	DataStruct->Mz = (DataStruct->Mz_Raw * ((DataStruct->ASAZ - 128) / 256 + 1)) / MPU9250_Mag_LSB;

	DataStruct->Gx -= DataStruct->Gx_Offset;
	DataStruct->Gy -= DataStruct->Gy_Offset;
	DataStruct->Gz -= DataStruct->Gz_Offset;
	DataStruct->Gx_Rad = DataStruct->Gx * D2R;
	DataStruct->Gy_Rad = DataStruct->Gy * D2R;
	DataStruct->Gz_Rad = DataStruct->Gz * D2R;
	DataStruct->Mx -= DataStruct->Mx_Offset;
	DataStruct->My -= DataStruct->My_Offset;
	DataStruct->Mz -= DataStruct->Mz_Offset;
}

void MPU9250_Parsing_NoOffset(MPU9250_t *DataStruct)
{
	DataStruct->Ax_Raw = (MPU9250_rx_buf[0] << 8 | MPU9250_rx_buf[1]);
	DataStruct->Ay_Raw = -(MPU9250_rx_buf[2] << 8 | MPU9250_rx_buf[3]);
	DataStruct->Az_Raw = -(MPU9250_rx_buf[4] << 8 | MPU9250_rx_buf[5]);
	// Didn't Save Temp Value
	DataStruct->Gx_Raw = (MPU9250_rx_buf[8] << 8 | MPU9250_rx_buf[9]);
	DataStruct->Gy_Raw = -(MPU9250_rx_buf[10] << 8 | MPU9250_rx_buf[11]);
	DataStruct->Gz_Raw = -(MPU9250_rx_buf[12] << 8 | MPU9250_rx_buf[13]);

	DataStruct->My_Raw = -(MPU9250_rx_buf[15] << 8 | MPU9250_rx_buf[14]);
	DataStruct->Mx_Raw = (MPU9250_rx_buf[17] << 8 | MPU9250_rx_buf[16]);
	DataStruct->Mz_Raw = (MPU9250_rx_buf[19] << 8 | MPU9250_rx_buf[18]);

	DataStruct->Gx = DataStruct->Gx_Raw / MPU9250_Gyro_LSB* D2R;
	DataStruct->Gy = DataStruct->Gy_Raw / MPU9250_Gyro_LSB* D2R;
	DataStruct->Gz = DataStruct->Gz_Raw / MPU9250_Gyro_LSB* D2R;
	DataStruct->Gx_Rad = DataStruct->Gx * D2R;
	DataStruct->Gy_Rad = DataStruct->Gy * D2R;
	DataStruct->Gz_Rad = DataStruct->Gz * D2R;
	DataStruct->Ax = DataStruct->Ax_Raw / MPU9250_Acc_LSB;
	DataStruct->Ay = DataStruct->Ay_Raw / MPU9250_Acc_LSB;
	DataStruct->Az = DataStruct->Az_Raw / MPU9250_Acc_LSB;
	DataStruct->Mx = (DataStruct->Mx_Raw * ((DataStruct->ASAX - 128) / 256 + 1)) / MPU9250_Mag_LSB;
	DataStruct->My = (DataStruct->My_Raw * ((DataStruct->ASAY - 128) / 256 + 1)) / MPU9250_Mag_LSB;
	DataStruct->Mz = (DataStruct->Mz_Raw * ((DataStruct->ASAZ - 128) / 256 + 1)) / MPU9250_Mag_LSB;
}
