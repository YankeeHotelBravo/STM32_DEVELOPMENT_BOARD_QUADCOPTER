/*
 * FS_iA6B.c
 *
 *  Created on: 2021. 7. 6.
 *      Author: muk20
 */
#include "FS-iA6B.h"

FSiA6B_iBus iBus;
uint8_t iBus_return;
uint8_t iBus_failsafe = 0;
uint8_t motor_arming_flag = 0;
uint8_t iBus_rx_cnt = 0;

unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len)
{
	unsigned short chksum = 0xffff;

	for(int i=0; i<len-2;i++)
	{
		chksum = chksum - data[i];
	}

	return ((chksum&0x00ff)==data[30]) && ((chksum>>8) && data[31]);

}

void iBus_Parsing(unsigned char* data, FSiA6B_iBus* iBus)
{
	iBus->RH = (data[2] | data[3]<<8) & 0x0FFF;
	iBus->RV = (data[4] | data[5]<<8) & 0x0FFF;
	iBus->LV = (data[6] | data[7]<<8) & 0x0FFF;
	iBus->LH = (data[8] | data[9]<<8) & 0x0FFF;
	iBus->SwA = (data[10] | data[11]<<8) & 0x0FFF;
	iBus->SwB = (data[12] | data[13]<<8) & 0x0FFF;
	iBus->VrA = (data[14] | data[15]<<8) & 0x0FFF;
	iBus->VrB = (data[16] | data[17]<<8) & 0x0FFF;
	iBus->SwC = (data[18] | data[19]<<8) & 0x0FFF;
	iBus->SwD = (data[20] | data[21]<<8) & 0x0FFF;

	iBus->FailSafe = (data[13] >> 4);
}

unsigned char iBus_isActiveFailSafe(FSiA6B_iBus* iBus)
{
	return iBus->FailSafe != 0;
}

