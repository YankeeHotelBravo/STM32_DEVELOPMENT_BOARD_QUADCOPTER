#include "M8N.h"

M8N_UBX_NAV_POSLLH posllh;
M8N_UBX_NAV_PVT pvt;


const unsigned char UBX_CFG_PRT[] = {
0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x01, 0x00,
0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x79
}; //UBX Protocol In, Out, UART1, 8N1-9600

const unsigned char UBX_CFG_MSG[] = {
0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01,
0x00, 0x00, 0x00, 0x00, 0x13, 0xBE
}; //NAV-POSLLH(01-02), UART1

const unsigned char UBX_CFG_RATE[] = {
0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00,
0x01, 0x00, 0xDE, 0x6A
}; //GPS Time, 5Hz Navigation Frequency

const unsigned char UBX_CFG_CFG[] = {
0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00,
0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31,
0xBF
}; //Save current configuration, Devices: BBR, FLASH, I2C-EEPROM, SPI-FLASH,

const unsigned char UBX_CFG_MSGPVT[] = {
0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01,
0x00, 0x00, 0x00, 0x00, 0x18, 0xE1
}; //NAV-POSLLH(01-07), UART1

void M8N_TransmitData(unsigned char* data, unsigned char len)
{
	for(int i=0; i<len; i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(UART4));
		LL_USART_TransmitData8(UART4, *(data+i));
	}
}

void M8N_Initialization(void)
{
	M8N_TransmitData(&UBX_CFG_PRT[0], sizeof(UBX_CFG_PRT));
	HAL_Delay(100);
//	M8N_TransmitData(&UBX_CFG_MSG[0], sizeof(UBX_CFG_MSG));
	M8N_TransmitData(&UBX_CFG_MSGPVT[0], sizeof(UBX_CFG_MSGPVT));
	HAL_Delay(100);
	M8N_TransmitData(&UBX_CFG_RATE[0], sizeof(UBX_CFG_RATE));
	HAL_Delay(100);
	M8N_TransmitData(&UBX_CFG_CFG[0], sizeof(UBX_CFG_CFG));
	HAL_Delay(100);
}

unsigned char M8N_UBX_CHKSUM_Check(unsigned char* data, unsigned char len)
{
	unsigned char CK_A = 0, CK_B = 0;

	for(int i=2;i<len-2;i++)
	{
		CK_A = CK_A + data[i];
		CK_B = CK_B + CK_A;
	}

	return ((CK_A == data[len-2]) && (CK_B == data[len-1]));
}

void M8N_UBX_NAV_POSLLH_Parsing(unsigned char* data, M8N_UBX_NAV_POSLLH* posllh)
{
	posllh->CLASS = data[2];
	posllh->ID = data[3];
	posllh->length = data[4] | data [5]<<8;

	posllh->iTOW = data[6] | data[7]<<8 | data[8]<<16 | data[9]<<24 ;
	posllh->lon = data[10] | data[11]<<8 | data[12]<<16 | data[13]<<24 ;
	posllh->lat = data[14] | data[15]<<8 | data[16]<<16 | data[17]<<24 ;
	posllh->height = data[18] | data[19]<<8 | data[20]<<16 | data[21]<<24 ;
	posllh->hMSL = data[22] | data[23]<<8 | data[24]<<16 | data[25]<<24 ;
	posllh->hAcc = data[26] | data[27]<<8 | data[28]<<16 | data[29]<<24 ;
	posllh->vAcc = data[30] | data[31]<<8 | data[32]<<16 | data[33]<<24 ;

//	posllh->lon_f64 = posllh->lon/ 10000000.;
//	posllh->lat_f64 = posllh->lat/ 10000000.;

}


void M8N_UBX_NAV_PVT_Parsing(unsigned char* data, M8N_UBX_NAV_PVT* pvt)
{
	pvt->CLASS = data[2];
	pvt->ID = data[3];
	pvt->length = data[4] | data [5]<<8;

	pvt->iTOW = data[6] | data[7]<<8 | data[8]<<16 | data[9]<<24 ;
	pvt->year =  data[10] | data[11]<<8 ;
	pvt->month = data[12] ;
	pvt->day = data[13] ;
	pvt->hour = data[14] ;
	pvt->min = data[15] ;
	pvt->sec = data[16] ;
	pvt->valid = data[17] ;
	pvt->tAcc = data[18] | data[19]<<8 | data[20]<<16 | data[21]<<24 ;
	pvt->nano = data[22] | data[23]<<8 | data[24]<<16 | data[25]<<24 ;
	pvt->fixType = data[26] ;
	pvt->flags = data[27] ;
	pvt->flags2 = data[28] ;
	pvt->numSV = data[29] ;
	pvt->lon = data[30] | data[31]<<8 | data[32]<<16 | data[33]<<24 ;
	pvt->lat = data[34] | data[35]<<8 | data[36]<<16 | data[37]<<24 ;
	pvt->height = data[38] | data[39]<<8 | data[40]<<16 | data[41]<<24 ;
	pvt->hMSL = data[42] | data[43]<<8 | data[44]<<16 | data[45]<<24 ;
	pvt->hAcc = data[46] | data[47]<<8 | data[48]<<16 | data[49]<<24 ;
	pvt->vAcc = data[50] | data[51]<<8 | data[52]<<16 | data[53]<<24 ;
	pvt->velN = data[54] | data[55]<<8 | data[56]<<16 | data[57]<<24 ;
	pvt->velE = data[58] | data[59]<<8 | data[60]<<16 | data[61]<<24 ;
	pvt->velD = data[62] | data[63]<<8 | data[64]<<16 | data[65]<<24 ;
	pvt->gSpeed = data[66] | data[67]<<8 | data[68]<<16 | data[69]<<24 ;
	pvt->headMot = data[70] | data[71]<<8 | data[72]<<16 | data[73]<<24 ;
	pvt->sAcc = data[74] | data[75]<<8 | data[76]<<16 | data[77]<<24 ;
	pvt->headAcc = data[78] | data[79]<<8 | data[80]<<16 | data[81]<<24 ;
	pvt->pDOP = data[82] | data[83]<<8 ;
	pvt->flags3 = data[84] | data[85]<<8 ;
	pvt->reserved1 = data[86] | data[87]<<8 | data[88]<<16 | data[89]<<24 ;
	pvt->headVeh = data[90] | data[91]<<8 | data[92]<<16 | data[93]<<24 ;
	pvt->magDec = data[94] | data[95]<<8 ;
	pvt->magAcc = data[96] | data[97]<<8 ;
}
