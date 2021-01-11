#include "stm32f1xx_hal.h"
#include "mems.h"
#include "i2c.h"
#include <math.h>
#include "ssd1306.h"
uint8_t CC = 0;
char buf1[10];
char buf2[10];
volatile	float temper,hum;






  void hts221WriteByte(uint8_t address, uint8_t data) {
	uint8_t dd[1];
	dd[0] = data;
	HAL_I2C_Mem_Write(&hi2c1, HTS221_ADDRESS, address, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) dd, 1, 1000);
}


uint8_t hts221ReadByte(uint8_t address) {
	int8_t data;

	uint8_t rmsg[1];
	HAL_I2C_Mem_Read(&hi2c1, HTS221_ADDRESS, address, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) rmsg, 1, 1000);
	data = rmsg[0];
	return data;
}


void hts221_ini(void) {
		hts221WriteByte(HTS221_CTRL_REG1,	0x00); 
	uint8_t rmsg[1];
	HAL_I2C_Mem_Read(&hi2c1, HTS221_ADDRESS, HTS221_WHO_AM_I_REG,
			I2C_MEMADD_SIZE_8BIT, (uint8_t*) rmsg, 1, 1000);

	switch (rmsg[0]) {
	case HTS221_WHO_AM_I_VAL:
		CC = 1;
		ssd1306_SetCursor(92, 10);
		ssd1306_WriteString("PASS", Font_7x10);
		ssd1306_UpdateScreen();
		hts221WriteByte(HTS221_RES_CONF_REG, 0x3F);
		hts221WriteByte(HTS221_CTRL_REG1,	0x87); 
		hts221WriteByte(HTS221_CTRL_REG2, 0x00);
		hts221WriteByte(HTS221_CTRL_REG3, 0x00);
		HAL_Delay(800);
		break;

	default:
		CC = 0;
		ssd1306_SetCursor(92, 10);
		ssd1306_WriteString("FAIL", Font_7x10);
		ssd1306_UpdateScreen();
		HAL_Delay(800);
		break;
	}


}

void hts221_Get_Temp(float *pData) {
	int16_t T0_degC, T1_degC;
	int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
	uint8_t buffer[4], tmp;
	buffer[0] = hts221ReadByte(HTS221_T0_DEGC_X8);
	buffer[1] = hts221ReadByte(HTS221_T1_DEGC_X8);
	tmp = hts221ReadByte(HTS221_T0_T1_DEGC_H2);
	T0_degC_x8_u16 = (((uint16_t) (tmp & 0x03)) << 8) | ((uint16_t) buffer[0]);
	T1_degC_x8_u16 = (((uint16_t) (tmp & 0x0C)) << 6) | ((uint16_t) buffer[1]);
	T0_degC = T0_degC_x8_u16 >> 3;
	T1_degC = T1_degC_x8_u16 >> 3;
	buffer[0] = hts221ReadByte(HTS221_T0_OUT_L);
	buffer[1] = hts221ReadByte(HTS221_T0_OUT_H);
	buffer[2] = hts221ReadByte(HTS221_T1_OUT_L);
	buffer[3] = hts221ReadByte(HTS221_T1_OUT_H);
	T0_out = (((uint16_t) buffer[1]) << 8) | ((uint16_t) buffer[0]);
	T1_out = (((uint16_t) buffer[3]) << 8) | ((uint16_t) buffer[2]);
	buffer[0] = hts221ReadByte(HTS221_TEMP_OUT_L_REG);
	buffer[1] = hts221ReadByte(HTS221_TEMP_OUT_H_REG);
	T_out = (((uint16_t) buffer[1]) << 8) | ((uint16_t) buffer[0]);
	T_out = (((uint16_t) buffer[1] << 8) + (uint16_t) buffer[0]);
		*pData = (float)(T_out - T0_out) * (float)(T1_degC - T0_degC) /\
		(float)(T1_out - T0_out) + T0_degC;
		
}

void hts221_Get_Hum(float *pData) {
	uint8_t buffer[2];
	int16_t H0_T0_out, H1_T0_out, H_T_out;
	int16_t H0_rh, H1_rh;
	float tmp_f;
	buffer[0] = hts221ReadByte(HTS221_H0_RH_X2);
	buffer[1] = hts221ReadByte(HTS221_H1_RH_X2);
	H0_rh = buffer[0] >> 1;
	H1_rh = buffer[1] >> 1;
	buffer[0] = hts221ReadByte(HTS221_H0_T0_OUT_L);
	buffer[1] = hts221ReadByte(HTS221_H0_T0_OUT_H);
	H0_T0_out = (((uint16_t) buffer[1]) << 8) | ((uint16_t) buffer[0]);
	buffer[0] = hts221ReadByte(HTS221_H1_T0_OUT_L);
	buffer[1] = hts221ReadByte(HTS221_H1_T0_OUT_H);
	H1_T0_out = (((uint16_t) buffer[1]) << 8) | ((uint16_t) buffer[0]);
	buffer[0] = hts221ReadByte(HTS221_HR_OUT_L_REG);
	buffer[1] = hts221ReadByte(HTS221_HR_OUT_H_REG);
	H_T_out = (((uint16_t) buffer[1]) << 8) | ((uint16_t) buffer[0]);

tmp_f = (float)(H_T_out - H0_T0_out) * (float)(H1_rh - H0_rh) /\
	(float)(H1_T0_out - H0_T0_out) + H0_rh;
	*pData = (tmp_f>100.0f)?100.0f
		:(tmp_f<0.0f)?0.0f
		:tmp_f;
}







void hts221_Read(void) {
	
if(CC  == 1) {
	
	

	hts221_Get_Temp(&temper);
	hts221_Get_Hum(&hum);

	
			ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("ALARM BUTTON PRJ", Font_7x10);
	
		sprintf(buf1, "%.02f'C", temper);
		ssd1306_SetCursor(0, 10);
		ssd1306_WriteString((char*) buf1, Font_7x10);


		sprintf(buf2, "%.02f%%", hum);
		ssd1306_SetCursor(0, 20);
		ssd1306_WriteString((char*) buf2, Font_7x10);

	
 }
else {
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("ALARM BUTTON PRJ", Font_7x10);
			ssd1306_SetCursor(0, 10);
		ssd1306_WriteString("HW V1.1", Font_7x10);
			ssd1306_SetCursor(0, 20);
		ssd1306_WriteString("SW V.84", Font_7x10);

}


}

