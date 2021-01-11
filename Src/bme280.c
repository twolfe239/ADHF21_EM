#include "stm32f1xx_hal.h"
#include "BME280.h"
#include "i2c.h"
#include <math.h>
#include "ssd1306.h"

uint16_t BMx280 = 0;
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;

uint8_t firstON1 = 0;
double altoffset1 = 0;
double paoffset1 = 0;
uint8_t dig_H1;
int16_t dig_H2;
uint8_t dig_H3;
int16_t dig_H4;
int16_t dig_H5;
uint8_t dig_H6;

int16_t BME280ReadCali(uint8_t address) {

	int16_t data;

	uint8_t rmsg[2];
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDRESS, address, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) rmsg, 2, 1000);

	data = (rmsg[1] << 8) + rmsg[0];
	return data;
}
uint8_t BME280ReadByte(uint8_t address) {
	int8_t data;

	uint8_t rmsg[1];
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDRESS, address, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) rmsg, 1, 1000);
	data = rmsg[0];
	return data;
}

uint16_t BME280ReadShort(uint8_t address) {
	int16_t data;

	uint8_t rmsg[2];
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDRESS, address, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) rmsg, 2, 1000);
	data = (rmsg[0] << 8) + rmsg[1];
	return data;
}
uint32_t BME280ReadLong(uint8_t address) {
	uint32_t data;
	uint8_t rmsg[3];
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDRESS, address, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) rmsg, 3, 1000);
	data = ((rmsg[0] << 16) + (rmsg[1] << 8) + rmsg[2]) >> 4;
	return data;
}

//----------------------------------------

void BME280WriteByte(uint8_t address, uint8_t data) {
	uint8_t dd[1];
	dd[0] = data;
	HAL_I2C_Mem_Write(&hi2c1, BME280_ADDRESS, address, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) dd, 1, 1000);
}
//----------------------------------------

int32_t BME280ReadTemp(void) {
	int32_t temperature = 0;

	temperature = BME280ReadLong(BME280_REG_RESULT_TEMPRERATURE);

	return temperature;
}
//----------------------------------------

int32_t BME280ReadPressure(void) {
	int32_t pressure = 0;

	pressure = BME280ReadLong(BME280_REG_RESULT_PRESSURE);

	return pressure;
}

int32_t BME280ReadHumidity(void) {
	int32_t humidity = 0;

	humidity = BME280ReadShort(BME280_REG_RESULT_HUMIDITY);

	return humidity;
}

int32_t BME280ConvertT(void) {
	int32_t adc_T;
	int32_t temperature;
	adc_T = BME280ReadTemp();

	int32_t var1, var2, t_fine;
	var1 = (((double) adc_T) / 16384.0 - ((double) dig_T1) / 1024.0)
			* ((double) dig_T2);
	var2 = ((((double) adc_T) / 131072.0 - ((double) dig_T1) / 8192.0)
			* (((double) adc_T) / 131072.0 - ((double) dig_T1) / 8192.0))
			* ((double) dig_T3);
	t_fine = (var1 + var2);

	return t_fine;
}

double BME280ConvertH(int32_t t_fine) {
	int32_t adc_H;
	adc_H = BME280ReadHumidity();
	double var_H;
	var_H = (((double) t_fine) - 76800.0);

	var_H = (adc_H
			- (((double) dig_H4) * 64.0 + ((double) dig_H5) / 16384.0 * var_H))
			* (((double) dig_H2) / 65536.0
					* (1.0
							+ ((double) dig_H6) / 67108864.0 * var_H
									* (1.0
											+ ((double) dig_H3) / 67108864.0
													* var_H)));

	var_H = var_H * (1.0 - ((double) dig_H1) * var_H / 524288.0);

	if (var_H > 100.0)
		var_H = 100.0;
	else if (var_H < 0.0)
		var_H = 0.0;

	return var_H;

}

int32_t BME280ConvertP(int32_t t_fine) {

	int32_t adc_P;
	adc_P = BME280ReadPressure();
	int32_t var1, var2, p;

	var1 = ((double) t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double) dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double) dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double) dig_P4) * 65536.0);
	var1 = (((double) dig_P3) * var1 * var1 / 524288.0
			+ ((double) dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double) dig_P1);
//	if (var1 == 0.0)	{		return 0; }
	p = 1048576.0 - (double) adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double) dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double) dig_P8) / 32768.0;
	p = (p + (var1 + var2 + ((double) dig_P7)) / 16.0);
	return p;
}

void BME280_Begin(void) {

	uint8_t rmsg[1];
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDRESS, BME280_id, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) rmsg, 1, 1000);

	switch (rmsg[0]) {
	case BME280_id_val:
		BMx280 = 1;
		ssd1306_SetCursor(92, 10);
		ssd1306_WriteString("PASSE", Font_7x10);
		ssd1306_UpdateScreen();
		HAL_Delay(800);
		break;
	case BMP280_id_val:
		BMx280 = 0;
		ssd1306_SetCursor(92, 10);
		ssd1306_WriteString("PASSP", Font_7x10);
		ssd1306_UpdateScreen();
		HAL_Delay(800);
		break;

	default:
		BMx280 = 13;
		ssd1306_SetCursor(92, 10);
		ssd1306_WriteString("FAIL", Font_7x10);
		ssd1306_UpdateScreen();
		HAL_Delay(800);
		break;
	}

	if (BMx280 < 3) {

		dig_T1 = BME280ReadCali(0x88); //dig_T1
		dig_T2 = BME280ReadCali(0x8A); //dig_T2
		dig_T3 = BME280ReadCali(0x8C); //dig_T3
		dig_P1 = BME280ReadCali(0x8E); //dig_P1
		dig_P2 = BME280ReadCali(0x90); //dig_P2
		dig_P3 = BME280ReadCali(0x92); //dig_P3	
		dig_P4 = BME280ReadCali(0x94); //dig_P4
		dig_P5 = BME280ReadCali(0x96); //dig_P5
		dig_P6 = BME280ReadCali(0x98); //dig_P6	
		dig_P7 = BME280ReadCali(0x9A); //dig_P7
		dig_P8 = BME280ReadCali(0x9C); //dig_P8
		dig_P9 = BME280ReadCali(0x9E); //dig_P9

		if (BMx280 == 1) {
			dig_H1 = BME280ReadByte(0xA1);
			dig_H2 = BME280ReadCali(0xE1);
			dig_H3 = BME280ReadByte(0xE3);
			dig_H6 = BME280ReadByte(0xE7);
			;
			dig_H4 = ((int16_t) ((BME280ReadByte(BME280_DIG_H4_MSB_REG) << 4)
					+ (BME280ReadByte(BME280_DIG_H4_LSB_REG) & 0x0F)));
			dig_H5 = ((int16_t) ((BME280ReadByte(BME280_DIG_H5_MSB_REG) << 4)
					+ ((BME280ReadByte(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
			BME280WriteByte(BME280_REG_HUM, BME280_HUM);
		}
		BME280WriteByte(BME280_REG_CONFIG, BME280_CONFIG);
		BME280WriteByte(BME280_REG_CONTROL, BME280_MEAS);
	}
}

void BME280_Read(void) {
	if (BMx280 < 3) {
		char bufbme[20];

		int32_t var1, var2;
		double temperature, t_fine;
		int32_t pressure;
		double humidity;
		int32_t altitude;
		int32_t pressure10 = 0;
		double temperature10 = 0;
		double t_fine10 = 0;
		double patm, pmmhg, altft, altm, phpa;

		t_fine = BME280ConvertT();
		temperature = (t_fine / 51.2);
		pressure = BME280ConvertP(t_fine);

		var1 = temperature / 100;
		var2 = temperature - var1 * 100;



		pmmhg = pressure / 133.333f;
		patm = pressure / 101325.0f;
		phpa = pressure / 100.0f;

		if (BMx280 == 0) {
			double difpa, difmmhg;
			if (firstON1 == 0) {

				paoffset1 = pressure;
				firstON1 = 1;

			}
			difpa = pressure - paoffset1;
			difmmhg = difpa / 133.333f;

			if (difmmhg > 150) {

				paoffset1 = pressure;

			}

			/*
			 sprintf(bufbme,"D:%03.02f(mm Hg)",difmmhg);
			 ssd1306_SetCursor(0, 20);  
			 ssd1306_WriteString((char *)bufbme, Font_7x10);
			 */
		}

		
		sprintf(bufbme, "%02d.%01d'C", var1, var2);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString((char*) bufbme, Font_7x10);
		
		
		if (BMx280 == 1) {

			sprintf(bufbme, "%03.01fmm", pmmhg);
			ssd1306_SetCursor(0, 10);
			ssd1306_WriteString((char*) bufbme, Font_7x10);

			humidity = BME280ConvertH(t_fine);
			sprintf(bufbme, "%02.01f%%", humidity);
			ssd1306_SetCursor(0, 20);
			ssd1306_WriteString((char*) bufbme, Font_7x10);

		} else {

			sprintf(bufbme, "%03.01f", pmmhg);
			ssd1306_SetCursor(0, 10);
			ssd1306_WriteString((char*) bufbme, Font_7x10);

		}

	} else {
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("ALARM BUTTON PRJ", Font_7x10);
			ssd1306_SetCursor(0, 10);
		ssd1306_WriteString("HW V1.1", Font_7x10);
			ssd1306_SetCursor(0, 20);
		ssd1306_WriteString("SW V.84", Font_7x10);

}
}

