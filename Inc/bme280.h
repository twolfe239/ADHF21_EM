#include "stm32f1xx_hal.h"
#include <math.h>



//------------------------------------------------
#define	BME280_ADDRESS	0xEC

//------------------------------------------------
#define	BME280_id	0xD0
#define	BME280_hum_lsb	0xFE
#define	BME280_hum_msb	0xFD
#define	BME280_temp_xlsb	0xFC
#define	BME280_temp_lsb	0xFB
#define	BME280_temp_msb	0xFA
#define	BME280_press_xlsb	0xF9
#define	BME280_press_lsb	0xF8
#define	BME280_press_msb	0xF7
#define	BME280_config	0xF5
#define	BME280_ctrl_meas	0xF4
#define	BME280_ctrl_hum	0xF2
#define	BME280_status	0xF3
#define	BME280_reset	0xE0
//------------------------------------------------
#define	BME280_id_val	0x60
#define	BMP280_id_val	0x58
//------------------------------------------------


#define BME280_DIG_H4_MSB_REG			0xE4
#define BME280_DIG_H4_LSB_REG			0xE5
#define BME280_DIG_H5_MSB_REG	0xE6


#define	BME280_REG_CONTROL 0xF4
#define	BME280_REG_CONFIG 0xF5
#define	BME280_REG_HUM 0xF2
#define	BME280_REG_RESULT_PRESSURE 0xF7			// 0xF7(msb) , 0xF8(lsb) , 0xF9(xlsb) : stores the pressure data.
#define BME280_REG_RESULT_TEMPRERATURE 0xFA		// 0xFA(msb) , 0xFB(lsb) , 0xFC(xlsb) : stores the temperature data.
#define BME280_REG_RESULT_HUMIDITY 0xFD		// 0xFD(msb) , 0xFE(lsb) : stores the hum data.

#define	BME280_OVERSAMPLING_T1		0x20
#define	BME280_OVERSAMPLING_T2		0x40
#define	BME280_OVERSAMPLING_T4		0x60
#define	BME280_OVERSAMPLING_T8		0x80
#define	BME280_OVERSAMPLING_T16		0xA0



#define	BME280_OVERSAMPLING_HS		0x00
#define	BME280_OVERSAMPLING_H1		0x01
#define	BME280_OVERSAMPLING_H2		0x02
#define	BME280_OVERSAMPLING_H4		0x03
#define	BME280_OVERSAMPLING_H8		0x04
#define	BME280_OVERSAMPLING_H16		0x05

#define	BME280_OVERSAMPLING_P1		0x04
#define	BME280_OVERSAMPLING_P2		0x08
#define	BME280_OVERSAMPLING_P4		0x0C
#define	BME280_OVERSAMPLING_P8		0x10
#define	BME280_OVERSAMPLING_P16		0x14

#define	BME280_MODE_SLEEP			0x00
#define	BME280_MODE_FORCED			0x01
#define	BME280_MODE_NORMAL			0x03

#define	BME280_TSB_0_5				0x00
#define	BME280_TSB_62_5				0x20
#define	BME280_TSB_125				0x40
#define	BME280_TSB_250				0x60
#define	BME280_TSB_500				0x80
#define	BME280_TSB_1000				0xA0
#define	BME280_TSB_10				0xC0
#define	BME280_TSB_20				0xE0

#define	BME280_FILTER_OFF			0x00
#define	BME280_FILTER_COEFFICIENT2	0x04
#define	BME280_FILTER_COEFFICIENT4	0x08
#define	BME280_FILTER_COEFFICIENT8	0x0C
#define	BME280_FILTER_COEFFICIENT16	0x10

#define	BME280_SPI_OFF	0x00
#define	BME280_SPI_ON	0x01
#define	BME280_HUM			(BME280_OVERSAMPLING_H16)
#define	BME280_MEAS			(BME280_OVERSAMPLING_T16 | BME280_OVERSAMPLING_P16 | BME280_MODE_NORMAL)
#define	BME280_CONFIG		(BME280_TSB_0_5 | BME280_FILTER_COEFFICIENT16 | BME280_SPI_OFF)
uint8_t  BME280ReadByte(uint8_t address);
uint16_t BME280ReadShort(uint8_t address);
uint32_t BME280ReadLong(uint8_t address);


void BME280WriteByte(uint8_t address, uint8_t data);
int32_t BME280ReadTemp(void);
int32_t BME280ReadPressure(void);
int32_t BME280ReadHumidity(void);
int32_t BME280ConvertT(void);
double BME280ConvertH(int32_t t_fine);
int32_t BME280ConvertP(int32_t t_fine);
void BME280_Begin(void);
void BME280_Read(void);
int16_t  BME280ReadCali(uint8_t address);
