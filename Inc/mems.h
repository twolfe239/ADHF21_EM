#ifndef __MEMS_H
#define __MEMS_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/









//------------------------------------------------
#define	HTS221_ADDRESS	0xBE
//------------------------------------------------
#define	HTS221_WHO_AM_I_REG	0x0F
#define HTS221_RES_CONF_REG 0x10
#define	HTS221_CTRL_REG1	0x20
#define	HTS221_CTRL_REG2	0x21
#define	HTS221_CTRL_REG3	0x22
//------------------------------------------------
#define	HTS221_WHO_AM_I_VAL	0xBC
//------------------------------------------------
#define HTS221_HR_OUT_L_REG	0x28
#define HTS221_HR_OUT_H_REG	0x29
#define	HTS221_H0_RH_X2	0x30
#define	HTS221_H1_RH_X2	0x31
#define	HTS221_T0_DEGC_X8	0x32
#define	HTS221_T1_DEGC_X8	0x33
#define	HTS221_T0_T1_DEGC_H2	0x35
#define	HTS221_H0_T0_OUT_L	0x36
#define	HTS221_H0_T0_OUT_H	0x37
#define	HTS221_H1_T0_OUT_L	0x3A
#define	HTS221_H1_T0_OUT_H	0x3B
#define	HTS221_T0_OUT_L	0x3C
#define	HTS221_T0_OUT_H	0x3D
#define	HTS221_T1_OUT_L	0x3E
#define	HTS221_T1_OUT_H	0x3F
#define HTS221_TEMP_OUT_L_REG	0x2A
#define HTS221_TEMP_OUT_H_REG	0x2B





/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

uint8_t  hts221ReadByte(uint8_t address);
void hts221WriteByte(uint8_t address, uint8_t data);
	void hts221_ini(void);
	void hts221_Get_Temp(float* pData);
	void hts221_Get_Hum(float* pData);
	void hts221_Read(void);

#endif
