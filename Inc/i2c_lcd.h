#ifndef _I2C_LCD_H_
#define _I2C_LCD_H_
#include "main.h"


#define LCD_ADDR 0x7F   // 2004

#define lcd_pause	HAL_Delay(1)


#define PCF_P0	0
#define PCF_P1	1
#define PCF_P2	2
#define PCF_P3	3
#define PCF_P4	4
#define PCF_P5	5
#define PCF_P6	6
#define PCF_P7	7

#define DB4		PCF_P4
#define DB5		PCF_P5
#define DB6		PCF_P6
#define DB7		PCF_P7
#define EN		PCF_P2
#define RW		PCF_P1
#define RS		PCF_P0
#define BL		PCF_P3




void lcd_Init(void); 
void lcd_Clear(void); 
void lcd_LClear(void); 
/*void lcd_Backlight(uint8_t state);*/
void lcd_BL(uint8_t state);
void lcd_Goto(uint8_t row, uint8_t col); 
void lcd_PrintC(const uint8_t *str);


void lcd_Send(uint8_t data);
void lcd_Command(uint8_t com);
void lcd_Data(uint8_t com);



#endif
