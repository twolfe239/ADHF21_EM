#include "i2c_lcd.h"
#include "i2c.h"
uint8_t backlightState = 1;


void lcd_Goto(uint8_t row, uint8_t col) {
	switch (row){
		case 0:
			lcd_Command(0x80 + col);
			break;
		case 1:
			lcd_Command(0x80 + col + 0x40);
			break;
		case 2:
			lcd_Command(0x80 + col + 0x14);
			break;
		case 3:
			lcd_Command(0x80 + col + 0x54);
			break;
	}
}

void lcd_PrintC(const uint8_t *str) {
 	uint8_t i;
 	while (i = *str++){
    	lcd_Data(i);
 	}
}

void lcd_Init(void) {
	lcd_Command(0x33);
	lcd_pause;
	lcd_Command(0x32);
	lcd_Command(0x28);
	lcd_Command(0x08);
	lcd_Command(0x01);
	lcd_pause;
	lcd_Command(0x06);
	lcd_Command(0x0C);
	
	
	
lcd_Goto(1,0);
lcd_PrintC("LCD INIT OK");
HAL_Delay(500);
}
void lcd_Clear(void){
		lcd_Command(0x01);
}

void lcd_LClear(void){
lcd_Goto(0, 0);	
lcd_PrintC("                ");
lcd_Goto(1, 0);	
lcd_PrintC("                ");
}




void lcd_Send(uint8_t data) {



	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &data, 1, 0x1000);





/*

	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, ((LCD_ADDR) << 1), I2C_Direction_Transmitter); //38- 4 20 - 2
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1, data);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTOP(I2C1, ENABLE);*/

}






void lcd_Command(uint8_t com) {
	uint8_t data = 0;

	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x10) >> 4) << DB4);
	data |= (((com & 0x20) >> 5) << DB5);
	data |= (((com & 0x40) >> 6) << DB6);
	data |= (((com & 0x80) >> 7) << DB7);
	lcd_Send(data);

	data |= (1 << EN);
	lcd_Send(data);
	lcd_pause;

	data &= ~(1 << EN);
	lcd_Send(data);
	lcd_pause;

	data = 0;

	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x01) >> 0) << DB4);
	data |= (((com & 0x02) >> 1) << DB5);
	data |= (((com & 0x04) >> 2) << DB6);
	data |= (((com & 0x08) >> 3) << DB7);
	lcd_Send(data);

	data |= (1 << EN);
	lcd_Send(data);
	lcd_pause;

	data &= ~(1 << EN);
	lcd_Send(data);
	lcd_pause;
}
/*
void lcd_Backlight(uint8_t state) {
	backlightState = (state & 0x01) << BL;
	lcd_Send(backlightState);
}*/
/////////////////////////////////////////////////////////////////////////////
void lcd_BL(uint8_t state) {
	backlightState = state;
	lcd_Send((backlightState & 0x01) << BL);
}

void lcd_Data(uint8_t com) {
	uint8_t data = 0;

	data |= (1 << EN);
	data |= (1 << RS);
	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x10) >> 4) << DB4);
	data |= (((com & 0x20) >> 5) << DB5);
	data |= (((com & 0x40) >> 6) << DB6);
	data |= (((com & 0x80) >> 7) << DB7);
	lcd_Send(data);
	lcd_pause;

	data &= ~(1 << EN);
	lcd_Send(data);
	lcd_pause;

	data = 0;

	data |= (1 << EN);
	data |= (1 << RS);
	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x01) >> 0) << DB4);
	data |= (((com & 0x02) >> 1) << DB5);
	data |= (((com & 0x04) >> 2) << DB6);
	data |= (((com & 0x08) >> 3) << DB7);
	lcd_Send(data);
	lcd_pause;

	data &= ~(1 << EN);
	lcd_Send(data);
	lcd_pause;
}


