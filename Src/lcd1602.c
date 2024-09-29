/*
 * lcd1602.c
 *
 *  Created on: Sep 28, 2024
 *      Author: artyom
 */
#include "lcd1602.h"


void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
  data_u = (cmd&0xf0);
  data_l = ((cmd<<8)&0xf0);

  uint8_t data_t[4];
  data_t[0] = data_u|0x0C;  //en=1, rs=0 -> bxxxx1100
  data_t[1] = data_u|0x08;  //en=0, rs=0 -> bxxxx1000

  data_t[2] = data_l|0x0C;  //en=1, rs=0 -> bxxxx1100
  data_t[3] = data_l|0x08;  //en=0, rs=0 -> bxxxx1000

  I2C1_burstWrite(SLAVE_ADDRESS_LCD,0,4,(uint8_t *) data_t);
  //HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1 -> bxxxx1101
	data_t[1] = data_u|0x09;  //en=0, rs=1 -> bxxxx1001
	data_t[2] = data_l|0x0D;  //en=1, rs=1 -> bxxxx1101
	data_t[3] = data_l|0x09;  //en=0, rs=1 -> bxxxx1001
	I2C1_burstWrite(SLAVE_ADDRESS_LCD,0,4,(uint8_t *) data_t);
	//HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_init (void)
	{
	I2C1_init();
	// 4 bit initialization
	systickDelayMs(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	systickDelayMs(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	systickDelayMs(1);  // wait for >100us
	lcd_send_cmd (0x30);
	systickDelayMs(10);
	lcd_send_cmd (0x20);  // 4bit mode
	systickDelayMs(10);

	// display initialization
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	systickDelayMs(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	systickDelayMs(1);
	lcd_send_cmd (0x01);  // clear display
	systickDelayMs(2);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	systickDelayMs(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}
void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_cmd (col);
}



