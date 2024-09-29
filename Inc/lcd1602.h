/*
 * lcd1602.h
 *
 *  Created on: Sep 28, 2024
 *      Author: artyom
 */


#ifndef LCD1602_H_
#define LCD1602_H_

#include "stm32f4xx.h"
#include "i2c.h"

void lcd_read_address(uint8_t reg);
void check_lcd_init(void);
#define SLAVE_ADDRESS_LCD 0x27


#endif /* LCD1602_H_ */
