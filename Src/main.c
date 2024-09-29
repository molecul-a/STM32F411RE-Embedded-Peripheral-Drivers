#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "adc.h"
#include "uart.h"
#include "systick.h"
#include "tim.h"
#include "exti.h"
#include "lcd1602.h"



int main(void)
{

	lcd_init ();
	lcd_put_cur(0, 0);
	lcd_send_string ("HELLO WORLD");
	lcd_put_cur(1, 0);
	lcd_send_string("BLEAT!!!");

	while(1)
	{

	}
}

