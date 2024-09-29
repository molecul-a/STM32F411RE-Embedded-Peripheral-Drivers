/*
 * Systick.c
 *
 *  Created on: Sep 8, 2024
 *      Author: artyom
 */
#include "stm32f4xx.h"

#define SYSTICK_LOAD_VALUE		16000
#define SYSTICK_CTRL_ENABLE		(1U<<0)
#define SYSTICK_CTRL_CLKSRC		(1U<<2)
#define SYSTICK_CTRL_COUNTFLAG	(1U<<16)
#define SYSTICK_CTRL_TICKINT	(1U<<1)
#define ONE_SEC_LOAD			16000000

void systickDelayMs(int delay)
{
	//Configure SysTick
	//Reload with number of clocks per millisecond
	SysTick->LOAD	= SYSTICK_LOAD_VALUE;
	//Clear SysTick current value register
	SysTick->VAL = 0;
	//Enable SysTick and select internal clock source
	SysTick->CTRL = SYSTICK_CTRL_ENABLE | SYSTICK_CTRL_CLKSRC;

	for(int i = 0; i<delay; i++)
	{
		//Wait until the count flag is set
		while((SysTick->CTRL & SYSTICK_CTRL_COUNTFLAG) == 0){}
	}
	SysTick->CTRL = 0;
}

void systick_1hz_interrupt(void)
{
	//Reload with number of clocks per millisecond
	SysTick->LOAD	= ONE_SEC_LOAD - 1;
	//Clear SysTick current value register
	SysTick->VAL = 0;
	//Enable SysTick and select internal clock source
	SysTick->CTRL = SYSTICK_CTRL_ENABLE | SYSTICK_CTRL_CLKSRC;
	//Enable SysTick interrupt
	SysTick->CTRL |= SYSTICK_CTRL_TICKINT;
}
