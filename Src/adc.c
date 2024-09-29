/*
 * adc.c
 *
 *  Created on: Sep 7, 2024
 *      Author: artyom
 */
#include "stm32f4xx.h"
#include "adc.h"

#define GPIOAEN			(1U<<0)
#define ADC1EN			(1U<<8)
#define ADC_CH1			(1U<<0)
#define ADC_SEQ_LEN_1   (0x00)
#define ADC_CR2_ADON	(1U<<0)
#define ADC_CR2_SWSTART	(1U<<30)
#define SR_EOC			(1U<<1)
#define ADC_CR1_EOCIE	(1U<<5)

void pa1_adc_init(void)
{
	//*******Configure the ADC GPIO pin*******
	//Enable clock access to GPIOA
	RCC->AHB1ENR |= GPIOAEN;
	//Set the mode of PA1 to analog
	GPIOA->MODER |= (1U<<2);
	GPIOA->MODER |= (1U<<3);
	//*******Configure the ADC module*********
	//Enable clock access to ADC
	RCC->APB2ENR |= ADC1EN;
	//Conversion sequence start
	ADC1->SQR3 = ADC_CH1;
	//Conversion sequence length
	ADC1->SQR1 = ADC_SEQ_LEN_1;
	//Enable ADC module
	ADC1->CR2 |= ADC_CR2_ADON;
}

void pa1_adc_interrupt_init(void)
{
	//*******Configure the ADC GPIO pin*******
	//Enable clock access to GPIOA
	RCC->AHB1ENR |= GPIOAEN;
	//Set the mode of PA1 to analog
	GPIOA->MODER |= (1U<<2);
	GPIOA->MODER |= (1U<<3);
	//*******Configure the ADC module*********
	//Enable clock access to ADC
	RCC->APB2ENR |= ADC1EN;
	//Enable EOC to ADC1 in CR1
	ADC1->CR1 |= ADC_CR1_EOCIE;
	//Enable ADC1 interruption in NVIC
	NVIC_EnableIRQ(ADC_IRQn);
	//Conversion sequence start
	ADC1->SQR3 = ADC_CH1;
	//Conversion sequence length
	ADC1->SQR1 = ADC_SEQ_LEN_1;
	//Enable ADC module
	ADC1->CR2 |= ADC_CR2_ADON;
}

void start_conversion(void)
{
	//Enable continuous conversion
	ADC1->CR2 |= ADC_CR2_CONT;
	//Start ADC conversion
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

uint32_t adc_read(void)
{
	//Wait for conversion to be complete
	while(!(ADC1->SR & SR_EOC)){}
	//Read converted result
	return (ADC1->DR);
}




