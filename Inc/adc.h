/*
 * adc.h
 *
 *  Created on: Sep 7, 2024
 *      Author: artyom
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

#define SR_EOC		(1U<<1)

void pa1_adc_init(void);
void pa1_adc_interrupt_init(void);
uint32_t adc_read(void);
void start_conversion(void);



#endif /* ADC_H_ */
