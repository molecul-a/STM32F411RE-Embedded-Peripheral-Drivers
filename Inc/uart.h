/*
 * uart.h
 *
 *  Created on: Sep 5, 2024
 *      Author: artyom
 */
#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include "stm32f4xx.h"

char uart2_read(void);
void uart2_rxtx_init(void);
void uart2_tx_init(void);
void uart2_rxtx_interrupt_init(void);
void dma1_stream6_init(uint32_t scr, uint32_t dst, uint32_t len);

#define SR_RXNE			(1U<<5)
#define DMA_HISR_TCIF6	(1U<<21)
#define DMA_HIFCR_CTCIF6 (1U<<21)
#endif /* UART_H_ */
