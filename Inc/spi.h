/*
 * spi.h
 *
 *  Created on: Sep 29, 2024
 *      Author: artyom
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f4xx.h"

void spi_qpio_init(void);
void spi1_cofig(void);
void spi1_transmit(uint8_t *data, uint32_t size);
void spi1_receive(uint8_t *data, uint32_t size);
void cs_enable(void);
void cs_disable(void);

#endif /* SPI_H_ */
