/*
 * i2c.c
 *
 *  Created on: Sep 28, 2024
 *      Author: artyom
 */
#include "i2c.h"
//PB8 as SCL
//PB9 AS SDA

#define GPIOBEN					(1U<<1)
#define I2C1EN					(1U<<21) //CLOCK ACCESS

#define I2C_100Khz				80
#define SD_MODE_MAX_RISE_TIME	17
#define I2C_CR1_PE				(1U<<0)

#define SR2_BUSY				(1U<<1)
#define CR1_START				(1U<<8)
#define SR1_SB					(1U<<0)
#define SR1_ADDR				(1U<<1)
#define SR1_TXE					(1U<<7)
#define CR1_ACK					(1U<<10)
#define CR1_STOP				(1U<<9)
#define SR1_RXNE				(1U<<6)
#define SR1_BTF					(1U<<2)

void I2C1_init(void)
{
	//Enable clock to GPIOB
	RCC->AHB1ENR |= GPIOBEN;
	//Set PB8 and PB9 mode to alternate function
	//PB8
	GPIOB->MODER &=~(1U<<16);
	GPIOB->MODER |= (1U<<17);
	//PB9
	GPIOB->MODER &=~(1U<<18);
	GPIOB->MODER |= (1U<<19);
	//Set PB8 and PB9 alternate function type to I2C (AF4)
	//PB8 TO AF4
	GPIOB->AFR[1] &=~(1U<<0);
	GPIOB->AFR[1] &=~(1U<<1);
	GPIOB->AFR[1] |= (1U<<2);
	GPIOB->AFR[1] &=~(1U<<3);
	//PB9 TO AF4
	GPIOB->AFR[1] &=~(1U<<4);
	GPIOB->AFR[1] &=~(1U<<5);
	GPIOB->AFR[1] |= (1U<<6);
	GPIOB->AFR[1] &=~(1U<<7);

	//Set PB8 and PB9 output type to open drain
	//PB8
	GPIOB->OTYPER |= (1U<<8);
	//PB9
	GPIOB->OTYPER |= (1U<<9);
	//Enable Pullup for PB8 and PB9
	//PB8
	GPIOB->PUPDR |= (1U<<16);
	GPIOB->PUPDR &=~ (1U<<17);
	//PB9
	GPIOB->PUPDR |= (1U<<18);
	GPIOB->PUPDR &=~(1U<<19);



	//Enable clock access to I2C
	RCC->APB1ENR |= I2C1EN;
	//Enter reset mode
	I2C1->CR1 |= (1U<<15);
	//Come out of reset mode
	I2C1->CR1 &=~(1U<<15);
	//Set Peripheral clock frequency
	I2C1->CR2 = (1U<<4); // 16MHz
	//Set I2C to standard mode, 100khz clock
	I2C1->CCR = I2C_100Khz;
	//Set rise time
	I2C1->TRISE = SD_MODE_MAX_RISE_TIME;
	//Enable I2C module
	I2C1->CR1 |= I2C_CR1_PE;
}

void I2C1_byteRead(char saddr, char maddr, char* data)
{
	volatile int tmp;
	//Wait until bus not busy
	while(I2C1->SR2 & SR2_BUSY){}
	//Generate start
	I2C1->CR1 |= CR1_START;
	//Wait until start flag is set
	while(!(I2C1->SR1 & SR1_SB)){}
	//Transmit slave address and WRITE
	I2C1->DR = saddr << 1;
	//Wait until addr flag is set
	while(!(I2C1->SR1 & SR1_ADDR)){}
	//Clear addr flag
	tmp = I2C1->SR2;
	//Send memory address
	I2C1->DR = maddr;
	//Wait until transmitter empty
	while(!(I2C1->SR1 & SR1_TXE));
	//Generate restart
	I2C1->CR1 |= CR1_START;
	//Wait until start flag is set
	while(!(I2C1->SR1 & SR1_SB)){}
	//Transmit slave address and READ
	I2C1->DR = saddr << 1 | 1;
	//Wait until addr flag is set
	while (!(I2C1->SR1 & SR1_ADDR)){}
	//Disable Acknowledge
	I2C1->CR1 &=~(CR1_ACK);
	//Clear addr flag
	tmp = I2C1->SR2;
	//Generate stop after data received
	I2C1->CR1 |= CR1_STOP;
	//Wait until RXNE flag is set
	while(!(I2C1->SR1 & SR1_RXNE)){}
	//Read data from DR
	*data++ = I2C1->DR;
}

void I2C1_burstRead(char saddr, char maddr, int n, char* data)
{
	volatile int tmp;
	//Wait until bus not busy
	while(I2C1->SR2 & SR2_BUSY){}
	//Generate start
	I2C1->CR1 |= CR1_START;
	//Wait until start flag is set
	while(!(I2C1->SR1 & SR1_SB)){}
	//Transmit slave address and WRITE
	I2C1->DR = saddr << 1;
	//Wait until addr flag is set
	while(!(I2C1->SR1 & SR1_ADDR)){}
	//Clear addr flag
	tmp = I2C1->SR2;
	//Send memory address
	I2C1->DR = maddr;
	//Wait until transmitter empty
	while(!(I2C1->SR1 & SR1_TXE));
	//Generate restart
	I2C1->CR1 |= CR1_START;
	//Wait until start flag is set
	while(!(I2C1->SR1 & SR1_SB)){}
	//Transmit slave address and READ
	I2C1->DR = saddr << 1 | 1;
	//Wait until addr flag is set
	while (!(I2C1->SR1 & SR1_ADDR)){}
	//Clear addr flag
	tmp = I2C1->SR2;
	//Enable Acknowledge
	I2C1->CR1 |= CR1_ACK;
	while(n>0U)
	{
		//Check if n=1 (one byte)
		if(n==1U)
		{
			//Disable Acknowledge
			I2C1->CR1 &=~(CR1_ACK);
			//Generate stop after data received
			I2C1->CR1 |= CR1_STOP;
			//Wait until RXNE flag is set
			while(!(I2C1->SR1 & SR1_RXNE)){}
			//Read data from DR
			*data++ = I2C1->DR;
			break;
		}
		else
		{
			//Wait until RXNE flag is set
			while(!(I2C1->SR1 & SR1_RXNE)){}
			//Read data from DR
			*data++ = I2C1->DR;
			n--;
		}
	}

}

void I2C1_burstWrite(char saddr, char maddr, int n, char* data)
{
	volatile int tmp; //temporary variable for clean status register
	//Wait until bus not busy
	while(I2C1->SR2 & SR2_BUSY){}
	//Generate start
	I2C1->CR1 |= CR1_START;
	//Wait until start flag is set
	while(!(I2C1->SR1 & SR1_SB)){}
	//Transmit slave address and WRITE
	I2C1->DR = saddr << 1;
	//Wait until addr flag is set
	while(!(I2C1->SR1 & SR1_ADDR)){}
	//Clear addr flag
	tmp = I2C1->SR2;
	//Wait until data register is empty
	while(!(I2C1->SR1 & SR1_TXE));
	//Send memory address
	I2C1->DR = maddr;

	for(int i=0; i<n; i++)
	{
		//Wait until data register is empty
		while(!(I2C1->SR1 & SR1_TXE));
		//Transmit memory address
		I2C1->DR = *data++;
	}

	//Wait until transfer finished
	while(!(I2C1->SR1 & SR1_BTF)){}
	 //Generate stop
	 I2C1->CR1 |= CR1_STOP;
}


