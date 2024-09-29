# Embedded Peripheral Drivers

### Overview
Low-level drivers for common embedded system peripherals, including ADC, I2C, SPI, UART, and more. This project provides a hardware abstraction layer (HAL) for efficient communication and control of microcontroller peripherals. Built for the STM32F411 microcontroller, the drivers support tasks such as analog-to-digital conversion, serial communication, interrupt handling, and more.

Includes CMSIS and STM32 device-specific headers: core_cm4.h, stm32f411xe.h and stm32f4xx.h. You NEED include the header files to your project. 
---
### Features
- **Analog-to-Digital Conversion (ADC)**: Capture and process analog signals.
- **External Interrupt (EXTI)**: Respond to external events through interrupt-based handling.
- **I2C Communication**: Communicate with peripheral devices using the I2C bus.
- **LCD1602 Display**: Control a 16x2 character LCD display for visual output.
- **SPI Communication**: High-speed communication with external devices via the SPI bus.
- **SysTick Timer**: System timer for periodic interrupts and time management.
- **Timers (TIM)**: Generate delays, PWM signals, and manage time-based events.
- **UART Communication**: Serial communication using the UART protocol.

---

## File Structure

### Source Files (`.c` files)
- **adc.c**: Driver for analog-to-digital conversion.
- **exti.c**: Driver for external interrupt management.
- **i2c.c**: Driver for I2C communication.
- **lcd1602.c**: Driver for controlling an LCD1602 display.
- **spi.c**: Driver for SPI communication.
- **systick.c**: Driver for managing the SysTick system timer.
- **tim.c**: Driver for hardware timers.
- **uart.c**: Driver for UART communication.

### Header Files (`.h` files)
- **adc.h**: Header file for ADC driver, contains function prototypes and constants.
- **exti.h**: Header file for EXTI driver, contains interrupt configuration settings.
- **i2c.h**: Header file for I2C driver, includes communication parameters and functions.
- **lcd1602.h**: Header file for LCD1602, contains display control commands.
- **spi.h**: Header file for SPI communication, defines SPI modes and data handling.
- **systick.h**: Header file for SysTick timer, includes timer configuration settings.
- **tim.h**: Header file for hardware timers, defines timer parameters.
- **uart.h**: Header file for UART communication, contains function prototypes and settings.

---

## Getting Started

### Prerequisites
To use these drivers, you will need:
- An embedded microcontroller development environment (e.g., STM32CubeIDE, Keil uVision).
- A microcontroller with peripherals such as ADC, I2C, SPI, UART, etc.
- Basic understanding of embedded system programming (C language, interrupts, and peripherals).

### Installation
1. Clone the repository or download the source code.
   ```bash
   git clone https://github.com/molecul-a/embedded-peripheral-drivers.git
   ```
2. Include the necessary driver files (`.c` and `.h`) into your project directory.
3. Configure the microcontroller clock and peripheral settings in your main project file.
4. Use the provided functions in your application to initialize peripherals and perform operations.

---

## Usage

### Example: Sampling from sensor(I used temprature) using ADC with interruptor for callback function
```c
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "adc.h"
#include "uart.h"
#include "systick.h"
#include "exti.h"

#define GPIOAEN			(1U<<0)
#define GPIOA_5			(1U<<5)
#define LED_PIN			GPIOA_5

uint32_t sensor_value;
int main(void)
{
	// ENABLE CKL ACCESS TO GPIOA
	RCC->AHB1ENR |= GPIOAEN;
	// SET PA5 AS OUTPUT PIN
	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &=~(1U<<11);

	uart2_tx_init();
	pa1_adc_interrupt_init();
	start_conversion();
	while(1)
	{
	}
}
static void adc_callback(void)
{
	sensor_value = ADC1->DR;
	GPIOA->ODR ^= LED_PIN;
	printf("Sensor value: %d\n\r",(int)sensor_value);
	systickDelayMs(1000);
}
void ADC_IRQHandler(void){
	//Check if RXNE is set
	if((ADC1->SR & SR_EOC) != 0)
	{
		//Clear EOC flag
		ADC1->SR &= ~(SR_EOC);
		//DO something
		adc_callback();
	}
}


```

### Example: Sending Data Over UART using DMA that calls interrupt for callback function
```c
#include "stm32f4xx.h"
#include "uart.h"
#include "systick.h"
#include "exti.h"

#define GPIOAEN			(1U<<0)
#define GPIOA_5			(1U<<5)
#define LED_PIN			GPIOA_5

int main(void)
{
	char massage[31] = "Hello from STM32 DMA TRANSFER\n\r";

	// ENABLE CKL ACCESS TO GPIOA
	RCC->AHB1ENR |= GPIOAEN;
	// SET PA5 AS OUTPUT PIN
	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &=~(1U<<11);


	uart2_tx_init();
	dma1_stream6_init((uint32_t)massage, (uint32_t)&USART2->DR, 31);

	while(1)
	{
	}
}
static void uart_callback(void)
{
	systickDelayMs(1000);
	GPIOA->ODR ^= LED_PIN;
	printf("WOW!\n\r");
}
void DMA1_Stream6_IRQHandler(void){
	//Check for transfer complete interrupt
	if ((DMA1->HISR & DMA_HISR_TCIF6))
	{
		//Clear flag
		DMA1->HIFCR &=~(DMA_HIFCR_CTCIF6);
		//Do something
		uart_callback();
	}
}

```
### Example: Sending Data to LCD16X2 using I2C 
```c
#include "lcd1602.h"
int main(void)
{

	lcd_init ();
	lcd_put_cur(0, 0);
	lcd_send_string ("HELLO WORLD");
	lcd_put_cur(1, 0);
	lcd_send_string("FROM STM32!!!");

	while(1)
	{
	}
}
```
### Example: Sending Data to LCD16X2 using I2C 
```c
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "uart.h"
#include "tim.h"

#define GPIOAEN			(1U<<0)
#define GPIOA_5			(1U<<5)
#define LED_PIN			GPIOA_5

int main(void)
{
	// ENABLE CKL ACCESS TO GPIOA
	RCC->AHB1ENR |= GPIOAEN;
	// SET PA5 AS OUTPUT PIN
	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &=~(1U<<11);

	uart2_tx_init();
	tim2_1hz_init();
	while(1)
	{
		//Wait for UIF
		while(!(TIM2->SR & SR_UIF)){}
		//Clear UIF
		TIM2->SR &= ~(SR_UIF);

		GPIOA->ODR ^= LED_PIN;
		printf("A second passed!\n\r");
	}
}
```
### Example: ADC module with continuous conversation
```c
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "adc.h"
#include "uart.h"

#define GPIOAEN			(1U<<0)
#define GPIOA_5			(1U<<5)
#define LED_PIN			GPIOA_5

char key;
uint32_t sensor_value;
int main(void)
{
	// ENABLE CKL ACCESS TO GPIOA
	RCC->AHB1ENR |= GPIOAEN;
	// SET PA5 AS OUTPUT PIN
	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &=~(1U<<11);
	uart2_rxtx_init();

	pa1_adc_init();

	while(1)
	{
		key = uart2_read();
		if(key == '1')
		{
			GPIOA->ODR |=  LED_PIN;
			start_conversion();
			sensor_value = adc_read();
			printf("Sensor value: %d\n\r",(int)sensor_value);
		}
		else{
			GPIOA->ODR &= ~LED_PIN;
		}

	}
}

```
### Example: ADC module with single conversation
```c
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "uart.h"
#include "adc.h"

uint32_t sensor_value;

int main(void)
{
	uart2_tx_init();
	pa1_adc_init();
	while(1)
	{
		start_converstion();
		sensor_value =  adc_read();
		printf("Sensor value : %d \n\r",(int)sensor_value);
	}
}
```
---

## File Descriptions

### Peripheral Drivers

- **adc.c / adc.h**:  
   Driver for the Analog-to-Digital Converter (ADC). Converts analog signals into digital values for processing.
   
- **exti.c / exti.h**:  
   External Interrupt (EXTI) driver. Configures and manages interrupts triggered by external hardware events.
   
- **i2c.c / i2c.h**:  
   I2C communication driver. Provides functions for data transmission and reception over the I2C bus.
   
- **lcd1602.c / lcd1602.h**:  
   Driver for controlling a 16x2 character LCD1602 display.
   
- **spi.c / spi.h**:  
   SPI communication driver. Enables high-speed communication with external devices over the SPI bus.
   
- **systick.c / systick.h**:  
   SysTick timer driver. Provides system-wide periodic interrupts for timing purposes.
   
- **tim.c / tim.h**:  
   Timer (TIM) driver for generating time delays and PWM signals.
   
- **uart.c / uart.h**:  
   UART communication driver for serial data transfer.

---
