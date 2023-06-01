/*
 * uart.c
 *
 *  Created on: May 30, 2023
 *      Author: skafash
 */

#include "uart.h"


#define GPIOAEN		(1U<<0)
#define USART2EN		(1U<<17)

#define SYS_FREQ	16000000
#define APB1_CLK	SYS_FREQ

#define UART_BAUDRATE	115200

#define SR_TXE		(1U<<7)

#define SR_RXNE		(1U<<5)

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);


void usart2_write(int ch);

int __io_putchar(int ch){
	usart2_write(ch);
	return ch;
}

void usart2_rxtx_init(void){
	// *****Configure USART GPIO pin*******
	// Enable clock access to GPIOA
	RCC->AHB1ENR |= GPIOAEN;
	// Set PA2 mode to alternate function mode
	GPIOA->MODER &= ~(1U<<4); //Set bit 4 to 0
	GPIOA->MODER |= (1U<<5); //Set bit 5 to 1
	// SET PA2 alternate function type to UART_TX (AF07)
	GPIOA->AFR[0]  |= (1U<<8);
	GPIOA->AFR[0]  |= (1U<<9);
	GPIOA->AFR[0]  |= (1U<<10);
	GPIOA->AFR[0]  &= ~(1U<<11);

	//Set PA3 mode to alternate function mode
	GPIOA->MODER &= ~(1U<<6);  //Set bit 6 to 0
	GPIOA->MODER |= (1U<<7);  //Set bit 7 to 1
	//Set PA3 alternate function type to UART_RX
	GPIOA->AFR[0]  |= (1U<<12);
	GPIOA->AFR[0]  |= (1U<<13);
	GPIOA->AFR[0]  |= (1U<<14);
	GPIOA->AFR[0]  &= ~(1U<<15);


	// *****Configure USART module*********
	// Enable clock access to USART2
	RCC->APB1ENR |= USART2EN;
	// Configure baudrate
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);
	// Configure the transfer direction
	USART2->CR1 = (1U<<3);
	USART2->CR1 |= (1U<<2);
	// Enable USART module
	USART2->CR1 |= (1U<<13);
}

void usart2_tx_init(void){
	// *****Configure USART GPIO pin*******
	// Enable clock access to GPIOA
	RCC->AHB1ENR |= GPIOAEN;
	// Set PA2 mode to alternate function mode
	GPIOA->MODER &= ~(1U<<4); //Set bit 4 to 0
	GPIOA->MODER |= (1U<<5); //Set bit 5 to 1
	// SET PA2 alternate function type to UART_TX (AF07)
	GPIOA->AFR[0]  |= (1U<<8);
	GPIOA->AFR[0]  |= (1U<<9);
	GPIOA->AFR[0]  |= (1U<<10);
	GPIOA->AFR[0]  &= ~(1U<<11);


	// *****Configure USART module*********
	// Enable clock access to USART2
	RCC->APB1ENR |= USART2EN;
	// Configure baudrate
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);
	// Configure the transfer direction
	USART2->CR1 = (1U<<3);

	// Enable USART module
	USART2->CR1 |= (1U<<13);
}


char usart2_read(void){
	// Make sure the receive data register is not empty
	while (!(USART2->SR & SR_RXNE)){}
	// Read data
	return USART2->DR;
}

void usart2_write(int ch){
	// Make sure the transmit data register is empty
	while (!(USART2->SR & SR_TXE)){}
	// Write to transmit data register
	USART2->DR = (ch & 0xFF);
}



static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate){
	USARTx->BRR = compute_uart_bd(PeriphClk, BaudRate);
}

static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate){
	return ((PeriphClk + (BaudRate/2U))/BaudRate);
}
