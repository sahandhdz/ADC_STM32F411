/*
 * uart.h
 *
 *  Created on: May 30, 2023
 *      Author: skafash
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f4xx.h"
#include <stdint.h>

void usart2_tx_init(void);
char usart2_read(void);
void usart2_rxtx_init(void);



#endif /* UART_H_ */
