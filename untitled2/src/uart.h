//
// Created by Jun Kim on 3/28/23.
//
//
// Created by Jun Kim on 2/10/23.
//

#ifndef UART_H
#define UART_H

void UART_init(int prescale);

void UART_send( unsigned char data);

unsigned char UART_receive(void);

void UART_write(int data);

void UART_putstring(char* StringPtr);

#endif