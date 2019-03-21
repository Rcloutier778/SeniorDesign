#ifndef UART_H
#define UART_H

#include <stdint.h>

void put(char *ptr_str);
void putnumU(int i);
void uart0_init(void);
uint8_t uart_getchar(void);
void uart_putchar(char ch);
void get(char *ptr_str);
#endif
