#ifndef UART_H_
#define UART_H_

#include <stdint.h>

void uart_init(void);
uint8_t uart_getchar(void);
void uart_putchar(char ch);
void uart_put(char *ptr_str);
void putnumU(int i);

#endif /* UART_H_ */
