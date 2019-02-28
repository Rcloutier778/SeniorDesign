/*
 * File:        uart.c
 * Purpose:     Provide UART routines for serial IO
 *
 * Notes:
 *
 */

#include "MK64F12.h"
#define BAUD_RATE 9600      //default baud rate
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)

void uart_init() {
    //define variables for baud rate and baud rate fine adjust
    uint16_t ubd, brfa;

    //Enable clocks for UART
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // Enable Port B clock
    SIM_SCGC4 |= SIM_SCGC4_UART0_MASK; // Enable UART0 clock

    //Configure the port control register to alternative 3 (which is UART mode for K64)
    PORTB_PCR16 |= PORT_PCR_MUX(3); // UART0_RX
    PORTB_PCR17 |= PORT_PCR_MUX(3); // UART0_TX

    /*Configure the UART for establishing serial communication*/
    //Disable transmitter and receiver until proper settings are chosen for the UART module
    UART0_C2 &= ~UART_C2_TE_MASK; // Disable transmitter
    UART0_C2 &= ~UART_C2_RE_MASK; // Disable receiver

    //Select default transmission/reception settings for serial communication of UART by clearing the control register 1
    UART0_C1 = 0;

    //UART Baud rate is calculated by: baud rate = UART module clock / (16 × (SBR[12:0] + BRFD))
    //13 bits of SBR are shared by the 8 bits of UART0_BDL and the lower 5 bits of UART0_BDH
    //BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
    //BRFA is defined by the lower 4 bits of control register, UART0_C4

    //calculate baud rate settings: ubd = UART module clock/16* baud rate
    ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));

    //clear SBR bits of BDH
    UART0_BDH &= ~UART_BDH_SBR_MASK;

    //distribute this ubd in BDH and BDL
    UART0_BDH |= UART_BDH_SBR(ubd >> 8);
    UART0_BDL = UART_BDL_SBR(ubd);


    //BRFD = (1/32)*BRFA
    //make the baud rate closer to the desired value by using BRFA
    brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

    //clear BRFA bits of UART0_C4
    UART0_C4 &= ~UART_C4_BRFA_MASK;

    //write the value of brfa in UART0_C4
    UART0_C4 |= UART_C4_BRFA(brfa);

    //Enable transmitter and receiver of UART
    UART0_C2 |= UART_C2_TE_MASK; // Enable transmitter
    UART0_C2 |= UART_C2_RE_MASK; // Enable receiver
}

uint8_t uart_getchar() {
    /* Wait until there is space for more data in the receiver buffer*/
    while (!(UART0_S1 & UART_S1_RDRF_MASK));

    /* Return the 8-bit data from the receiver */
    return UART0_D;
}

void uart_putchar(char ch) {
    /* Wait until transmission of previous bit is complete */
    while (!(UART0_S1 & UART_S1_TDRE_MASK));

    /* Send the character */
    UART0_D = ch;
}

void uart_put(char *ptr_str) {
    /*use putchar to print string*/
    while (*ptr_str) {
        uart_putchar(*ptr_str++);
    }
}

void putnumU(int i) {
    char str[11];       // Buffer to hold digits (in reverse order)
    short num_digs = 0; // Number of digits read so far

    // Zero is a special case. Just print it
    if (i == 0) {
        uart_putchar('0');
    }

    // Read digits one by one until there are no more
    while (i > 0) {
        str[num_digs++] = i % 10 + '0'; // Add the ones digit to the buffer
        i -= i % 10;                    // Subtract off the ones digit
        i /= 10;                        // Divide out the ones place
    }

    // Since digits are read starting from the ones place, we get them in
    // reverse order. Print the buffer backwards to correct the order
    while (num_digs > 0) {
        uart_putchar(str[--num_digs]);
    }
}
