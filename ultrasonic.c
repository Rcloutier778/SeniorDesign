#include "stdio.h"
#include <math.h>
#include <stdlib.h>
#include "isr.h"
#include "MK64F12.h"
#include "uart.h"
#include "LEDS.h"
#include "inits.h"


// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
//    (camera clk is the mod value set in FTM2)
// default = .0075f
#define INTEGRATION_TIME .0075f
#define BAUD_RATE 9600      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)

#define MAX_BUF_SZ 128

/*
Initialize UART3 on pins PTC 14 and PTC 15. 
Enables recieve interrupts at 9600 baud. 
*/
void init_ultrasonic(void){
    uint16_t ubd, brfa;
    SIM_SCGC1 |= SIM_SCGC1_UART4_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    PORTC_PCR14 |= PORT_PCR_MUX(3); //TX
    PORTC_PCR15 |= PORT_PCR_MUX(3); //RX
    UART4_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    UART4_C1 = 0;
    UART4_BDH &= ~UART_BDH_SBR_MASK;
    ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));
    UART4_BDH = (((ubd & 0x1F00) >> 8));
    UART4_BDL = (uint8_t)(ubd & UART_BDL_SBR_MASK);
    UART4_C4 &= ~(UART_C4_BRFA_MASK);
    UART4_C4 |= UART_C4_BRFA(brfa);
    UART4_C2 |= UART_C2_RIE_MASK; //Enable recieve interrupts 
    UART4_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
    NVIC_EnableIRQ(UART4_RX_TX_IRQn);    
}


void UART4_RX_TX_IRQHandler(void){
    uint8_t temp;
    char str[100];
    int temp2=0;
    UART4_S1; //clears interrupt
    temp=UART4_D; //data received
    
}




