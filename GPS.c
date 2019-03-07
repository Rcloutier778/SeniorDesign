#include "stdio.h"
#include <math.h>
#include <stdlib.h>
#include "MK64F12.h"
#include "uart.h"
#include "LEDS.h"
#include "stdio.h"

#define BAUD_RATE 9600      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)


/*
Send inerrupt to bluetooth, get gps from that
Send interrupt to arduino, get gps from that
Calc distance using hypotenous
Calc angle 

TODO: Don't calc distance or angle, average it with camera and ultrasonic?
*/
void getGPS(void){
    
    //Get GPS from Bluetooth  -- brian
    //TODO
    
    //Copy GPS interrupt and polling bluetooth code, use for mark
    
}

void initGPS(void){
    
}



void uart2_init(){
//define variables for baud rate and baud rate fine adjust
uint16_t ubd, brfa;

//Enable clock for UART
  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

//Configure the port control register to alternative 3 (which is UART mode for K64)
  PORTD_PCR2 |= PORT_PCR_MUX(3);
  PORTD_PCR3 |= PORT_PCR_MUX(3);

/*Configure the UART for establishing serial communication*/
  
 
//Disable transmitter and receiver until proper settings are chosen for the UART module
  UART2_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
  //UART0_C2 &= !(UART_C2_RE_MASK);

//Select default transmission/reception settings for serial communication of UART by clearing the control register 1
  UART2_C1 = 0;

//UART Baud rate is calculated by: baud rate = UART module clock / (16 × (SBR[12:0] + BRFD))
//13 bits of SBR are shared by the 8 bits of UART3_BDL and the lower 5 bits of UART3_BDH 
//BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
//BRFA is defined by the lower 4 bits of control register, UART0_C4 

//calculate baud rate settings: ubd = UART module clock/16* baud rate
ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));  

//clear SBR bits of BDH
  UART2_BDH &= ~UART_BDH_SBR_MASK;
  
  //UART4_BDH |= UART_BDH_RXEDGIE_MASK;
  //UART0_BDH |= UART_BDH_RXEDGIE_MASK;
//distribute this ubd in BDH and BDL
  UART2_BDH = (((ubd & 0x1F00) >> 8));
  
  //UART0_BDH = ((ubd >> 8) | UART_BDH_SBR_MASK);
  //UART0_BDL = ubd & UART_BDL_SBR_MASK;
  UART2_BDL = (uint8_t)(ubd & UART_BDL_SBR_MASK);
  

//BRFD = (1/32)*BRFA 
//make the baud rate closer to the desired value by using BRFA
  brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

//write the value of brfa in UART0_C4
  UART2_C4 &= ~(UART_C4_BRFA_MASK);
  UART2_C4 |= UART_C4_BRFA(brfa);
  
  //UART0_C4 |= (brfa | UART_C4_BRFA_MASK);
     
//Enable transmitter and receiver of UART
  UART2_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
    
}
