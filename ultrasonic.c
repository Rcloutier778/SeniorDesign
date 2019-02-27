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
#define INTEGRATION_TIME 0.00001f
#define BAUD_RATE 9600      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)

#define MAX_BUF_SZ 128

int ultrasonic_counter=0;
//  uS/58=centimeter   uS/148=inch
float ultrasonic_distance=0.0f;
int ultrasonic_ready_flag=0;






void UART4_RX_TX_IRQHandler(void){
    uint8_t temp;
    char str[100];
	
    UART4_S1; //clears interrupt
    temp=UART4_D; //data received
    if (temp==1){
        ultrasonic_counter++;
		ultrasonic_ready_flag=0;
    }else if(temp==0){
        ultrasonic_distance=ultrasonic_counter/14.80f;
        ultrasonic_counter=0;
		ultrasonic_ready_flag=1;
		
    }
	UART4_C2 &= ~UART_C2_RIE_MASK; //Disable recieve interrupts 
    //UART4_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    return;
}


//internal
int ultrasonic_input_counter=0; //counter for input trigger
int ultrasonic_input_sending=1; //sending high input signal

/* PIT0 determines the integration period
*		When it overflows, it triggers the clock logic from
*		FTM2. Note the requirement to set the MOD register
* 	to reset the FTM counter because the FTM counter is 
*		always counting, I am just enabling/disabling FTM2 
*		interrupts to control when the line capture occurs
fire every ms
*/
void PIT0_IRQHandler(void){
	// Clear interrupt
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	
	// Setting mod resets the FTM counter
	//PIT_MCR |= PIT_MCR_MDIS_MASK;
	
	//Send high input trigger pulse for 10 uS
	if (ultrasonic_input_sending==1){
		//Since timer counts in increments in 10 uS, only wait 1 count
		if (ultrasonic_input_counter == 100000){
			ultrasonic_input_counter=0;
			ultrasonic_input_sending=0;
			GPIOC_PCOR |= (1<<15);
		}
	}else{
		if (ultrasonic_input_counter == 100000){
			ultrasonic_input_sending=1;
			ultrasonic_input_counter=0;
            GPIOC_PSOR |= (1<<15);
		}
	}
	
	ultrasonic_input_counter++;
	
	
	// Enable uart4 interrupts (ultrasonic)
	UART4_C2 |= UART_C2_RIE_MASK; //Enable recieve interrupts 
    UART4_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
	
	
	return;
}

/* Initialization of PIT timer to control 
* 		integration period
*/
void init_PIT(void){
	// Setup periodic interrupt timer (PIT)
	
	// Enable clock for timers
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	PIT_MCR &= ~PIT_MCR_MDIS_MASK;
	
	// Enable timers to continue in debug mode
	PIT_MCR &= ~PIT_MCR_FRZ_MASK; // In case you need to debug
	
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	PIT_LDVAL0 = (uint32_t) DEFAULT_SYSTEM_CLOCK * INTEGRATION_TIME; //10 uS
		
	// Enable timer interrupts
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
	
	// Enable the timer 0,1,2,3?
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

	// Clear interrupt flag
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT0_IRQn);
	return;
}

/*
Initialize UART3 on pins PTC 14 and PTC 15. 
Enables recieve interrupts at 9600 baud. 
*/
void init_ultrasonic(void){
    uint16_t ubd, brfa;
    SIM_SCGC1 |= SIM_SCGC1_UART4_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    PORTC_PCR14 |= PORT_PCR_MUX(3); //UART4 RX
	
	// 0/3.3v pin for transmit pulses
    PORTC_PCR15 |= PORT_PCR_MUX(1);
	PORTC_PCR15 |= PORT_PCR_PE_MASK;
	GPIOC_PDDR |= (1<<15);
	
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
    init_PIT();
}

