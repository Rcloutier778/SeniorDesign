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
#define INTEGRATION_TIME 0.000001f
#define BAUD_RATE 9600      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)
#define FTM3_MOD_VALUE (DEFAULT_SYSTEM_CLOCK/200)

#define MAX_BUF_SZ 128

static volatile unsigned int PWMTick2 = 0;

int ultrasonic_counter=0;
//  uS/58=centimeter   uS/148=inch
float ultrasonic_distance=0.0f;
int ultrasonic_ready_flag=0;
int measureEcho=0; //If ultrasonic echo is high or low rn. 

float getUltrasonic(void){
    ultrasonic_counter=0;
    ultrasonic_ready_flag=0;
    
    //fire trigger
    GPIOD_PSOR |= (1<<1);

    // Enable PIT interrupts
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
    
    //wait 10 us
    while((ultrasonic_counter<10));
    
    //Disable pit interrupts
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;
    ultrasonic_counter=0;
    
    //set trigger low
    GPIOD_PCOR |= (1<<1);
    measureEcho=1;
    
    
    //Wait until it measures the echo pulse
    while(ultrasonic_ready_flag==0);
    ultrasonic_distance=ultrasonic_counter/148;
    
    
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;
    measureEcho=0;
    return ultrasonic_distance;
}

//Used to time output pulse
void PIT0_IRQHandler(void){
	/*// Clear interrupt
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	ultrasonic_counter++;
    if(measureEcho){
        if (measureEcho==2 && (GPIO_PDIR_PDI(1)==0)
        if(measureEcho==2 && (GPIO_PDIR_PDI(1)==0)){
            ultrasonic_ready_flag=1;
        }
    }
	return;*///TODO
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
	PIT_LDVAL0 = (uint32_t) DEFAULT_SYSTEM_CLOCK * INTEGRATION_TIME; 
		
	// Enable timer interrupts
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
	
	// Enable the timer 0,1,2,3?
	//PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

	// Clear interrupt flag
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT0_IRQn);
	return;
}

void init_FTM(void){

// 12.2.13 Enable the Clock to the FTM0 Module
	SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;
	
	
	// 11.4.1 Route the output of FTM channel 0 to the pins
	// Use drive strength enable flag to high drive strength
	//These port/pins may need to be updated for the K64 <Yes, they do. Here are two that work.>
    PORTD_PCR3  = PORT_PCR_MUX(4)  | PORT_PCR_DSE_MASK;//Ch3
			
	// 39.3.10 Disable Write Protection
	FTM3_MODE |= FTM_MODE_WPDIS_MASK;
	
	
	// 39.3.8 Set the Counter Initial Value to 0
	FTM3_CNTIN = 0x0F00;
	
    // 39.3.4 FTM Counter Value
	// Initialize the CNT to 0 before writing to MOD
	FTM3_CNT = 0;
	
    
    
	// 39.3.6 Set the Status and Control of both channels
	// Used to configure mode, edge and level selection
	// See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
	FTM3_C3SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM3_C3SC &= ~FTM_CnSC_ELSA_MASK;

	
	// 39.3.3 FTM Setup
	// Set prescale value to 1 
	// Chose system clock source
	FTM3_SC |= (FTM_SC_PS(5) | FTM_SC_CLKS(1));

	// Enable Interrupt Vector for FTM
    //NVIC_EnableIRQ(FTM0_IRQn);
    //NVIC_EnableIRQ(FTM3_IRQn);
    
    //10 us
    FTM3_C3V = (uint16_t) (0x000F);

    //60010 us
    // Update the clock to the new frequency
	FTM3_MOD = ((uint16_t) (0xFFFF));

}

/*
Initialize UART3 on pins PTC 14 and PTC 15. 
Enables recieve interrupts at 9600 baud. 
*/
void init_ultrasonic(void){
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
    
    PORTD_PCR2 |= PORT_PCR_MUX(1); //UART2 RX
    GPIOD_PDDR |= (0<<2);
    
    PORTD_PCR1 |= PORT_PCR_MUX(1);
    GPIOD_PDDR |= (1<<1);
    GPIOD_PSOR |= (1<<1);    
    PORTD_PCR1 |= PORT_PCR_DSE_MASK;
    
    init_PIT();
    //init_FTM();
    //NVIC_EnableIRQ(PORTD_IRQn);
}

