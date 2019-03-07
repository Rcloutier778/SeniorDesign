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
#define ECHO_RESPONSE 0.000001f
#define TRIGGER_TIME  0.000001f
#define BAUD_RATE 9600      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)
#define FTM3_MOD_VALUE (DEFAULT_SYSTEM_CLOCK/200)
#define SCALE_FACTOR 2.666f
#define MAX_BUF_SZ 128

int ultrasonic_state=0;
int ultrasonic_counter=0;
/*
Gets a command to get the distance
*/

float getUltrasonic(void){
    int i;
    //  uS/58=centimeter   uS/148=inch
    float ultrasonic_distance=0.0f;
    ultrasonic_state=0;
    ultrasonic_counter=0;
    
    //Set pit timer to trigger interval
    PIT_LDVAL0 = (uint32_t) DEFAULT_SYSTEM_CLOCK * TRIGGER_TIME;
    
    //fire trigger
    GPIOD_PCOR |= (1<<1);
    
    // Enable the timer
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
    // Clear interrupt flag
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
    
    //Wait 10 uS
    while(ultrasonic_state == 0);
  
    //Set trigger low
    GPIOD_PSOR |= (1<<1);
    
    ultrasonic_counter=0;

    //Set pit timer to echo response interval
    PIT_LDVAL0 = (uint32_t) DEFAULT_SYSTEM_CLOCK * ECHO_RESPONSE; //1 uS
    // Enable the timer
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
    // Clear interrupt flag
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
    while(ultrasonic_state != 3);
    
    
    
    ultrasonic_distance=((ultrasonic_counter/148.0f)*SCALE_FACTOR)-1;
    return ultrasonic_distance;
}

//Used to time output pulse
void PIT0_IRQHandler(void){
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
    if(ultrasonic_state==0){
        ultrasonic_counter++;
        if(ultrasonic_counter > 10*TRIGGER_TIME*1000000){
            // Disable the timer
            PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;
            ultrasonic_state=1;
            return;
        }
    }else{
        if((GPIOD_PDIR & (1<<0))==0){
            //Echo has been received and has ended
            if(ultrasonic_state==2){
                // Disable the timer
                PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;
                ultrasonic_state=3;
            }
            //Else wait for response
            
        }else{
            //Start counting echo response
            if(ultrasonic_state==1){
                ultrasonic_state=2;
            }
            //Count up
            else{
                ultrasonic_counter++;
            }
        }
    }
	
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
	PIT_LDVAL0 = (uint32_t) DEFAULT_SYSTEM_CLOCK * ECHO_RESPONSE; 
		
	// Enable timer interrupts
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
	
	// Enable the timer
	//PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

	// Clear interrupt flag
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT0_IRQn);
	return;
}


void init_ultrasonic(void){
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
    
    PORTD_PCR0 |= PORT_PCR_PE_MASK;
    PORTD_PCR0 |= PORT_PCR_MUX(1); //Echo recieve
    GPIOD_PDDR &= ~(1<<0);
    PORTD_PCR0 |= PORT_PCR_IRQC(12);
    
    
    PORTD_PCR1 |= PORT_PCR_MUX(1);
    GPIOD_PDDR |= (1<<1);
    GPIOD_PSOR |= (1<<1);    
    PORTD_PCR1 |= PORT_PCR_DSE_MASK;
    
    init_PIT();
}

