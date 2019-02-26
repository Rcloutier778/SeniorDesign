/*
 *Contains all interrupt handlers for the K64F
 * 
 * Author:  Richard Cloutier
 * Created:  
 * Modified:  
*/
#include "stdio.h"
#include <math.h>
//#include <string>
#include <stdlib.h>
#include "isr.h"
#include "MK64F12.h"
#include "uart.h"
#include "LEDS.h"
// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 

#define MAX_BUF_SZ 128
#define clip(n, lower, upper){\
	if(n<lower){\
		n=lower;\
	}else if(n>upper){\
		n=upper;\
	}\
}
// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
//		ADC reads start
extern int pixcnt;
// clkval toggles with each FTM interrupt
extern int clkval;
// data count
extern int dataCount_buf;
extern int dataCount_line;
extern uint16_t line[MAX_BUF_SZ];
extern uint16_t line2[MAX_BUF_SZ];
// ptr to the buffer array of camera data
extern uint16_t* bufferPtr;
// ptr to the current array of camera data
extern uint16_t* linePtr;

extern int ready;
// These variables are for streaming the camera
//	 data over UART
extern int debugcamdata;
extern int capcnt;
//extern char str[100];

// ADC0VAL holds the current ADC value
extern uint16_t ADC0VAL;

extern int binline[128];

void PORTA_IRQHandler(void){ //For switch 3
    
	return;
}
	
/*
If switch 2 is pressed, the IRQ handler is triggered.
Clears the interrupt flag and toggles the ready switch
*/
void PORTC_IRQHandler(void){ //For switch 2
    PORTC_ISFR = PORT_ISFR_ISF_MASK; //clear interrupt
    ready=1;
    return;
}

/* ADC0 Conversion Complete ISR  */
void ADC0_IRQHandler(void) {
	// Reading ADC0_RA clears the conversion complete flag
	ADC0VAL = ADC0_RA;
}

/* 
* FTM2 handles the camera driving logic
*	This ISR gets called once every integration period
*		by the periodic interrupt timer 0 (PIT0)
*	When it is triggered it gives the SI pulse,
*		toggles clk for 128 cycles, and stores the line
*		data from the ADC into the line variable
*/
void FTM2_IRQHandler(void){ //For FTM timer
	// Clear interrupt
  FTM2_SC &= ~FTM_SC_TOF_MASK;
    
	// Toggle clk
  if(clkval){
    //turn clk off
    clkval=0;
    GPIOB_PCOR = (1<<9);
  }else{
    //turn clk on
    clkval=1;
    GPIOB_PSOR = (1<<9);
  }
  
  if (MAX_BUF_SZ == dataCount_buf){
			if((MAX_BUF_SZ == dataCount_line || NULL == linePtr)) {
				bufferPtr = bufferPtr == line ? line2 : line;
				linePtr = bufferPtr == line ? line2 : line;
				dataCount_line = 0;
				dataCount_buf = 0;
			}else{
				return;
			}
  }
	
	
	// Line capture logic
	if ((pixcnt >= 2) && (pixcnt < 256)) {
		if (!clkval) {	// check for falling edge
			// ADC read (note that integer division is 
			//  occurring here for indexing the array)
			bufferPtr[pixcnt/2] = ADC0VAL;
      dataCount_buf++;
		}
		pixcnt += 1;
	} else if (pixcnt < 2) {
		if (pixcnt == -1) {
			GPIOB_PSOR |= (1 << 23); // SI = 1
		} else if (pixcnt == 1) {
			GPIOB_PCOR |= (1 << 23); // SI = 0
			// ADC read
			bufferPtr[0] = ADC0VAL;
      dataCount_buf++;
		} 
		pixcnt += 1;
	} else {
		GPIOB_PCOR |= (1 << 9); // CLK = 0
		clkval = 0; // make sure clock variable = 0
		pixcnt = -2; // reset counter
		// Disable FTM2 interrupts (until PIT0 overflows
		//   again and triggers another line capture)
		FTM2_SC &= ~FTM_SC_TOIE_MASK;
	
	}
	return;
}

/* PIT0 determines the integration period
*		When it overflows, it triggers the clock logic from
*		FTM2. Note the requirement to set the MOD register
* 	to reset the FTM counter because the FTM counter is 
*		always counting, I am just enabling/disabling FTM2 
*		interrupts to control when the line capture occurs
*/
void PIT0_IRQHandler(void){
	if (debugcamdata) {
		// Increment capture counter so that we can only 
		//	send line data once every ~2 seconds
		capcnt += 1;
	}
	// Clear interrupt
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	
	// Setting mod resets the FTM counter
	//PIT_MCR |= PIT_MCR_MDIS_MASK;
    FTM2_MOD = (DEFAULT_SYSTEM_CLOCK)/100000;
	
	// Enable FTM2 interrupts (camera)
	FTM2_SC |= FTM_SC_TOIE_MASK;
	
	return;
}
