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

extern float SL;
extern float SH;
extern float TL;
extern float TH;
extern float TURN_DESIRED;
extern float STRAIGHT_DESIRED;
extern float REV_BRAKE_DESIRED[3];
extern float TURBO_DESIRED;
extern float SUPER_DESIRED;
extern float TURBO_L;
extern float TURBO_H;
extern float SUPER_L;
extern float SUPER_H;

extern float KP;
extern float KI;
extern float KD;
extern int RevBrake[2];
extern int aggro;

/*
If switch 3 is pressed, the IRQ handler is triggered.
Clears the interrupt flag, determines what level of agression to 
set the car to.
Red==safe
Blue==Medium
Green==Fast
White==Dangerous
*/
void PORTA_IRQHandler(void){ //For switch 3
    PORTA_ISFR = PORT_ISFR_ISF_MASK; //clear interrupt
    aggro++;
    if(aggro>3){
        aggro=0;
    }
    if(aggro==0){ //safe
        LEDon(RED);
        TURN_DESIRED=30.0f;//75.0f; 
        STRAIGHT_DESIRED=40.0f;//80.0f
        REV_BRAKE_DESIRED[0]=0.0f;//-10.0f;//20.0f;
        REV_BRAKE_DESIRED[1]=-10.0f;//-5.0f;//0.0f; //-25
        REV_BRAKE_DESIRED[2]=-20.0f;//-10.0f;//-55.0f; //-50
        
        TURBO_DESIRED=STRAIGHT_DESIRED+20.0f;
        TURBO_L=TURBO_DESIRED-20.0f;
        TURBO_H=100.0f;
        
        SL=STRAIGHT_DESIRED-20.0f;
        SH=STRAIGHT_DESIRED+20.0f;
        
        TL=TURN_DESIRED-20.0f;
        TH=TURN_DESIRED+20.0f;
        
        SUPER_DESIRED=STRAIGHT_DESIRED+10.0f;
        SUPER_L=SUPER_DESIRED-20.0f;
        SUPER_H=100.0f;
        
        KP=0.70f; //77
        KI=0.70f; //70
        KD=0.20f; //20
    }else if(aggro==1){ //confirmed working
        LEDon(BLUE);
        TURN_DESIRED=60.0f;//70.0f; //65
        STRAIGHT_DESIRED=75.0f;//80.0f; //75
        REV_BRAKE_DESIRED[0]=0.0f; //0.0f
        REV_BRAKE_DESIRED[1]=-25.0f; //-25
        REV_BRAKE_DESIRED[2]=-40.0f;//-55.0f; //-50
        
        TURBO_DESIRED=STRAIGHT_DESIRED+20.0f;
        TURBO_L=TURBO_DESIRED-80.0f;
        TURBO_H=100.0f;
        
        SL=STRAIGHT_DESIRED-40.0f;
        SH=STRAIGHT_DESIRED+20.0f;
        
        TL=TURN_DESIRED-40.0f;
        TH=TURN_DESIRED+20.0f;
        
        SUPER_DESIRED=STRAIGHT_DESIRED+10.0f;
        SUPER_L=SUPER_DESIRED-60.0f;
        SUPER_H=100.0f;
        
        KP=0.65f; //60
        KI=0.70f; //70
        KD=0.25f; //25
    }else if(aggro==2){
        //if doesn;t work, in main, convert straight into turning in turning and up straight
        LEDon(GREEN);
        TURN_DESIRED=65.0f;//75.0f; 
        STRAIGHT_DESIRED=75.0f;//80.0f
        REV_BRAKE_DESIRED[0]=-25.0f;//-10.0f;//20.0f;
        REV_BRAKE_DESIRED[1]=-55.0f;//-5.0f;//0.0f; //-25
        REV_BRAKE_DESIRED[2]=-75.0f;//-10.0f;//-55.0f; //-50
        
        TURBO_DESIRED=STRAIGHT_DESIRED+20.0f;
        TURBO_L=TURBO_DESIRED-20.0f;
        TURBO_H=100.0f;
        
        SL=STRAIGHT_DESIRED-20.0f;
        SH=STRAIGHT_DESIRED+20.0f;
        
        TL=TURN_DESIRED-20.0f;
        TH=TURN_DESIRED+20.0f;
        
        SUPER_DESIRED=STRAIGHT_DESIRED+10.0f;
        SUPER_L=SUPER_DESIRED-20.0f;
        SUPER_H=100.0f;
        
        KP=1.00f; //95
        KI=0.90f; //70
        KD=0.20f; //20
    }else if(aggro==3){//dejaVu
        LEDon(WHITE);
        RevBrake[0]=90;
        RevBrake[1]=50;
        TURN_DESIRED=65.0f;//75.0f; 
        STRAIGHT_DESIRED=75.0f;//80.0f
        REV_BRAKE_DESIRED[0]=-25.0f;//-10.0f;//20.0f;
        REV_BRAKE_DESIRED[1]=-55.0f;//-5.0f;//0.0f; //-25
        REV_BRAKE_DESIRED[2]=-75.0f;//-10.0f;//-55.0f; //-50
        
        TURBO_DESIRED=STRAIGHT_DESIRED+20.0f;
        TURBO_L=TURBO_DESIRED-20.0f;
        TURBO_H=100.0f;
        
        SL=STRAIGHT_DESIRED-20.0f;
        SH=STRAIGHT_DESIRED+20.0f;
        
        TL=TURN_DESIRED-20.0f;
        TH=TURN_DESIRED+20.0f;
        
        SUPER_DESIRED=STRAIGHT_DESIRED+10.0f;
        SUPER_L=SUPER_DESIRED-20.0f;
        SUPER_H=100.0f;
        
        KP=1.15f; //95
        KI=0.90f; //70
        KD=0.20f; //20
    }
        
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
