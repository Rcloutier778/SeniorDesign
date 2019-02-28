/*







*/


#include "stdio.h"
#include <math.h>
#include <stdlib.h>
#include "isr.h"
#include "MK64F12.h"
#include "uart.h"

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
#define clip(n, lower, upper){\
    if(n<lower){\
        n=lower;\
    }else if(n>upper){\
        n=upper;\
    }\
}

float SL;
float SH;
float TL;
float TH;
float TURN_DESIRED;
float STRAIGHT_DESIRED;
float REV_BRAKE_DESIRED[3];
float TURBO_DESIRED;
float SUPER_DESIRED;
float TURBO_L;
float TURBO_H;
float SUPER_L;
float SUPER_H;
int TESTING;

int ready;

int RX_lcv=0; //Current index of UART_TX_STR

extern float KP;
extern float KI;
extern float KD;

/*
Initialize UART3 on pins PTB10 and PTB 11. Used to transmit
and recieve control data over the HC-06 bluetooth slave module.
Enables recieve interrupts at 9600 baud. 
*/
void init_BT(void){
    uint16_t ubd, brfa;
    SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    PORTB_PCR11 |= PORT_PCR_MUX(3); //BT TX
    PORTB_PCR10 |= PORT_PCR_MUX(3); //BT RX
    UART3_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    UART3_C1 = 0;
    UART3_BDH &= ~UART_BDH_SBR_MASK;
    ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));
    UART3_BDH = (((ubd & 0x1F00) >> 8));
    UART3_BDL = (uint8_t)(ubd & UART_BDL_SBR_MASK);
    UART3_C4 &= ~(UART_C4_BRFA_MASK);
    UART3_C4 |= UART_C4_BRFA(brfa);
    UART3_C2 |= UART_C2_RIE_MASK; //Enable recieve interrupts 
    UART3_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
    NVIC_EnableIRQ(UART3_RX_TX_IRQn);    
}

/*
Adjust the desired straight speed by delta. 
Uses the clip macro to prevent excessive speed. 
*/
void StraightHelper(float delta){
	STRAIGHT_DESIRED+=delta;
    clip(STRAIGHT_DESIRED,0.0f,100.0f);
    SL=STRAIGHT_DESIRED-10.0f;
    clip(SL,0.0f,100.0f);
    SH=STRAIGHT_DESIRED+10.0f;
    clip(SH,0.0f,100.0f);
}

/*
Adjust the desired turning speed by delta. 
Uses the clip macro to prevent excessive speed. 
*/
void TurnHelper(float delta){
    TURN_DESIRED+=delta;
    clip(TURN_DESIRED,0.0f,100.0f);
    TL=TURN_DESIRED-10.0f;
    clip(TL,0.0f,100.0f);
    TH=TURN_DESIRED+10.0f;
    clip(TH,0.0f,100.0f);
    REV_BRAKE_DESIRED[0]=TURN_DESIRED-20.0f;
    clip(REV_BRAKE_DESIRED[0],0.0f,100.0f);
    REV_BRAKE_DESIRED[1]=TURN_DESIRED-30.0f;
    clip(REV_BRAKE_DESIRED[0],0.0f,100.0f);
    REV_BRAKE_DESIRED[2]=TURN_DESIRED-40.0f;
    clip(REV_BRAKE_DESIRED[0],0.0f,100.0f);
}



/*
Gets input from bluetooth.
Styles:
    0 == stops car
    1 == reset to testing speeds
    2 == reset to normal speeds
    3 == increase straight speeds by 5.0f
    4 == decrease straight speeds by 5.0f
    5 == increase turning speeds by 5.0f
    6 == decrease turning speeds by 5.0f
    7PX.XXIY.YYDZ.ZZ == KP=X.XX, KI=Y.YY, KD=Z.ZZ
*/
//TODO
//have it send stuff back? (Say what it did)
void UART3_RX_TX_IRQHandler(void){
    uint8_t temp;
    char str[100];
    int temp2=0;
    UART3_S1; //clears interrupt
    temp=UART3_D;
	temp2= temp;
	if(temp){
		sprintf(str, "Received info\n\r");
		uart_put(str);
	}
    
  return;
}
