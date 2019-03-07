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

#define clip(n, lower, upper) {\  //Need?
    if(n < lower) {\
        n = lower;\
    } else if(n > upper) {\
        n = upper;\
    }\
}

extern void normalSet(void);w
extern int[10] control;

extern float LB;
extern float UB;
extern float LEFT_DESIRED;
extern float RIGHT_DESIRED;

extern int ready;
extern float manualDelta[2];
int RX_lcv = 0; //Current index of UART_TX_STR

extern float KP;
extern float KI;
extern float KD;

int controlIndex = -1;

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
Gets information input from Android device via bluetooth.
  Protocol:
    [Input Char][Input value]
  Known Inputs:
    SXXX == Speed = XXX, 0 <= XXX <= 100
    TX.XX == Turn = X.XX, -30 <= X.XX <= 30
    AxX.XXAyY.YYAzZ.ZZ = (Accelerometer) x = X.XX, y = Y.YY, z = Z.ZZ
    GxX.XXXXGyY.YYYY = (GPS Location) x = X.XXXX, y = Y.YYYY
 
*/
void UART3_RX_IRQHandler(void) {
  uint8_t val;
  char name;
  if(UART3_D >= '0' && UART3_D <= '9') {
    val = UART3_D;
  } else {
    name = UART3_D;
  }
  if (controlIndex == -1) {
    switch (name) {
      case 'S':
        controlIndex = 1;
        break;
      case 'T':
        controlIndex = 2;
        break;
      case 'Ax':
        controlIndex = 3;
        break;
      case 'Ay':
        controlIndex = 4;
        break;
      case 'Az':
        controlIndex = 5;
        break;
      case 'Gx':
        controlIndex = 6;
        break;
      case 'Gy':
        controlIndex = 7;
        break;
      default:
        controlIndex = -1;
        break;
    }
  } else {
    control[controlIndex] = val;
    controlIndex = -1;
  }
}
  
void UART3_TX_IRQHandler(void) {
  return;
}
  
void sendFloatTx() {
    
}

  
void pollGPSRx() {
    
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
    int temp2 = 0;
    UART3_S1; //clears interrupt
    temp = UART3_D;
    temp2 = temp;
    LEDon(YELLOW);
    if(temp2 >= 0 && temp2 <= 14){
        if(temp2 == 0){//stop car
            LEFT_DESIRED = 0.0f;
            RIGHT_DESIRED = 0.0f;
            manualDelta[0] = 0.0f;
            manualDelta[1] = 0.0f;
        }else if(temp2 == 1){//testing speed
            KP = 0.45;
            KI = 0.15;
            KD = 0.25;
            RIGHT_DESIRED = 25.0f;
            LEFT_DESIRED = 25.0f;
        }else if(temp2 == 2){ //normal speed
            normalSet();
        }else if(temp2 == 3){//+5 straight
            manualDelta[0] += 5.0f;
            manualDelta[1] += 5.0f;
        }else if(temp2 == 4){//-5 straight
            manualDelta[0] -= 5.0f;
            manualDelta[1] -= 5.0f;
        }else if(temp2 == 5){//+5 turn left
            manualDelta[0] += 5.0f;
        }else if(temp2 == 6){//-5 turn left
            manualDelta[0] -= 5.0f;
        }else if(temp2 == 7){
            ready = 1;
        }else if(temp2 == 8){
            KP -= 0.05f;
        }else if(temp2 == 9){
            KP += 0.05f;
        }else if(temp2 == 10){
            KI -= 0.05f;
        }else if(temp2 == 11){
            KI += 0.05f;
        }else if(temp2 == 12){
            KD -= 0.05f;
        }else if(temp2 == 13){
            KD += 0.05f;
        }else if(temp2 == 14){//+5 turn right
            manualDelta[1] += 5.0f;
        }else if(temp2 == 15) {//-5 turn right
            manualDelta[1] -= 5.0f;
        }else if(temp2 == 7) {
            ready = 1;
        }
        return;
            
    }
    
    return;
}
