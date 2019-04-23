#include "stdio.h"
#include <math.h>
#include <stdlib.h>
#include "MK64F12.h"
#include "uart.h"
#include "LEDS.h"
#include "stdio.h"
#include "camera.h"
#include "Bluetooth.h"
#include "string.h"

#define BAUD_RATE 9600      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)


extern int VERBOSE;
extern void delay(int);
extern android_data *data;

extern double distance;
extern double angle;
extern void spinToWin(void);


/*
   Send inerrupt to bluetooth, get gps from that
   Send interrupt to arduino, get gps from that
   Calc distance using hypotenous
   Calc angle 

TODO: Don't calc distance or angle, average it with camera and ultrasonic.
*/
void getCamera(){
    double phoneGPS[2]={0.0,0.0}; //GPS coords from phone
    char phoneChar[64];
    char distChar[64]={0};
    char angleChar[64]={0};
    const double __distance_offset = 6.0;

    bt_toggle_interrupts(0);
    
    
    LEDon(BLUE);
    for(;;){
        put("Camera sending\r\n");
        uart4_put(0);

        //Get distance and angle
        uart4_get_DistAngle(distChar, angleChar);
        if (distChar[0]==1){
            put("Was none\r\n");
            spinToWin();
            memset(distChar, 0, sizeof(distChar));
            memset(angleChar, 0, sizeof(angleChar));
            delay(100);
            continue;
        }
        break;
    }
    put("\r\n");
    put(distChar);
    put("\r\n");
    put(angleChar);
    put("\r\n");
    sscanf(distChar, "%lf", &distance);
    sscanf(angleChar, "%lf", &angle);  
    
    bt_toggle_interrupts(1);
    //TODO spin if not found
        //Do this on pi?
    
    //distance = distance - __distance_offset;
    if (distance < 0.0) {distance=0.0;}
    
    
    LEDon(GREEN);
}

void uart4_init(void){
    //define variables for baud rate and baud rate fine adjust
    uint16_t ubd, brfa;

    //Enable clock for UART
    SIM_SCGC1 |= SIM_SCGC1_UART4_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

    //Configure the port control register to alternative 3 (which is UART mode for K64)
    PORTE_PCR24 |= PORT_PCR_MUX(3);
    PORTE_PCR25 |= PORT_PCR_MUX(3);

    /*Configure the UART for establishing serial communication*/


    //Disable transmitter and receiver until proper settings are chosen for the UART module
    UART4_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    //UART0_C2 &= !(UART_C2_RE_MASK);

    //Select default transmission/reception settings for serial communication of UART by clearing the control register 1
    UART4_C1 = 0;

    //UART Baud rate is calculated by: baud rate = UART module clock / (16 × (SBR[12:0] + BRFD))
    //13 bits of SBR are shared by the 8 bits of UART3_BDL and the lower 5 bits of UART3_BDH 
    //BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
    //BRFA is defined by the lower 4 bits of control register, UART0_C4 

    //calculate baud rate settings: ubd = UART module clock/16* baud rate
    ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));  

    //clear SBR bits of BDH
    UART4_BDH &= ~UART_BDH_SBR_MASK;

    //UART4_BDH |= UART_BDH_RXEDGIE_MASK;
    //UART0_BDH |= UART_BDH_RXEDGIE_MASK;
    //distribute this ubd in BDH and BDL
    UART4_BDH = (((ubd & 0x1F00) >> 8));

    //UART0_BDH = ((ubd >> 8) | UART_BDH_SBR_MASK);
    //UART0_BDL = ubd & UART_BDL_SBR_MASK;
    UART4_BDL = (uint8_t)(ubd & UART_BDL_SBR_MASK);


    //BRFD = (1/32)*BRFA 
    //make the baud rate closer to the desired value by using BRFA
    brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

    //write the value of brfa in UART0_C4
    UART4_C4 &= ~(UART_C4_BRFA_MASK);
    UART4_C4 |= UART_C4_BRFA(brfa);

    //UART0_C4 |= (brfa | UART_C4_BRFA_MASK);

    //Enable transmitter and receiver of UART
    UART4_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);

}

uint8_t uart4_getchar(){
    /* Wait until there is space for more data in the receiver buffer*/
    while(!(UART4_S1 & UART_S1_RDRF_MASK));
    return UART4_D;
    /* Return the 8-bit data from the receiver */
}

void uart4_putchar(char ch)
{
    /* Wait until transmission of previous bit is complete */
    //wait until 1
    while(!(UART4_S1 & UART_S1_TDRE_MASK));// != UART_S1_TDRE_MASK){}
    /* Send the character */
    UART4_D = (uint8_t)ch;
}

void uart4_put(char *ptr_str){
    /*use putchar to print string*/
    while(*ptr_str){
        //uart_putchar(*ptr_str);
        uart4_putchar(*ptr_str++);

    }
}

void uart4_get(char *ptr_str){
    int lcv;
    uint8_t cu;
    lcv=0;
    while(lcv < 254){
        cu = uart4_getchar();
        if(cu == 0){ //if entered character is character return
            return;
        }
        ptr_str[lcv] = cu;
        lcv++;
    }
    return;
}

void uart4_get_DistAngle(char *ptr_str_dist, char *ptr_str_angle){
    /*
       Used instead of two instances of uart4_get because the k64 
       is slower than the arduino and will miss the second message.
       */
    int lcv;
    uint8_t cu;
    lcv=0;
    while(lcv < 254){
        cu = uart4_getchar();
        if(cu == 0){ //if entered character is character return
            break;
        }
        ptr_str_dist[lcv] = cu;
        lcv++;
    }
    lcv=0;
    while(lcv < 254){
        cu = uart4_getchar();
        if(cu == 0){ //if entered character is character return
            break;
        }
        ptr_str_angle[lcv] = cu;
        lcv++;
    }
    return;
}

