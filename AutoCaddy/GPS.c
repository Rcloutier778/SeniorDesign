#include "stdio.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "MK64F12.h"
#include "uart.h"
#include "LEDS.h"
#include "stdio.h"
#include "GPS.h"
#include "Bluetooth.h"

#define BAUD_RATE 9600      //default baud rate
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)


extern int VERBOSE;
extern void delay(int);
extern android_data *data;

extern double distance;
extern double angle;

const int TIMEOUT = 10000; // 10s
clock_t timerStart;


/*
 Send inerrupt to bluetooth, get gps from that
 Send interrupt to arduino, get gps from that
 Calc distance using hypotenous
 Calc angle
 TODO: Don't calc distance or angle, average it with camera and ultrasonic.
 */
void getGPS(){
  double phoneGPS[2]={0.0,0.0}; //GPS coords from phone
  char phoneChar[64];
  char distChar[64];
  char angleChar[64];
  const double __distance_offset = 6.0;
  
  bt_toggle_interrupts(0);
  //Get GPS from Bluetooth  -- brian //TODO
  //get_BT_GPS_dev(&phoneGPS[0],&phoneGPS[1]);
  if (data->avggpsx == 0.0f){//Init values
    //home   43.136269;    -77.750473;
    //school   43.084514;    -77.678525;
    data->avggpsx = 43.136269;
    data->avggpsy = -77.750473;
  }
  phoneGPS[0]=data->avggpsx;
  phoneGPS[1]=data->avggpsy;
  
  LEDon(WHITE);
  
  //Send (XX.XXX,YY.YYY) to arduino
  snprintf(phoneChar,sizeof phoneChar, "%lf", phoneGPS[0]);
  
  uart2_put(phoneChar);
  
  if (VERBOSE==2){
    put("User lat: ");
    put(phoneChar); //TODO
    put("\r\n");
  }
  uart2_putchar(',');
  
  snprintf(phoneChar,sizeof phoneChar, "%lf", phoneGPS[1]);
  
  uart2_put(phoneChar);
  
  if (VERBOSE==2){
    put("User long: ");
    put(phoneChar); //TODO
    put("\r\n");
  }
  
  uart2_putchar(0);
  
  //Get distance and angle
  uart2_get_DistAngle(distChar, angleChar);
  
  sscanf(distChar, "%lf", &distance);
  sscanf(angleChar, "%lf", &angle);
  bt_toggle_interrupts(1);
  
  distance = distance - __distance_offset;
  if (distance < 0.0) {distance=0.0;}
  
  
  LEDon(GREEN);
}

void gpsDemo(void){
  int demoi;
  int demoj;
  char test;
  char k64[255];
  char from_ard[255];
  double phoneGPS[2]={0.0,0.0}; //GPS coords from phone
  char phoneChar[64];
  char angleChar[64];
  char distChar[64];
  float tDistance;
  float tAngle;
  put("In demo\r\n");
  
  for(;;){
    phoneGPS[0]=45.123456;
    phoneGPS[1]=-77.123456;
    
    
    for(;;){
      phoneGPS[0]=data->gpsx;
      phoneGPS[1]=data->gpsy;
      snprintf(phoneChar,sizeof phoneChar, "%g", phoneGPS[0]);
      put(phoneChar);
      put(",");
      snprintf(phoneChar,sizeof phoneChar, "%g", phoneGPS[1]);
      put(phoneChar);
      put("\r\n");
      
      
      snprintf(phoneChar,sizeof phoneChar, "%i", data->speed);
      put(phoneChar);
      put("\r\n");
      
      delay(500);
    }
    
    /*
     uart_putchar( bt_getbyte());
     uart_putchar( bt_getbyte());
     uart_putchar( bt_getbyte());
     uart_putchar( bt_getbyte());
     uart_putchar( bt_getbyte());
     uart_putchar( bt_getbyte());
     uart_putchar( bt_getbyte());
     uart_putchar( bt_getbyte());
     bt_getData();
     */
    
    
    
    //Send (XX.XXX,YY.YYY) to arduino
    snprintf(phoneChar,sizeof phoneChar, "%g", phoneGPS[0]);
    if(phoneGPS[0] > 0){
      uart2_putchar('+');
    }
    uart2_put(phoneChar);
    put(phoneChar);
    uart2_putchar(',');
    put(",");
    snprintf(phoneChar,sizeof phoneChar, "%g", phoneGPS[1]);
    if(phoneGPS[1] > 0){
      uart2_putchar('+');
    }
    uart2_put(phoneChar);
    put(phoneChar);
    put("\r\n");
    uart2_putchar('\n');
    
    //Get distance and angle
    uart2_get_DistAngle(distChar, angleChar);
    //uart2_get(distChar);
    tDistance = atof(distChar);
    //uart2_get(angleChar);
    tAngle = atof(angleChar);
    
    
    put("Got distance of: ");
    put(distChar);
    put("\r\n");
    put("Got angle of: ");
    put(angleChar);
    put("\r\n");
    
    delay(1000);
    
  }
}



void uart2_init(void){
//define variables for baud rate and baud rate fine adjust
uint16_t ubd, brfa;

//Enable clock for UART
  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
  SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

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

uint8_t uart2_getchar(int timeout){
  /* Wait until there is space for more data in the receiver buffer*/
  if (timeout) {
    timerStart = clock();
    do {
      clock_t diff = clock() - timerStart;
      if (diff * 1000 / DEFAULT_SYSTEM_CLOCK >= TIMEOUT) return 0;
    } while (!(UART2_S1 & UART_S1_RDRF_MASK));
  } else {
    while(!(UART2_S1 & UART_S1_RDRF_MASK));
  }
  
  return UART2_D;
  /* Return the 8-bit data from the receiver */
}

void uart2_putchar(char ch)
{
  /* Wait until transmission of previous bit is complete */
  //wait until 1
  while(!(UART2_S1 & UART_S1_TDRE_MASK));// != UART_S1_TDRE_MASK){}
  /* Send the character */
  UART2_D = (uint8_t)ch;
}

void uart2_put(char *ptr_str){
  /*use putchar to print string*/
  while(*ptr_str){
    //uart_putchar(*ptr_str);
    uart2_putchar(*ptr_str++);
    
  }
}

void uart2_get(char *ptr_str){
  int lcv;
  uint8_t cu;
  lcv=0;
  while(lcv < 254){
    cu = uart2_getchar(0);
    if(cu == 0){ //if entered character is character return
      return;
    }
    //    uart_putchar(cu);
    ptr_str[lcv] = cu;
    lcv++;
  }
  return;
}

void uart2_get_DistAngle(char *ptr_str_dist, char *ptr_str_angle){
  /*
   Used instead of two instances of uart2_get because the k64
   is slower than the arduino and will miss the second message.
   */
  int lcv;
  uint8_t cu;
  lcv=0;
  while(lcv < 254){
    cu = uart2_getchar(TIMEOUT);
    if(cu == 0){ //if entered character is character return
      break;
    }
    //    uart_putchar(cu);
    ptr_str_dist[lcv] = cu;
    lcv++;
  }
  lcv=0;
  while(lcv < 254){
    cu = uart2_getchar(0);
    if(cu == 0){ //if entered character is character return
      break;
    }
    //    uart_putchar(cu);
    ptr_str_angle[lcv] = cu;
    lcv++;
  }
  return;
}


