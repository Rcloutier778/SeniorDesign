/*
 * Freescale Cup linescan camera code
 *
 *    This method of capturing data from the line
 *    scan cameras uses a flex timer module, periodic
 *    interrupt timer, an ADC, and some GPIOs.
 *    CLK and SI are driven with GPIO because the FTM2
 *    module used doesn't have any output pins on the
 *     development board. The PIT timer is used to
 *  control the integration period. When it overflows
 *     it enables interrupts from the FTM2 module and then
 *    the FTM2 and ADC are active for 128 clock cycles to
 *    generate the camera signals and read the camera
 *  output.
 *
 *    PTB9            - camera CLK    J2-20
 *    PTB23         - camera SI     J2-19
 *    ADC0_DP1(bottom left)     - camera AOut   J2-4
 *    PTC3    reverse signal  Left
 *    PTC4    reverse signal  Right
 *    PTA1    pwm signal  Left
 *    PTA2    pwm signal  Right
 *    PTB11   UART3_RX (Red)
 *    PTB10   UART3_TX (Blue)
 *    PTD0    Ultrasonic echo
 *    PTD1    Ultrasonic trigger
 PTD2    UART2_RX  arduino
 PTD3    UART2_TX  arduino
 *
 * Author:  Richard Cloutier
 * Created:  11/20/17
 */

#include "MK64F12.h"
#include "uart.h"
#include "stdio.h"
#include "pwm.h"
#include "inits.h"
#include <math.h>
#include "isr.h"
#include "Bluetooth.h"
#include "LEDS.h"
#include "ultrasonic.h"
#include "GPS.h"
#include <stdlib.h>

void initialize(void);
void en_interrupts(void);
void delay(int del);
void printLine(void);
float calc(const float currentPWM, const float desiredPWM, const int wheel);
void normalSet(void);
void turnCalc(void);
void distanceCalc(void);
void demo(void);
void turn(int angle);
void waitForReady(void);

const int DC_freq = 25000;
const int BLUETOOTH=1;
const int LEFT=0;
const int RIGHT=1;
const int MIDDLE=70;

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u

// default = .0075f
#define INTEGRATION_TIME .0075f
#define MAX_BUF_SZ 128

//Android data - free at termination
android_data *data = NULL;

//Set Kp,Ki,Kd values in normalSet function
float KP; //60
float KI; //70
float KD; //25

float PWMErrOld1[2] = {0.0,0.0}; //e(n-1)
float PWMErrOld2[2] = {0.0,0.0}; //e(n-2)

float LB= -30.0; //Lower bound of wheel speed
float UB=  50.0; //Upper bound of wheel speed

//Desired PWM of left and right wheels
float LEFT_DESIRED=0.0;
float RIGHT_DESIRED=0.0;

//PWM of left/right wheels
float LPWM=0.0; //PWM of left wheel
float RPWM=0.0; //PWM of right wheel

//Manual PWM delta. Controlled by bluetooth to manually boost/retard motors
float manualDelta[2] = {0.0,0.0};
int manualControl=0;

int ready=0;
double angle = 0;
double distance=0;

//[0,359], 0==North, 90==East, 180==South, 270==West
int direction=0;

//latitude, longitude
float location[2]={0.0f,0.0f};

int control[10];

int VERBOSE=0; //Print to uart0 (terminal)


/*
 Limit n to the lower and upper bounds only.
 */
#define clip(n, lower, upper){\
if(n<lower){\
n=lower;\
}else if(n>upper){\
n=upper;\
}\
}


int main(void){
  char c[254]={0};
  //Run demo
  int demov=0;
  int gps_demov=1;
  
  // Initialize everything
  initialize();
  
  // Print welcome over serial
  put("Running... \n\r");
  
  normalSet();
  SetDutyCycle(0,DC_freq);
  
  for(;;){
	if (gps_demov==0){
    waitForReady();
    }
	
    //Demo code
    if (demov==1){
      demo();
    }
    if(gps_demov==1){
        gpsDemo();
    }
    //Main code
    for(;;){
      if(!ready){
		
        waitForReady();
      }
      
      //distance calc
      distanceCalc();
      
      //turn calc
      turnCalc();
      
      if (VERBOSE){
        sprintf(c,"Angle: %g",angle);
        put(c);
        put("\r\n");
      }
      
      //set duty cycles
      LPWM=calc(LPWM, LEFT_DESIRED, LEFT);
      RPWM=calc(RPWM, RIGHT_DESIRED, RIGHT);
      
      if (VERBOSE){
        sprintf(c,"PWM: %g",LPWM);
        put(c);
        sprintf(c,"   %g",RPWM);
        put(c);
        put("\r\n");
      }
      
      
      LeftDuty((int)LPWM,DC_freq);
      RightDuty((int)RPWM,DC_freq);
      
      delay(100);
    }
  }
}



/*
 Demo code
 */
void demo(void){
    int demoi;
    int demoj;
    char test;
    char k64[255];
    char from_ard[255];
    double phoneGPS[2]={0.0,0.0}; //GPS coords from phone
    char phoneChar[64];
    char angleChar[64];
    char distChar[64];
    put("In demo\r\n");

    
      
      /*
       for(;;){
       distanceCalc();
       LPWM=calc(LPWM, LEFT_DESIRED, LEFT);
       RPWM=calc(RPWM, RIGHT_DESIRED, RIGHT);
       
       sprintf(c,"LPWM: %g",LPWM);
       put(c);
       put("\r\n");
       sprintf(c,"RPWM: %g",RPWM);
       put(c);
       put("\r\n");
       
       LeftDuty((int)LPWM,DC_freq);
       RightDuty((int)RPWM,DC_freq);
       delay(100);
       }
       
       //Stop
       LEDon(WHITE);
       for(;;){
       LPWM=calc(LPWM, 0.0, LEFT);
       LeftDuty((int)LPWM,DC_freq);
       RPWM=calc(RPWM, 0.0, RIGHT);
       RightDuty((int)RPWM,DC_freq);
       }
       */
  
}

/*
 Calculates distance between user and cart.
 Adjusts desired speed accordingly
 */
void distanceCalc(void){
  double distance = 0.0; //GPS, camera, ultrasonic
  const float maxDistance=50.0f; //Max distance == max speed
  const float minDistance=15.0f;
  const int distRange[2] = {10, 3}; //Minimum distance ranges for distance calc methods
  float desiredSpeed = 0.0f;
  char  c[255];
  
  if(manualControl){
    LEFT_DESIRED=manualDelta[0];
    RIGHT_DESIRED=manualDelta[1];
    return;
  }
  
  //GPS
  getGPS(&distance, &angle);
  if (distance < distRange[0]){
    //Camera call here
    
    if (distance < distRange[0]){
      //Ultrasonic
      distance=getUltrasonic();
    }
  }
  
  if(VERBOSE){
    sprintf(c,"Distance: %g inches",distance);
    put(c);
    put("\r\n");
  }
  //linear calc
  if(distance < minDistance){
    desiredSpeed=0.0f;
  }else{
    desiredSpeed=6.0f+((UB*distance)/maxDistance);
    clip(desiredSpeed,0.0f,UB); //TODO LB=-UB?
  }
  LEFT_DESIRED=desiredSpeed;
  RIGHT_DESIRED=desiredSpeed;
}


/*
 Calculates angle between user and current path of cart.
 Angle range is [0,359], with 0 being directly in front
 */
void turnCalc(void){
  const int minAngle = 5;
  
  if(manualControl){
    turn(angle);
    return;
  }
  
  //TODO angle calculations for camera
  
  if (abs(angle) > minAngle) {
    /*
     Slows inner wheel. Then drive in reverse
     Max slow speed == -UB
     Slow speed dictated by speed of other wheel
     */
    if(angle>180){
      angle=angle-360;
    }
    turn(angle);
  }else{
    LEDon(GREEN);
  }
}

/*
 Turn the cart
 angle in range [-180,180], 0 being directly in front.
 */
void turn(int angle){
  //Angle at which the inner wheel will be set to negative pwm of outer wheel
  const int maxAngle=180;
  //Angle at which reverse braking will start. Inner wheel pwm == 0 at this point.
  const int revBrakeAngle=90;
  
  if(angle > 0){ //Right turn
    if (angle <= revBrakeAngle){
      RIGHT_DESIRED = (LEFT_DESIRED*(revBrakeAngle-angle))/revBrakeAngle;
    }else{ //reverse braking
      RIGHT_DESIRED = -LEFT_DESIRED*(angle-revBrakeAngle)/(maxAngle-revBrakeAngle);
    }
    clip(RIGHT_DESIRED,LB,UB);
  }else if(angle < 0){ //Left
    if (abs(angle) <= revBrakeAngle){
      LEFT_DESIRED = (RIGHT_DESIRED*(revBrakeAngle-abs(angle)))/revBrakeAngle;
    }else{ //reverse braking
      LEFT_DESIRED = -RIGHT_DESIRED*(abs(angle)-revBrakeAngle)/(maxAngle-revBrakeAngle);
    }
    clip(LEFT_DESIRED,LB,UB);
  }
}

/**
 * Waits for a delay (in milliseconds)
 *
 * del - The delay in milliseconds
 */
void delay(int del){
  int i;
  for (i=0; i<del*(DEFAULT_SYSTEM_CLOCK/1000); i++){
    // Do nothing
  }
}

/*
 Reset all vars to nominal/safe values
 */
void normalSet(void){
  LEFT_DESIRED=0.0f;//75.0f;
  RIGHT_DESIRED=0.0f;//80.0f
  KP=0.45f; //0.45
  KI=0.0f; //0.63
  KD=0.0f; //0.15
  manualDelta[0]=0.0f;
  manualDelta[1]=0.0f;
}

/*
 Calculate the PID PWM value based on the current PWM value,
 the desired PWM value, and if the car is going straight or turning.
 Wheel=0: Left wheel
 Wheel=1: Right wheel
 */
float calc(const float currentPWM, const float desiredPWM, const int wheel){
  float err = desiredPWM - currentPWM; //e(n)
  /*
   float newPWM = currentPWM +\
   (KP * (err-PWMErrOld1[wheel])) + \
   (KI * ((err+PWMErrOld1[wheel])/2.0f)) + \
   (KD * (err - (2.0f * PWMErrOld1[wheel]) + PWMErrOld2[wheel])) + \
   manualDelta[wheel];
   */
  
  float newPWM=currentPWM+(KP*err);
  clip(newPWM,LB,UB);
  
  PWMErrOld2[wheel]=PWMErrOld1[wheel];
  PWMErrOld1[wheel]=err;
  return newPWM;
}

/*
 Wait for the ready flag to be set.
 Flash the red led until ready is set, then count down with yellow led
 */
void waitForReady(void){
  int demoi;
  while(!ready){
    if(LPWM!=0.0f || RPWM!=0.0f){
      LPWM=calc(LPWM, 0.0f, LEFT);
      RPWM=calc(RPWM, 0.0f, RIGHT);
      LeftDuty((int)LPWM,DC_freq);
      RightDuty((int)RPWM,DC_freq);
    }
    LEDon(RED);
    delay(10);
    LEDoff();
    delay(10);
  }
  for(demoi=3; demoi>0; demoi--){
    LEDon(YELLOW);
    delay(50*demoi);
    LEDoff();
    delay(50*demoi);
  }
  LEDon(GREEN);
  
}

/*
 Initializes all required modules
 */
void initialize(void){
  // Initialize UART
  uart0_init();    //Terminal
  uart2_init();   //arduino comm
  
  // Initialize the FlexTimer
  InitPWM();
  
  init_GPIO(); // For CLK and SI output on GPIO
  init_LEDS();
  if(BLUETOOTH) {init_BT();} //Initialize the bluetooth controller
  init_ultrasonic();
}
