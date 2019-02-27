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
 *  ADC0_DP1(bottom left)     - camera AOut   J2-4
 *  3.3V      - camera Vdd
 *    PTC3    A1  J1-5    reverse signal
 *    PTC4    A2  J1-7    pwm signal
 *    PTA2    B1  J10-12  pwm signal
 *    PTA1    B2  J10-10  reverse signal
 *    PTD2    Servo J10-2
 *  PTB10 UART3_RX (Blue)
 *  PTB11 UART3_TX (Red)
 *
 * Author:  Richard Cloutier
 * Created:  11/20/17
 */

#include "MK64F12.h"
#include "uart.h"
#include "stdio.h"
#include "PWM.h"
#include "inits.h"
#include <math.h>
#include "isr.h"
#include "Bluetooth.h"
#include "LEDS.h"
#include "ultrasonic.h"

void initialize(void);
void en_interrupts();
void delay(int del);
void printBinline(void);
void printLine(void);
void turnLeft(int index);
void turnRight(int index);
float calc(const float currentPWM, const float desiredPWM, const int wheel);
void detectFinish(int maximumVal);
void normalSet(void);
void turnCalc(void);
void distanceCalc(void);
void demo(void);


const int FORWARD=0;
const int REVERSE=1;
const int STRAIGHT = 1;
const int TURNING = 0;

const int DC_freq = 10000;
const int BLUETOOTH=1;
const int LEFT=0;
const int RIGHT=1;
const int MIDDLE=70;
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
#define MAX_BUF_SZ 128

// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
//        ADC reads start
int pixcnt = -2;
// clkval toggles with each FTM interrupt
int clkval = 0;
// data count
int dataCount_buf = 0;
int dataCount_line = 0;
// stores the current array of camera data
uint16_t line[MAX_BUF_SZ];
uint16_t line2[MAX_BUF_SZ];
// ptr to the buffer array of camera data
uint16_t* bufferPtr=line;
// ptr to the current array of camera data
uint16_t* linePtr=NULL;

// These variables are for streaming the camera
//     data over UART
int debugcamdata = 1;
int capcnt = 0;
char str[100];
// ADC0VAL holds the current ADC value
uint16_t ADC0VAL;

int binline[128]; //binary smoothed line 

//Set Kp,Ki,Kd values in normalSet function
float KP; //60
float KI; //70
float KD; //25

float PWMErrOld1[2] = {0.0,0.0}; //e(n-1)
float PWMErrOld2[2] = {0.0,0.0}; //e(n-2)

float LB=0.0; //Lower bound of wheel speed
float UB=70.0; //Upper bound of wheel speed

//Desired PWM of left and right wheels
float LEFT_DESIRED=0.0;
float RIGHT_DESIRED=0.0;

//PWM of left/right wheels
float LPWM=0.0; //PWM of left wheel
float RPWM=0.0; //PWM of right wheel

//Manual PWM delta. Controlled by bluetooth to manually boost/retard motors
float manualDelta[2] = {0.0,0.0};

int ready=0;

extern float ultrasonic_distance;
extern int ultrasonic_ready;


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
    //Run demo
    int demov=1;
    // Initialize everything
    initialize();
    
    // Print welcome over serial
    put("Running... \n\r");
    
    normalSet();
    SetDutyCycle(0,DC_freq,FORWARD);
    for(;;){
        while(!ready){
            LEDon(RED);
            delay(5);
            LEDoff();
            delay(5);
        }
        
        //Demo code
        if (demov==1){
            demo();
            
        }
        
        //Main code
        for(;;){        
            //distance calc
            distanceCalc();
            
            //turn calc
            turnCalc();
            
            //set duty cycles
            LPWM=calc(LPWM, LEFT_DESIRED, LEFT);
            RPWM=calc(RPWM, RIGHT_DESIRED, RIGHT);
            RightDuty((int)RPWM,DC_freq,FORWARD);
            LeftDuty((int)LPWM,DC_freq,FORWARD);
                
            delay(1);
      }
    }
    return 0;
}

/*
Demo code
*/
void demo(void){
    int demoi;
    int demoj;
    for (demoi=0; demoi<=1; demoi++){
        for (demoj=1; demoj<=10; demoj++){
            if (demoi==0){//left
                LeftDuty(10*demoj,DC_freq,FORWARD);
            }else{//right
                LeftDuty(0,DC_freq,FORWARD);
                RightDuty(10*demoj,DC_freq,FORWARD);
            }
            delay(300);
        }
    }
    for(;;){
        LPWM=calc(LPWM, 0.0, LEFT);
        LeftDuty((int)LPWM,DC_freq,FORWARD);
        RPWM=calc(RPWM, 0.0, RIGHT);
        RightDuty((int)RPWM,DC_freq,FORWARD);
    }
}



/*
Reset all vars to nominal/safe values
*/
void normalSet(void){
    LEFT_DESIRED=0.0f;//75.0f; 
    LEFT_DESIRED=0.0f;//80.0f
    KP=0.60f; //0.45
    KI=0.70f; //0.63
    KD=0.25f; //0.15
    manualDelta[0]=0.0f;
    manualDelta[1]=0.0f;
}

/* 
Calculates distance between user and cart.
Adjusts desired speed accordingly
*/
void distanceCalc(void){
    float distance = 0.0f;
    const float maxDistance=100.0f; //Max distance == max speed 
    float desiredSpeed = 0.0f;
    
    //TODO distance calculations
    //GPS = long
    //Camera = med
    //Ultrasonic = med/short 
    
    //linear calc
    desiredSpeed=(UB*distance)/maxDistance;
    clip(desiredSpeed,LB,UB);
    LEFT_DESIRED=desiredSpeed;
    RIGHT_DESIRED=desiredSpeed;
}

/*
Calculates angle between user and current path of cart. 
Angle range is [-90,90], with 0 being directly in front
*/
void turnCalc(void){
    float angle = 0.0f;
    const int minAngle = 5.0f;
    
    //TODO angle calculations
    
    if (abs(angle) > minAngle) {
        /*
        degree = -90 to 90
        Slows inner wheel.
        Max slow speed == 0
        Slow speed dictated by speed of other wheel
        */
        if(angle > 0.0f){ //Right turn
            LEDon(RED);
            RIGHT_DESIRED = (LEFT_DESIRED*angle)/90.0f;
            clip(RIGHT_DESIRED,LB,UB);
        }else if(angle < 0.0f){ //Left
            LEDon(BLUE);
            LEFT_DESIRED = (RIGHT_DESIRED*angle)/90.0f;
            clip(LEFT_DESIRED,LB,UB);
        }
    }else{
        LEDon(GREEN);
    }
}

/**
 * Waits for a delay (in milliseconds)
 * 
 * del - The delay in milliseconds
 */
void delay(int del){
    int i;
    for (i=0; i<del*50000; i++){
        // Do nothing
    }
}

//Prints what the camera is seeing
void printLine(void){
    char str[100];
        if (debugcamdata) {
            // Every 2 seconds
            //if (capcnt >= (2/INTEGRATION_TIME)) {
            if (capcnt >= (500)) {
                // send the array over uart
                sprintf(str,"%i\n\r",-1); // start value
                put(str);
                for (int i = 0; i < 127; i++) {
                    sprintf(str,"%i\n", linePtr[i]);
                    put(str);
                }
                sprintf(str,"%i\n\r",-2); // end value
                put(str);
                capcnt = 0;
            }
        }
}

/*
Calculate the PID PWM value based on the current PWM value,
the desired PWM value, and if the car is going straight or turning.
    Wheel=0: Left wheel
    Wheel=1: Right wheel
*/
float calc(const float currentPWM, const float desiredPWM, const int wheel){
    float err = desiredPWM - currentPWM; //e(n)
    float newPWM = currentPWM +\
    (KP * (err-PWMErrOld1[wheel])) + \
    (KI * ((err+PWMErrOld1[wheel])/2.0f)) + \
    (KD * (err - (2.0f * PWMErrOld1[wheel]) + PWMErrOld2[wheel])) + \
    manualDelta[wheel]; 
    
    clip(newPWM,LB,UB);
    
    PWMErrOld2[wheel]=PWMErrOld1[wheel];
    PWMErrOld1[wheel]=err;
    return newPWM;
}

/*
Initializes all required modules
*/
void initialize(void){
    // Initialize UART
    uart_init();    
    
    // Initialize the FlexTimer
    InitPWM();
    
    init_GPIO(); // For CLK and SI output on GPIO
    //init_FTM2(); // To generate CLK, SI, and trigger ADC
    //init_ADC0();
    init_LEDS();
    if(BLUETOOTH){init_BT();} //Initialize the bluetooth controller
    init_ultrasonic();
}
