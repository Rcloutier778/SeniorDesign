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
 *    PTC3    reverse signal  Left
 *    PTC4    reverse signal  Right
 *    PTA1    pwm signal  Left
 *    PTA2    pwm signal  Right
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
#include "i2c.h"

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
void getGPS(void);
void turn(int angle);


const int FORWARD=0;
const int REVERSE=1;
const int STRAIGHT = 1;
const int TURNING = 0;

const int DC_freq = 25000;
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

float LB= -70.0; //Lower bound of wheel speed
float UB=  70.0; //Upper bound of wheel speed

//Desired PWM of left and right wheels
float LEFT_DESIRED=0.0;
float RIGHT_DESIRED=0.0;

//PWM of left/right wheels
float LPWM=0.0; //PWM of left wheel
float RPWM=0.0; //PWM of right wheel

//Manual PWM delta. Controlled by bluetooth to manually boost/retard motors
float manualDelta[2] = {0.0,0.0};

int ready=0;


//[0,359], 0==North, 90==East, 180==South, 270==West
int direction=0;

//latitude, longitude
float location[2]={0.0f,0.0f};


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
    char c[50];
    //Run demo
    int demov=1;
    float __test_remove__=0.0f;
    // Initialize everything
    initialize();
    
    // Print welcome over serial
    put("Running... \n\r");
    
    normalSet();
    SetDutyCycle(0,DC_freq);
    /*
    for(;;){
        __test_remove__=getUltrasonic();
        sprintf(c,"%g",__test_remove__);
        put(c);
        put("\r\n");
        delay(300);
        
    }
    */
    

    for(;;){
        while(!ready){
            LEDon(RED);
            delay(5);
            LEDoff();
            delay(5);
        }
        LEDon(GREEN);
        
        //Demo code
        if (demov==1){
            demo();
            
        }
        
        //Main code
        for(;;){        
        
            getGPS();
        
            //distance calc
            distanceCalc();
            
            //turn calc
            turnCalc();
            
            //set duty cycles
            LPWM=calc(LPWM, LEFT_DESIRED, LEFT);
            RPWM=calc(RPWM, RIGHT_DESIRED, RIGHT);
            LeftDuty((int)LPWM,DC_freq);
            RightDuty((int)RPWM,DC_freq);
                
            delay(1);
      }
    }
    return 0;
}

void getGPS(void){
    
    
}

/*
Demo code
Used to spool up motors
*/
void demo(void){
    int demoi;
    int demoj;
    
    for(demoj=0; demoj<10; demoj++){

        LEFT_DESIRED=30.0f;
        RIGHT_DESIRED=30.0f;
        for (demoi=0; demoi<=5; demoi++){
            LPWM=calc(LPWM, LEFT_DESIRED, LEFT);
            RPWM=calc(RPWM, RIGHT_DESIRED, RIGHT);
            LeftDuty((int)LPWM,DC_freq);
            RightDuty((int)RPWM,DC_freq);
            delay(30);
        }
        for (demoi=0; demoi<=6; demoi++){
            turn(180-(demoi));
            LPWM=calc(LPWM, LEFT_DESIRED, LEFT);
            RPWM=calc(RPWM, RIGHT_DESIRED, RIGHT);
            LeftDuty((int)LPWM,DC_freq);
            RightDuty((int)RPWM,DC_freq);
            delay(50);
        }
        //Motor spool up
        /*
        LEDon(GREEN);
        for (demoi=0; demoi<=1; demoi++){
            for (demoj=1; demoj<=4; demoj++){
                if (demoi==0){//left
                    LeftDuty( 10*demoj,DC_freq,FORWARD);
                }else{//right
                    LPWM=calc(LPWM, 0.0, LEFT);
                    LeftDuty((int)LPWM,DC_freq,FORWARD);
                    RightDuty(10*demoj,DC_freq,FORWARD);
                }
                delay(300);
            }
        }
        */
        
        
    }
    LEDon(WHITE);
        for(;;){
            LPWM=calc(LPWM, 0.0, LEFT);
            LeftDuty((int)LPWM,DC_freq);
            RPWM=calc(RPWM, 0.0, RIGHT);
            RightDuty((int)RPWM,DC_freq);
        }
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
Angle range is [0,359], with 0 being directly in front
*/
void turnCalc(void){
    int angle = 0.0f;
    const int minAngle = 5.0f;
    
    //TODO angle calculations
    
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
*/
void turn(int angle){
    if(angle > 0){ //Right turn
        LEDon(RED);
        if (angle > 90){
            RIGHT_DESIRED = LEFT_DESIRED-((LEFT_DESIRED*angle)/90.0f);
        }else{
            RIGHT_DESIRED = (LEFT_DESIRED*angle)/90.0f;
        }
        clip(RIGHT_DESIRED,LB,UB);
    }else if(angle < 0){ //Left
        LEDon(BLUE);
        if (angle > -90){
            LEFT_DESIRED = -(RIGHT_DESIRED*angle)/90.0f;
        }else{
            LEFT_DESIRED =-( RIGHT_DESIRED - ((RIGHT_DESIRED*angle)/90.0f));
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
    float newPWM=currentPWM+\
        (KP*err)+\
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
