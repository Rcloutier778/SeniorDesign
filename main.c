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
 *    PTC3    A1  J1-5
 *    PTC4    A2  J1-7
 *    PTA2    B1  J10-12
 *    PTA1    B2  J10-10
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

void initialize(void);
void en_interrupts();
void delay(int del);
void printBinline(void);
void printLine(void);
void turnLeft(int index);
void turnRight(int index);
float calc(const float currentPWM, const float desiredPWM, const int State, const int wheel);
float servoCalc(const float currentPWM, const float desiredPWM);
void detectFinish(int maximumVal);


int aggro=0;


const int FORWARD=0;
const int REVERSE=1;
const int STRAIGHT = 1;
const int TURNING = 0;
const int SERVO = 2;
const int TURBO=3;
const int SUPER=4;
const int REV_BRAKE=5;
int Lbound=47;//44; //left bound for camera middle check
int Rbound=93;//96; //right bound for camera middle check
const int DC_freq = 10000;
const int servo_freq = 50;
const int BLUETOOTH=1;
const int LEFT=0;
const int RIGHT=1;
const int MIDDLE=70;
int RevBrake[2]={90,50};
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

float KP; //60
float KI; //70
float KD; //25
float PWMErrOld1[2] = {0.0,0.0}; //e(n-1)
float PWMErrOld2[2] = {0.0,0.0}; //e(n-2)


float KPs=0.45f; //0.45
float KIs=0.63f; //0.63
float KDs=0.15f; //0.15


float sPWMErrOld1 = 0.0; //e(n-1)
float sPWMErrOld2 = 0.0; //e(n-2)


float TURN_DESIRED=50.0; //turning desired speed
float STRAIGHT_DESIRED=55.0; //Straight desired speed

float SL; //lower bound of straight speed
float SH; //upper bound of straight speed
float TL; //lower bound of turning speed
float TH; //upper bound of turning speed
float REV_BRAKE_DESIRED[3];//40.0; //Differential braking slower wheel desired speed
float TURBO_DESIRED;
float TURBO_L;
float TURBO_H;
float SUPER_DESIRED;
float SUPER_L;
float SUPER_H;


int TESTING=0; //testing mode on/off

float SPWM=50.0;
float RIGHT_DESIRED=100.0;
float LEFT_DESIRED=0.0;
float SERVO_L=0.0;
float SERVO_H=100.0;
float LPWM=25.0; //PWM of left wheel
float RPWM=25.0; //PWM of right wheel

int laps=0; //Laps completed
int ready=0; //Ready to run?

int tCounter=0;
int lrCounter=0;
int lrPoint=20;

int superPoint=45;//45
int turboPoint=65; //65
float turboBreak=10.0f;
float superBreak=5.0f;
int start=0;

int turn=0;

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
/*
Calculate the PID PWM value based on the current PWM value,
the desired PWM value, and if the car is going straight or turning.
    State=1: Straight
    State=0: Turning
    State=2: Super
    State=3: Turbo
    Wheel=0: Left wheel
    Wheel=1: Right wheel
*/
float calc(const float currentPWM, const float desiredPWM, const int state, const int wheel){
    float err = desiredPWM - currentPWM; //e(n)
    float newPWM = currentPWM +\
    (KP * (err-PWMErrOld1[wheel])) + \
    (KI * ((err+PWMErrOld1[wheel])/2.0f)) + \
    (KD * (err - (2.0f * PWMErrOld1[wheel]) + PWMErrOld2[wheel])); 
    if(state==STRAIGHT){
        clip(newPWM, SL, SH);
    }else if(state==TURNING){
        clip(newPWM, TL, TH);
    }else if(state==TURBO){
        clip(newPWM,TURBO_L,TURBO_H);
    }else if(state==SUPER){
        clip(newPWM,SUPER_L,SUPER_H);
    }else{
        clip(newPWM,-100.0f,SH);
    }
    PWMErrOld2[wheel]=PWMErrOld1[wheel];
    PWMErrOld1[wheel]=err;
    return newPWM;
}

/*
Calculate the PID Servo PWM value based on the current PWM value,
the desired PWM value.
*/
float servoCalc(const float currentPWM, const float desiredPWM){
    float err = desiredPWM - currentPWM; //e(n)
    float newPWM = currentPWM +\
    (KPs * (err-sPWMErrOld1)) + \
    (KIs * ((err+sPWMErrOld1)/2.0f)) + \
    (KDs * (err - (2.0f * sPWMErrOld1) + sPWMErrOld2)); 
    
    clip(newPWM,SERVO_L,SERVO_H);
  
    sPWMErrOld2=sPWMErrOld1;
    sPWMErrOld1=err;
    return newPWM;
}

/*
Initializes the uart, GPIO pins, FTM, ADC0, and PIT. 
If debugcamdata is enabled, will print the values gathered
by the NXP car campera to the terminal using uart. 
*/
int main(void){
    char str[100];
    int maxval=0;
    uint16_t smoothline[128];
    int Lcount=0;
    int Rcount=0;
    for(int i=0;i<127;i++){
        binline[i]=0;
        line[i]=0;        \
        line2[i]=0;
    }
    
    // Initialize everything
    initialize();
    
    // Print welcome over serial
    put("Running... \n\r");
    
    
        
    //TODO
    //Figure out where on camera the sides of the car are
    //If a black line gets too close, steer away?
    
    normalSet();
    if(TESTING){
        TURN_DESIRED=25.0f;
        STRAIGHT_DESIRED=30.0f;
        TURBO_DESIRED=STRAIGHT_DESIRED+25.0f;
        TURBO_L=TURBO_DESIRED-20.0f;
        TURBO_H=TURBO_DESIRED-20.0f;
        SL=STRAIGHT_DESIRED-10.0f;
        SH=STRAIGHT_DESIRED+10.0f;
        TL=TURN_DESIRED-10.0f;
        TH=TURN_DESIRED+10.0f;
        REV_BRAKE_DESIRED[0]=TURN_DESIRED-20.0f;
        REV_BRAKE_DESIRED[1]=TURN_DESIRED-30.0f;
        REV_BRAKE_DESIRED[2]=TURN_DESIRED-40.0f;
        SUPER_DESIRED=STRAIGHT_DESIRED+15.0f;
        SUPER_L=SUPER_DESIRED-20.0f;
        SUPER_H=SUPER_DESIRED-20.0f;    
    }
    SetDutyCycle(0,DC_freq,FORWARD);
    ServoDutyCycle(50.0,servo_freq);
  for(;;){
  while(!ready){
      rugDetect:
        continue;
  }
    for(;;){        
        ready=0;
        if(laps>=2){
            goto rugDetect;
        }
        if((linePtr == NULL) || (dataCount_line==MAX_BUF_SZ)){continue;}
        /*    
        if(aggro==2){
            Lbound=RevBrake[1];
            Rbound=RevBrake[0];
        }else{
            
            Lbound=44;
            Rbound=96;
        }*/
        smoothline[0]=linePtr[0];
        smoothline[1]=linePtr[1];

        for(int i=2; i<126; i++){
            smoothline[i] = (linePtr[i-2] +linePtr[i-1] + linePtr[i] + linePtr[i+1] + linePtr[i+2])/5;
            if(smoothline[i] > maxval){  //get max value
                maxval=smoothline[i];
            }
            //sprintf(str,"%i\r\n",smoothline[i]);
            //put(str);
        }
        smoothline[126]=linePtr[126];
        smoothline[127]=linePtr[127];
        
        /*
        if(maxval/2<18000){
            calc(RPWM,0.0f,STRAIGHT,RIGHT);
            RightDuty(RPWM,DC_freq,FORWARD);
            calc(LPWM,0.0f,STRAIGHT,LEFT);
            LeftDuty(LPWM,DC_freq,FORWARD);
            delay(2);
            SetDutyCycle(0,DC_freq,FORWARD);
            LEDon(CYAN);
            break;
        }
        */
        
        //get binline
        //only look from left to right bound?
        for(int i=0; i<MAX_BUF_SZ; i++){
            if(smoothline[i] <= maxval/2){
                binline[i] = 0;
            }
            else{
                binline[i] = 1;
            }
            
            dataCount_line++;
        }
        
        //for(int i=44;i<98;i++){
        //    sprintf(str,"%i",binline[i]);
        //    put(str);
        //}
        //put("\r\n");
        //Detect finish line
        
       // detectFinish(maxval);
        
		
        //check if in middle
        
        for(int i=MIDDLE; i>Lbound; i--){ //detect left
            if(binline[i] != 1){
                Lcount=0;
                int Rval=0;
                for(int l=MIDDLE; l>Lbound;l--){
                    if(binline[l]!=1){
                        Lcount++;
                    }
                }
                Rcount=0;
                for(int l=MIDDLE; l<Rbound;l++){
                    if(binline[l]!=1){
                        Rcount++;
                        if(Rval==0){
                            Rval=l;
                        }
                    }
                }
                    
                if(Rcount-5>Lcount){//if(Rcount-5>Lcount){
                    turnLeft(Rval);
                    goto stmt;
                }
                for(int k=MIDDLE; k<MIDDLE+(MIDDLE-i); k++){
                    if(binline[k]!=1){
                        turnLeft(k);
                        goto stmt;
                    }
                }
                //black line passed left bound
                turnRight(i);
                goto stmt;
            }
        }
        
        for(int i=MIDDLE; i<Rbound; i++){ //detect right
            if(binline[i] != 1){
                Lcount=0;
                int Lval=0;
                for(int l=MIDDLE; l>Lbound;l--){
                    if(binline[l]!=1){
                        Lcount++;
                        if(Lval==0){
                            Lval=l;
                        }
                    }
                }
                Rcount=0;
                for(int l=MIDDLE; l<Rbound;l++){
                    if(binline[l]!=1){
                        Rcount++;
                        
                    }
                }
                    
                if(Lcount-5>Rcount){//if(Lcount-5>Rcount){
                    turnRight(Lval);
                    goto stmt;
                }
                for(int k=MIDDLE; k<MIDDLE-(i-MIDDLE); k--){
                    if(binline[k]!=1){
                        turnRight(k);
                        goto stmt;
                    }
                }
                //black line passed right bound
                turnLeft(i);
                goto stmt;
            }
        }
    tCounter++;
    lrCounter=0;
    //turbo
    if(tCounter >= turboPoint){
      LEDon(WHITE);
      LPWM=calc(LPWM, TURBO_DESIRED, TURBO, LEFT);
      RPWM=calc(RPWM, TURBO_DESIRED, TURBO, RIGHT);
      RightDuty((int)RPWM,DC_freq,FORWARD);
      LeftDuty((int)LPWM,DC_freq,FORWARD);
    }else if(tCounter >= superPoint){  //super
      LEDon(MAGENTA);
      LPWM=calc(LPWM, SUPER_DESIRED, SUPER, LEFT);
      RPWM=calc(RPWM, SUPER_DESIRED, SUPER, RIGHT);
      RightDuty((int)RPWM,DC_freq,FORWARD);
      LeftDuty((int)LPWM,DC_freq,FORWARD);
    }else{ //straight
      LEDon(GREEN);
      LPWM=calc(LPWM, STRAIGHT_DESIRED, STRAIGHT, LEFT);
      RPWM=calc(RPWM, STRAIGHT_DESIRED, STRAIGHT, RIGHT);
      RightDuty((int)RPWM,DC_freq,FORWARD);
      LeftDuty((int)LPWM,DC_freq,FORWARD);
    }
    SPWM=servoCalc(SPWM,50.0);
    ServoDutyCycle((int)SPWM,servo_freq);
       
    stmt:
        continue;
  }
}
    return 0;
}

/*
Turn Right
If the black line is between the left boundary and the 
reverse braking point, servo will turn.
If the black line is between the reverse braking point and
the middle of the car, servo will be set to max right turn and
right wheel will be slowed down.
Turns on the green LED
*/
void turnRight(int index){
    //2.08 == 2
    int breaker=0;
    start=1;
    lrCounter++;
    tCounter--;
    if(lrCounter>lrPoint){
        if(tCounter>=turboPoint){
            breaker=2;
            
        }else if(tCounter>=superPoint){
            if(breaker<1){
                breaker=1;
            }
        }
        
        tCounter=0;
    }
    
    LEDon(RED);
    
    
        if(index>RevBrake[RIGHT] || breaker!=0){//turn right wheel slower
            SPWM=servoCalc(SPWM,RIGHT_DESIRED);
            ServoDutyCycle((int)SPWM,servo_freq);
            LPWM=calc(LPWM, STRAIGHT_DESIRED, TURNING, LEFT);
            RPWM=calc(RPWM, REV_BRAKE_DESIRED[breaker], TURNING, RIGHT);
            if(RPWM<0.0f){
                RightDuty((int)(-1.0f*RPWM),DC_freq,REVERSE);
            }else{
                RightDuty((int)RPWM,DC_freq,FORWARD);
                LeftDuty((int)LPWM,DC_freq,FORWARD);
            }
        }else{
            SPWM=servoCalc(SPWM,(50.0f));
            ServoDutyCycle(SPWM,servo_freq);
            LPWM=calc(LPWM, STRAIGHT_DESIRED, STRAIGHT, LEFT);
            RPWM=calc(RPWM, TURN_DESIRED, TURNING, RIGHT);
            RightDuty((int)RPWM,DC_freq,FORWARD);
            LeftDuty((int)LPWM,DC_freq,FORWARD);
        }
    

    
}

/*
Turn Left
If the black line is between the right boundary and the 
reverse braking point, servo will turn.
If the black line is between the reverse braking point and
the middle of the car, servo will be set to max left turn and
left wheel will be slowed down.
Turns on the green LED
*/
void turnLeft(int index){
    int breaker=0;
    lrCounter++;
    tCounter--;
    start=1;
    
    if(lrCounter>lrPoint){
        if(tCounter>=turboPoint){
            breaker=2;
            
        }else if(tCounter>=superPoint){
            if(breaker<1){
                breaker=1;
            }
        }
        tCounter=0;
    }
    
    
    LEDon(BLUE);
        
        if(index<RevBrake[LEFT] || breaker!=0){//turn left wheel slower
            SPWM=servoCalc(SPWM,LEFT_DESIRED);
            ServoDutyCycle((int)SPWM,servo_freq);
            LPWM=calc(LPWM, REV_BRAKE_DESIRED[breaker], TURNING, LEFT);
            RPWM=calc(RPWM, STRAIGHT_DESIRED, TURNING, RIGHT);
            if(LPWM<0.0f){
                LeftDuty((int)(-1.0f*LPWM),DC_freq,REVERSE);
            }else{
                RightDuty((int)RPWM,DC_freq,FORWARD);
                LeftDuty((int)LPWM,DC_freq,FORWARD);
            }
        }else{
            SPWM=servoCalc(SPWM,50.0f);
            ServoDutyCycle(SPWM,servo_freq);
            LPWM=calc(LPWM, TURN_DESIRED, TURNING,LEFT);
            RPWM=calc(RPWM, STRAIGHT_DESIRED, STRAIGHT, RIGHT);
            RightDuty((int)RPWM,DC_freq,FORWARD);
            LeftDuty((int)LPWM,DC_freq,FORWARD);
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

/*
Detects if the finish line has been passed
Checks to see if the maximum value found is less than a certain value.
This represents that all of what the camera is seeing is black,
meaning that the finish line has been passed. 
*/
int rugs=0;
void detectFinish(int maximumVal){
    int rugCount=0;
    for(int i=Lbound; i<Rbound; i++){
        if(binline[i]){
            rugCount++;
        }
    }
    if(rugCount<20){
        rugs++;
    }
    if(laps == 2 || rugs == 5){
        LEDon(CYAN);
        REV_BRAKE_DESIRED[0]=0.0f;
        REV_BRAKE_DESIRED[1]=0.0f;
        REV_BRAKE_DESIRED[2]=0.0f;
        TURN_DESIRED=0.0f;
        STRAIGHT_DESIRED=0.0f;
        TURBO_DESIRED=0.0f;
        SUPER_DESIRED=0.0f;
        TURBO_L=-100.0f;
        TURBO_H=100.0f;
        SL=-100.0f;
        SH=100.0f;
        TL=-100.0f;
        TH=100.0f;
        SUPER_L=-100.0f;
        SUPER_H=100.0f;
            
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
Initializes all required modules
*/
void initialize(void)
{
    // Initialize UART
    uart_init();    
    
    // Initialize the FlexTimer
    InitPWM();
    
    init_GPIO(); // For CLK and SI output on GPIO
    init_FTM2(); // To generate CLK, SI, and trigger ADC
    init_ADC0();
    init_PIT();    // To trigger camera read based on integration time
    init_LEDS();
  if(BLUETOOTH){init_BT();} //Initialize the bluetooth controller
}
