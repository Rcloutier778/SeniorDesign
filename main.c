/*
 * Main Method for testing the PWM Code for the K64F
 * PWM signal can be connected to output pins are PC3 and PC4
 * 
 * Author:  
 * Created:  
 * Modified:  
 */

#include "MK64F12.h"
#include "uart.h"
#include "pwm.h"
#include "stdio.h"
#include "Bluetooth.h"
#include "initialization.h"


// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u

// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk
//  (camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME .0225f

void initialize();
void init_GPIO(void);
void en_interrupts();
void delay();

// Pixel counter for camera logic. Starts at -2 so that the SI pulse occurs
//      ADC reads start
int pixcnt = -2;

// clkval toggles with each FTM interrupt
int clkval = 0;

// line stores the current array of camera data
uint16_t line[128];

// These variables are for streaming the camera data over UART
int debugcamdata = 1;

int capcnt = 0;
int left = 0;
int right = 0;
int carpet = 0;
int notcarp = 0;
char str[100];

int main(void)
{
	int speed = 0;
	int i=0;
    // Initialize UART and PWM
    initialize();

    // Print welcome over serial
    uart_put("Running... \n\r");

    //Generate 50% duty cycle at 10kHz for the DC motors. Forward direction
    SetDCDutyCycle(speed, 10000);
	delay(5000);
	enable_motor(2);

    for(;;)  //loop forever
    {
		speed=speed+5;
		if(speed >40){
			speed = 0;
			toggle_direction(2);
			i++;
		}
		if(i>5){
			disable_motor(2);
		}
		// Send the array over uart
		sprintf(str,"%i\n\r",-1); // Start value
		uart_put(str); 
		SetDCDutyCycle(speed, 10000);
		delay(2000);
	}
    return 0;
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

void initialize()
{
    // Initialize UART
    uart_init();  
    init_GPIO(); // For CLK and SI output on GPIO
    init_FTM1(); // To generate CLK, SI, and trigger ADC
    init_ADC0();
    init_PIT();  // To trigger camera read based on integration time

    // Initialize the FlexTimer
    InitPWM();
	init_BT();
}

