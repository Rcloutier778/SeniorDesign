/*
 * Pulse-Width-Modulation Code for K64
 * PWM signal can be connected to output pins PC3 and PC4
 * 
 * Author: Brent Dimmig <bnd8678@rit.edu>
 * Modified by: 
 * Created: 2/20/2014
 * Modified: 3/07/2015
 */
#include "MK64F12.h"
#include "pwm.h"

/*From clock setup 0 in system_MK64f12.c*/
#define DEFAULT_SYSTEM_CLOCK 20485760u /* Default System clock value */
#define CLOCK                   20485760u
#define PWM_FREQUENCY           10000
#define SERVO_FREQUENCY         50
#define FTM2_PS                 32
#define FTM0_MOD_VALUE          (CLOCK/PWM_FREQUENCY)
#define FTM2_MOD_VALUE          (CLOCK/FTM2_PS/SERVO_FREQUENCY)

static volatile unsigned int PWMTick = 0;

/*
 * Change the DC Motor Duty Cycle and Frequency
 * @param DutyCycle (0 to 100)
 * @param Frequency (~1000 Hz to 20000 Hz)
 */
void SetDCDutyCycle(double DutyCycle, unsigned int Frequency)
{
    // Calculate the new cutoff value
    uint16_t mod = (uint16_t) (((CLOCK/Frequency) * DutyCycle) / 100);
  
    // Set outputs 

	FTM0_C2V = mod; 
	FTM0_C6V= mod;
    // Update the clock to the new frequency
    FTM0_MOD = (CLOCK/Frequency);
}

/*
 * Change the Servo Motor Duty Cycle and Frequency
 * @param DutyCycle (~5 to ~10)
 * @param Frequency (~50 Hz)
 */
void SetServoDutyCycle(double DutyCycle, unsigned int Frequency)
{
    // Calculate the new cutoff value
    uint16_t mod = (uint16_t) (((CLOCK/FTM2_PS/Frequency) * DutyCycle) / 100);
 
    FTM2_C0V = mod;

    // Update the clock to the new frequency
    FTM2_MOD = (CLOCK/FTM2_PS/Frequency);
}

/*
 * Initialize the FlexTimer for PWM
 */
void InitPWM()
{
    // 12.2.13 Enable the Clock to the FTM0 and FTM2 Modules
    SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
    SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;
    
    // Enable clock on PORT B and C so it can output
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;
    
    // 11.4.1 Route the output of FTM channel 0 to the pins
    // Use drive strength enable flag to high drive strength
    //These port/pins may need to be updated for the K64 <Yes, they do. Here are two that work.>
    
    PORTC_PCR3  = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; //FTM0_CH2
    //PORTC_PCR4  = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; //FTM0_CH3
    PORTA_PCR1  = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; //FTM0_CH6
    //PORTA_PCR2  = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; //FTM0_CH7
    PORTB_PCR18 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; //FTM2_CH0
    
    // 39.3.10 Disable Write Protection
    FTM0_MODE |= FTM_MODE_WPDIS_MASK;
    FTM2_MODE |= FTM_MODE_WPDIS_MASK;
    
    // 39.3.4 FTM Counter Value
    // Initialize the CNT to 0 before writing to MOD
    FTM0_CNT = 0;
    FTM2_CNT = 0;
    
    // 39.3.8 Set the Counter Initial Value to 0
    FTM0_CNTIN = 0;
    FTM2_CNTIN = 0;
    
    // 39.3.5 Set the Modulo resister
    FTM0_MOD = FTM0_MOD_VALUE;
    FTM2_MOD = FTM2_MOD_VALUE;
    //FTM0->MOD = (DEFAULT_SYSTEM_CLOCK/(1<<7))/1000;

    // 39.3.6 Set the Status and Control of both channels
    // Used to configure mode, edge and level selection
    // See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
    FTM0_C3SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK;
 
    // See Table 39-67,  Edge-aligned PWM, Low-true pulses (clear out on match)
    FTM0_C2SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM0_C2SC &= ~FTM_CnSC_ELSA_MASK;
    
    // 39.3.6 Set the Status and Control of both channels
    // Used to configure mode, edge and level selection
    // See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
    FTM0_C7SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM0_C7SC &= ~FTM_CnSC_ELSA_MASK;
 
    // See Table 39-67,  Edge-aligned PWM, Low-true pulses (clear out on match)
    FTM0_C6SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM0_C6SC &= ~FTM_CnSC_ELSA_MASK;
 
    // See Table 39-67,  Edge-aligned PWM, Low-true pulses (clear out on match)
    FTM2_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM2_C0SC &= ~FTM_CnSC_ELSA_MASK;
    
    // 39.3.3 FTM Setup
    // Set prescale value to 1 
    // Chose system clock source
    // Timer Overflow Interrupt Enable
    FTM0_SC = FTM_SC_PS(0) | FTM_SC_CLKS(1); 
    FTM0_SC |= FTM_SC_TOIE_MASK;

    FTM2_SC = FTM_SC_PS(5) | FTM_SC_CLKS(1); 
    FTM2_SC |= FTM_SC_TOIE_MASK;

    // Enable Interrupt Vector for FTM
    NVIC_EnableIRQ(FTM0_IRQn);
    NVIC_EnableIRQ(FTM2_IRQn);

}

/*OK to remove this ISR?*/
void FTM0_IRQHandler(void){ //For FTM timer
  FTM0_SC &= ~FTM_SC_TOF_MASK;
  
    //if motor tick less than 255 count up... 
    if (PWMTick < 0xff)
        PWMTick++;
}

/*OK to remove this ISR?*/
void FTM2_IRQHandler(void){ //For FTM timer
  FTM2_SC &= ~FTM_SC_TOF_MASK;
}

void enable_motor(int mot_num){
	// output enable for motor 1
	if(mot_num == 0){
		GPIOC_PSOR 	|= (1 << 2);
	}
	// output enable for motor 2
	else if(mot_num == 1){
		GPIOB_PSOR 	|= (1 << 9);
	}
	// output enable for both motors
	else if(mot_num == 2){
		
	}
}

void disable_motor(int mot_num){
	// output disable for motor 1
	if(mot_num == 0){
		GPIOC_PCOR 	|= (1 << 2);
	}
	// output disable for motor 2
	else if(mot_num == 1){
		GPIOB_PCOR 	|= (1 << 9);
	}
	// output disable for both motors
	else if(mot_num == 2){
		GPIOC_PCOR 	|= (1 << 2);
		GPIOB_PCOR 	|= (1 << 9);
	}
}

void forward_motor(int mot_num){
	// output forward for motor 1
	if(mot_num == 0){
		GPIOA_PSOR |= (1 << 2);
	}
	// output forward for motor 2
	else if(mot_num == 1){
		GPIOB_PSOR |= (1 << 23);
	}
	// output forward for both motors
	else if(mot_num == 2){
		GPIOA_PSOR |= (1 << 2);
		GPIOB_PSOR |= (1 << 23);
	}
}

void reverse_motor(int mot_num){
	// output reverse for motor 1
	if(mot_num == 0){
		GPIOA_PCOR |= (1 << 2);
	}
	// output reverse for motor 2
	else if(mot_num == 1){
		GPIOB_PCOR |= (1 << 23);
	}
	// output reverse for both motors
	else if(mot_num == 2){
		GPIOA_PCOR |= (1 << 2);
		GPIOB_PCOR |= (1 << 23);
	}
}

void toggle_direction(int mot_num){
	// output reverse for motor 1
	if(mot_num == 0){
		GPIOA_PTOR |= (1 << 2);
	}
	// output reverse for motor 2
	else if(mot_num == 1){
		GPIOB_PTOR |= (1 << 23);
	}
	// output reverse for both motors
	else if(mot_num == 2){
		GPIOA_PTOR |= (1 << 2);
		GPIOB_PTOR |= (1 << 23);
	}
}
