/*
 * Init Method the K64F
 * Initializes the ADC0, FTM2, PIT, and various GPIO pins
 * 
 * Author:  Richard Cloutier
 * Created:  12/1/2017
*/

#include "MK64F12.h"
#include "inits.h"

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
//	(camera clk is the mod value set in FTM2)
// default = .0075f
#define INTEGRATION_TIME .0075f
#define BAUD_RATE 9600      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)


/* Initialization of FTM2 for camera */
void init_FTM2(void){
	// Enable clock
    SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

	// Disable Write Protection
	FTM2_MODE |= FTM_MODE_WPDIS_MASK;
	
	// Set output to '1' on init  0 through 7, not sure which
	FTM2_OUTINIT |= (FTM_OUTINIT_CH0OI_MASK | FTM_OUTINIT_CH1OI_MASK | 
										FTM_OUTINIT_CH2OI_MASK | FTM_OUTINIT_CH3OI_MASK | 
										FTM_OUTINIT_CH4OI_MASK | FTM_OUTINIT_CH5OI_MASK | 
										FTM_OUTINIT_CH6OI_MASK | FTM_OUTINIT_CH7OI_MASK);
	
	// Initialize the CNT to 0 before writing to MOD
    FTM2_CNT = 0;
	
	// Set the Counter Initial Value to 0
    FTM2_CNTIN = 0x0;
	
	// Set the period (~10us)
	FTM2_MOD = (DEFAULT_SYSTEM_CLOCK)/100000;
	
	// 50% duty 
	//TODO?
	FTM2_C0V |= FTM_CnV_VAL (51);
	FTM2_C1V |= FTM_CnV_VAL (51);
	
	// Set edge-aligned mode
	//FTM2_C0SC |= FTM_CnSC_MSA_MASK;
	FTM2_C0SC |= FTM_CnSC_MSB_MASK;
	
	// Enable High-true pulses
	// ELSB = 1, ELSA = 0
	FTM2_C0SC |= FTM_CnSC_ELSB_MASK;
	FTM2_C0SC &= ~(FTM_CnSC_ELSA_MASK);
	
	// Enable hardware trigger from FTM2
	FTM2_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;
	
	// Don't enable interrupts yet (disable)
	FTM2_SC &= ~FTM_SC_TOIE_MASK;
	
	// No prescalar, system clock
	FTM2_SC |= FTM_SC_CLKS(1);
	
	//Enable FTM
	FTM2_MODE |= FTM_MODE_FTMEN_MASK;
	
	// Set up interrupt
	NVIC_EnableIRQ(FTM2_IRQn);
	
	return;
}

/* Initialization of PIT timer to control 
* 		integration period
*/
void init_PIT(void){
	// Setup periodic interrupt timer (PIT)
	
	// Enable clock for timers
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	PIT_MCR &= ~PIT_MCR_MDIS_MASK;
	
	// Enable timers to continue in debug mode
	PIT_MCR &= ~PIT_MCR_FRZ_MASK; // In case you need to debug
	
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	PIT_LDVAL0 = (uint32_t) DEFAULT_SYSTEM_CLOCK * INTEGRATION_TIME;
	
	// Enable timer interrupts
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
	
	// Enable the timer 0,1,2,3?
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

	// Clear interrupt flag
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT0_IRQn);
	return;
}


/* Set up pins for GPIO
* 	PTB9 		- camera clk
*   PTB23		- camera SI
*	PTB22		- red LED
*/
void init_GPIO(void){
    //INIT CLOCKS
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;//Camera,LEDs
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;//SW2
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;//SW3
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

    //CONFIG PCR for inputs 
    PORTB_PCR23 |= PORT_PCR_PE_MASK; //camera si
    PORTB_PCR9 |= PORT_PCR_PE_MASK; //camera clk

    PORTD_PCR1 |= PORT_PCR_PE_MASK;

    PORTC_PCR6 |= PORT_PCR_PE_MASK;//SW2
    PORTA_PCR4 |= PORT_PCR_PE_MASK;//SW3
    //PORTC_PCR14 |= PORT_PCR_PE_MASK; //BT RX
    //PORTC_PCR15 |= PORT_PCR_PE_MASK; //BT TX


    //configure mux for outputs
    PORTB_PCR23 = PORT_PCR_MUX(1); //CAMERA SI
    PORTB_PCR9 = PORT_PCR_MUX(1); //CAMERA CLK

    PORTD_PCR1 = PORT_PCR_MUX(1);


    //PORTC_PCR15 = PORT_PCR_MUX(1); //BT TX

    //configure mux for inputs
    PORTC_PCR6 = PORT_PCR_MUX(1); //SW2
    PORTA_PCR4 = PORT_PCR_MUX(1); //SW3
    //PORTC_PCR14 = PORT_PCR_MUX(1); //BT RX

    //switch the GPIO pins to output mode
    GPIOB_PDDR = (1<<9) | (1<<23); // CLK || SI
    //GPIOC_PDDR = (1<<15); //BT_TX

    GPIOD_PDDR = (1<<1);
    GPIOD_PDOR = (1<<1);
    //Configure GPIO pins for input
    GPIOC_PDDR |= (0<<6);// | (0<<14);//Button || BT_RX
    GPIOA_PDDR = (0<<4);

    PORTD_PCR1 |= PORT_PCR_DSE_MASK;// | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; 
    // interrupt configuration for SW3(Rising Edge) and SW2 (Either)
    //sw2, either
    PORTC_PCR6 |= PORT_PCR_IRQC (9);
    PORTA_PCR4 |= PORT_PCR_IRQC (9);
    NVIC_EnableIRQ(PORTB_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTC_IRQn);
    return;
}

/* Set up ADC for capturing camera data */
void init_ADC0(void) {
    unsigned int calib;
    // Turn on ADC0
    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;

    // Single ended 16 bit conversion, no clock divider
    ADC0_CFG1 |= ADC_CFG1_MODE (3);

    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC0_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
    calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
    calib = calib >> 1; calib |= 0x8000;
    ADC0_PG = calib;

    // Select hardware trigger.
    ADC0_SC2 |= ADC_SC2_ADTRG_MASK;

    // Set to single ended mode	
    ADC0_SC1A = ADC_SC1_ADCH (1);

    //Enable interrupt
    ADC0_SC1A |= ADC_SC1_AIEN_MASK;

    // Set up FTM2 trigger on ADC0
    SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL (10); // FTM2 select
    SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK; // Alternative trigger en.
    SIM_SOPT7 &= ~SIM_SOPT7_ADC0PRETRGSEL_MASK; // Pretrigger A

    // Enable NVIC interrupt
    NVIC_EnableIRQ(ADC0_IRQn);
}
