#include "MK64F12.h"
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

void init_GPIO(void) {
    // Enable PORTB clock
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

    // Set motor enable pins to output mode
	GPIOC_PDDR |= (1 << 2); // Motor 1 Enable
    GPIOB_PDDR |= (1 << 9); // Motor 2 enable
	
	
		// Set pins to output mode    
	GPIOA_PDDR |= (1 << 2); //Motor 1 Direction
	GPIOB_PDDR |= (1 << 23); // Motor 2 Direction
	
		// Disable motors
    GPIOC_PCOR 	|= (1 << 2);
	GPIOB_PCOR 	|= (1 << 9);
	
		//Set the motors to forward
	GPIOA_PSOR |= (1 << 2);
    GPIOB_PSOR |= (1 << 23);
    return;
}    

/* Set up ADC for capturing camera data */
void init_ADC0(void) {
    unsigned int calib;

    // Turn on ADC0
    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;

    // Single ended 16 bit conversion, no clock divider
    ADC0_CFG1 &= ~ADC_CFG1_ADIV_MASK; // No clock divider
    ADC0_CFG1 |= ADC_CFG1_MODE(3);    // Single-ended 16-bit conversion

    // Do ADC calibration for singled ended ADC. Do not touch.
    ADC0_SC3 = ADC_SC3_CAL_MASK;
    while ((ADC0_SC3 & ADC_SC3_CAL_MASK) != 0);
    calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
    calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
    calib = calib >> 1; calib |= 0x8000;
    ADC0_PG = calib;

    // Select hardware trigger.
    ADC0_SC2 |= ADC_SC2_ADTRG_MASK;

    // Set to single ended mode
    ADC0_SC1A &= ~ADC_SC1_DIFF_MASK;
    ADC0_SC1A &= ~ADC_SC1_ADCH_MASK;
	// Enable interrupts
    ADC0_SC1A |= ADC_SC1_AIEN_MASK;

    // Set up FTM2 trigger on ADC0
    SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(9);      // Select FTM1 trigger
    SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK;   // Alternative trigger enable
    SIM_SOPT7 &= ~SIM_SOPT7_ADC0PRETRGSEL_MASK; // Pretrigger A

    // Enable NVIC interrupt
    NVIC_EnableIRQ(ADC0_IRQn);
}


/* Initialization of PIT timer to control integration period */
void init_PIT(void) {
    // Enable clock for timers
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

    // Enable timers to continue in debug mode
    PIT_MCR &= ~PIT_MCR_FRZ_MASK; // In case you need to debug

    // PIT clock frequency is the system clock
    // Load the value that the timer will count down from
    PIT_LDVAL0 = INTEGRATION_TIME * DEFAULT_SYSTEM_CLOCK;

    // Enable timer interrupts
    PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;

    // Enable the timer
    PIT_MCR &= ~PIT_MCR_MDIS_MASK;
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

    // Clear interrupt flag
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

    // Enable PIT interrupt in the interrupt controller
    NVIC_EnableIRQ(PIT0_IRQn);

    return;
}


/* Initialization of FTM2 for camera */
void init_FTM1() {
    // Enable clock
    SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;

    // Disable Write Protection
    if (FTM1_FMS & FTM_FMS_WPEN_MASK) {
        FTM1_MODE |= FTM_MODE_WPDIS_MASK;
    }

    // Set output to '1' on init
    FTM1_OUTINIT |= FTM_OUTINIT_CH0OI_MASK;

    // Set the Counter Initial Value to 0
    FTM1_CNTIN = FTM_CNTIN_INIT(0);

    // Initialize the CNT to 0 before writing to MOD
    FTM1_CNT = 0;

    // Set the period (~10us)
    FTM1_MOD = DEFAULT_SYSTEM_CLOCK * 10 / 1000000;

    // 50% duty
    FTM1_C0V = (DEFAULT_SYSTEM_CLOCK * 10 / 1000000) / 2;

    // Set edge-aligned mode
    // DECAPEN = 0, COMBINE = 0, CPWMS = 0 - all default
    // MSB = 1, MSA = X
    FTM1_C0SC |= FTM_CnSC_MSB_MASK;

    // Enable High-true pulses
    // ELSB = 1, ELSA = 0
    FTM1_C0SC |= FTM_CnSC_ELSB_MASK;
    FTM1_C0SC &= ~FTM_CnSC_ELSA_MASK;

    // Enable hardware trigger from FTM2
    FTM1_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;

    // Don't enable interrupts yet (disable)
    FTM1_SC &= ~FTM_SC_TOIE_MASK;

    // No prescaler, system clock
    FTM1_SC &= ~FTM_SC_PS_MASK;
    FTM1_SC |= FTM_SC_CLKS(1);

    // Set up interrupt
    NVIC_EnableIRQ(FTM1_IRQn);

    return;
}
