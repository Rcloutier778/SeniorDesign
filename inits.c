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


    PORTC_PCR6 |= PORT_PCR_PE_MASK;//SW2
    PORTA_PCR4 |= PORT_PCR_PE_MASK;//SW3
    //PORTC_PCR14 |= PORT_PCR_PE_MASK; //BT RX
    //PORTC_PCR15 |= PORT_PCR_PE_MASK; //BT TX


    //configure mux for outputs
    PORTB_PCR23 = PORT_PCR_MUX(1); //CAMERA SI
    PORTB_PCR9 = PORT_PCR_MUX(1); //CAMERA CLK



    //PORTC_PCR15 = PORT_PCR_MUX(1); //BT TX

    //configure mux for inputs
    PORTC_PCR6 = PORT_PCR_MUX(1); //SW2
    PORTA_PCR4 = PORT_PCR_MUX(1); //SW3
    //PORTC_PCR14 = PORT_PCR_MUX(1); //BT RX

    //switch the GPIO pins to output mode
    GPIOB_PDDR = (1<<9) | (1<<23); // CLK || SI
    //GPIOC_PDDR = (1<<15); //BT_TX

    //Configure GPIO pins for input
    GPIOC_PDDR &= ~(1<<6);// | (0<<14);//Button || BT_RX
    GPIOA_PDDR &= ~(1<<4);

    // interrupt configuration for SW3(Rising Edge) and SW2 (Either)
    //sw2, either
    PORTC_PCR6 |= PORT_PCR_IRQC (9);
    PORTA_PCR4 |= PORT_PCR_IRQC (9);
    NVIC_EnableIRQ(PORTB_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTC_IRQn);
    return;
}
