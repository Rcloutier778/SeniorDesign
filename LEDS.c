/*
 * Handles LED states for the K64F
 * 
 * Author:  Richard Cloutier
 * Created:  12/1/2017
 * Modified:  
 */
#include "MK64F12.h"
#include "LEDS.h"


#define cr 1UL<<22
#define cb 1UL<<21
#define cg 1UL<<26


/*
   Initializes the LED GPIO pins red green and blue.
   */
void init_LEDS(){
    //INIT
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;//Camera,LEDs
    SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;//LED

    //CONFIG PCR for inputs 
    PORTB_PCR22 |= PORT_PCR_PE_MASK; //Red
    PORTB_PCR21 |= PORT_PCR_PE_MASK; //Blue
    PORTE_PCR26 |= PORT_PCR_PE_MASK;//Green

    //configure mux for outputs
    PORTB_PCR22 = PORT_PCR_MUX(1);//Red
    PORTE_PCR26 = PORT_PCR_MUX(1);//Green
    PORTB_PCR21 = PORT_PCR_MUX(1);//Blue

    //switch the GPIO pins to output mode
    GPIOB_PDDR |= (1<<22) | (1<<21); //Red || Blue 
    GPIOE_PDDR |= (1<<26); //Green

    //turn off leds
    GPIOB_PSOR = (1UL << 21) | (1UL << 22); //blue red
    GPIOE_PSOR = 1UL << 26; //green

}

/*
   Turns on the LED to the inputted color. 
   Input is the LED defined in LEDS.h
   */
void LEDon(int color){
    if(color==RED){
        GPIOE_PSOR = cg;
        GPIOB_PSOR = cb;
        GPIOB_PCOR=cr;
    }else if(color==GREEN){
        GPIOE_PCOR = cg;
        GPIOB_PSOR = cb;
        GPIOB_PSOR=cr;
    }else if(color==BLUE){
        GPIOE_PSOR = cg;
        GPIOB_PCOR = cb;
        GPIOB_PSOR=cr;
    }else if(color==CYAN){
        GPIOE_PCOR = cg;
        GPIOB_PCOR = cb;
        GPIOB_PSOR=cr;
    }else if(color==MAGENTA){
        GPIOE_PSOR = cg;
        GPIOB_PCOR = cb;
        GPIOB_PCOR=cr;
    }else if(color==YELLOW){
        GPIOE_PCOR = cg;
        GPIOB_PSOR = cb;
        GPIOB_PCOR=cr;
    }else if(color==WHITE){
        GPIOE_PCOR = cg;
        GPIOB_PCOR = cb;
        GPIOB_PCOR=cr;
    }
}

void LEDoff(void){
    GPIOE_PSOR = cg;
    GPIOB_PSOR = cb;
    GPIOB_PSOR=cr;
}
