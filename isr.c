/*
 *Contains all interrupt handlers for the K64F
 * 
 * Author:  Richard Cloutier
 * Created:  
 * Modified:  
 */
#include "stdio.h"
#include <math.h>
#include <stdlib.h>
#include "isr.h"
#include "MK64F12.h"
#include "LEDS.h"

extern int ready;

void PORTA_IRQHandler(void){ //For switch 3 (next to LED)
    PORTA_ISFR = PORT_ISFR_ISF_MASK;
    ready=1;
    return;
}

/*
   If switch 2 is pressed, the IRQ handler is triggered.
   Clears the interrupt flag and toggles the ready switch
   */
void PORTC_IRQHandler(void){ //For switch 2
    PORTC_ISFR = PORT_ISFR_ISF_MASK; //clear interrupt
    //ready=1;
    ready=0;
    return;
}
