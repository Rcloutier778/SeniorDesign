#include "MK64F12.h"
#include "stdio.h"

#define BAUD_RATE 9600      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)


//PTE24, PTE25
void i2c_init(void){
    //Enable i2c
    I2C0_C1 |= I2C_C1_IICEN_MASK;
    
    //Interrupt enable
    I2C0_C1 |= I2C_C1_IICIE_MASK;
    
    //slave mode enable
    I2C0_C1 &= ~I2C_C1_IICEN_MASK;
}