#ifndef ultrasonic_H_
#define ultrasonic_H_
void init_ultrasonic(void);
void PIT0_IRQHandler(void);
void init_PIT(void);
double getUltrasonic(void);
//void UART0_RX_TX_IRQHandler(void);
#endif /* ultrasonic_H_ */
