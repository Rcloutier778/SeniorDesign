#ifndef ultrasonic_H_
#define ultrasonic_H_
void init_ultrasonic(void);
void UART4_RX_TX_IRQHandler(void);
void PIT1_IRQHandler(void);
void init_PIT(void);
void init_FTM(void);
void FTM3_IRQHandler(void);

//void UART0_RX_TX_IRQHandler(void);
#endif /* ultrasonic_H_ */
