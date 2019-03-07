#ifndef Bluetooth_H_
#define Bluetooth_H_

void init_BT(void);
void UART3_RX_IRQHandler(void);
void UART3_TX_IRQHandler(void);
void UART3_RX_TX_IRQHandler(void);
void sendFloatTx(void);
void pollGPSRx(void);
#endif /* Bluetooth_H_ */
