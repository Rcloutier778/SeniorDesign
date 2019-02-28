#ifndef INIT_H_
#define INIT_H_

void init_FTM1(void);
void init_PIT(void);
void init_ADC0(void);
void FTM1_IRQHandler(void);
void PIT0_IRQHandler(void);
void ADC0_IRQHandler(void);

#endif /* INIT_H_ */
