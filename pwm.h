#ifndef PWM_H_
#define PWM_H_

void SetDutyCycle(int DutyCycle, unsigned int Frequency);
void InitPWM(void);
void PWM_ISR(void);
void RightDuty(int DutyCycle, unsigned int Frequency);
void LeftDuty(int DutyCycle, unsigned int Frequency);
#endif /* PWM_H_ */
