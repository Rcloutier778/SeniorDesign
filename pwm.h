#ifndef PWM_H_
#define PWM_H_

void SetDutyCycle(unsigned int DutyCycle, unsigned int Frequency, int dir);
void InitPWM(void);
void PWM_ISR(void);
void RightDuty(unsigned int DutyCycle, unsigned int Frequency, int dir);
void LeftDuty(unsigned int DutyCycle, unsigned int Frequency, int dir);
#endif /* PWM_H_ */
