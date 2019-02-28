#ifndef PWM_H_
#define PWM_H_

#define DUTY_MULT 5

void SetDCDutyCycle(double DutyCycle, unsigned int Frequency);
void SetServoDutyCycle(double DutyCycle, unsigned int Frequency);
void InitPWM(void);
void PWM_ISR(void);
void enable_motor(int mot_num);
void disable_motor(int mot_num);
void forward_motor(int mot_num);
void reverse_motor(int mot_num);
void toggle_direction(int mot_num);

#endif /* PWM_H_ */
