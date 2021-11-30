/*
 * U0_PWM.h
 *
 * Created: 11/7/2021 5:29:24 PM
 *  Author: Justin Loi - FL23090
 */ 


#ifndef U0_PWM_H_
#define U0_PWM_H_
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>

void PWM_Init();
void PWM_Rotate();

#endif /* U0_PWM_H_ */