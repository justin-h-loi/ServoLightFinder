/*
 * U0_PWM.c
 *
 * Created: 11/7/2021 5:28:16 PM
 *  Author: Justin Loi - FL23090
 */ 
//Input: none
//Output: none
//Initiates the 8-bit PWM
#include "U0_PWM.h"
#define STARTPOS 54 //0
#define ENDPOS 276 //180
#define FULLSWEEP 3
#define LOCSWEEPS 4 //local sweep start - primary angle
#define LOCSWEEPP 5 //local sweep plus - primary angle + 10
#define LOCSWEEPM 6 //local sweep minus - primary angle - 10

void PWM_Init()
{
	TCCR1A |= (1<<WGM11) | (1<<COM1A1) | (1<<COM1B1);
	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS10) | (1<<CS11);
	ICR1 = 4999;
	DDRB |= (1<<PB5);
}

//Input: none
//Output: none
//Rotates the Servo uninterrupted
void PWM_Rotate(int behavior, int primAng)
{	
	switch(behavior)
	{
		case FULLSWEEP: OCR1A += (ENDPOS-STARTPOS)/9; break; //fullsweep mode
		case LOCSWEEPS: OCR1A = STARTPOS + ((ENDPOS-STARTPOS)/18 * primAng/10); break; //local sweep prim. angle
		case LOCSWEEPP: OCR1A = STARTPOS + (ENDPOS-STARTPOS)/18 * primAng/10 + (ENDPOS-STARTPOS)/18; break; //local sweep prim. angle + 10
		case LOCSWEEPM: OCR1A = STARTPOS + (ENDPOS-STARTPOS)/18 * primAng/10 - (ENDPOS-STARTPOS)/18; break; //local sweep prim. angle - 10
	}
	_delay_ms(500);
}
