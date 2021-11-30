/*
 * main.c
 *
 * Created: 11/2/2021 4:47:46 PM
 * Author : Justin Loi - FL23090
 */ 
#ifndef F_CPU

#ifdef USING_BOOTLOADER
#define F_CPU 2000000UL
#else
#define F_CPU 16000000UL
#endif

#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "U0_LCD_Driver.h"
#include "U0_ADC.h"
#include "U0_PWM.h"

void SetupInterrupts(void);
void BootLoaderFixes(void);
void FTL();
void ATL();

//Defines for servo positions, modes, and behaviors
#define STARTPOS 54 //0
#define ENDPOS 276 //180
#define ATLMODE 1
#define FTLMODE 2
#define FULLSWEEP 3
#define LOCSWEEPS 4 //local sweep start - primary angle
#define LOCSWEEPP 5 //local sweep plus - primary angle + 10
#define LOCSWEEPM 6 //local sweep minus - primary angle - 10

int mode; //global variable for current mode of avr (FTL or ATL or none)

int main(void)
{
	//Allows interrupts in debug mode
	#ifdef USING_BOOTLOADER
	BootLoaderFixes();
	#endif
	
	//Setup All Joystick inputs
	DDRB  &= ~0b11000000;  //set B6,B7 as inputs
	PORTB |=  0b11000000;  //enable pull up resistors on B6,B7
	DDRE  &= ~0b00001000;  //set E3 to inputs
	PORTE |=  0b00001000;  //enable pull up resistor on pin E3
	
	LCD_Init(); //Initialize LCD
	ADC_init(); //Initialize ADC
	PWM_Init(); //Initialize PWM/Servo output
	SetupInterrupts();	//setup the interrupts
	sei();				//enable global interrupts
	mode = 0;			//Current mode set to none
	OCR1A = STARTPOS;	//Sets servo to 0 degrees
	while(1)	//infinite loop to prevent program from terminating
	{
		
	}
}

void SetupInterrupts(void)
{
	PCMSK1  |= (1<<PCINT14); //Unmask bit for UP Button on Butterfly, PB6->PCINT14 to allow it to trigger interrupt flag PCIF1
	PCMSK1  |= (1<<PCINT15); //Unmask bit for DOWN Button on Butterfly, PB7->PCINT15 to allow it to trigger interrupt flag PCIF1
	PCMSK0  |= (1<<PCINT3); //Unmask bit for RIGHT Button on Butterfly, PE3->PCINT3 to allow it to trigger interrupt flag PCIF0
	EIMSK   = (1<<PCIE0) | (1<<PCIE1);    //Enable interrupt for flag PCIF1 and PCIF0
}

//This performs adjustments needed to undo actions of Butterfly boot loader
void BootLoaderFixes(void)
{
	//Boot loader Disables E2 and E3 digital input drivers, which are used for left and right
	//The following code re-enables them by clearing the appropriate disable bits
	DIDR1 &= ~((1<<AIN0D)|(1<<AIN1D));
}

//called for PB6 (up), PB7 (down)
ISR(PCINT1_vect) 		//remember this is called on pin change 0->1 and 1->0
{
	//static uint8_t p4Prev=1; //for storing previous value of PB4 to detect
	static uint8_t p6Prev=1; //for storing previous value of PB6 to detect
	static uint8_t p7Prev=1; //for storing previous value of PB7 to detect
	
	if(((PINB & (1<<6))  == 0) &&
	((p6Prev & (1<<6))  != 0)) //when on UP button status being newly pressed, but not when it is released
	{
		while((PINB & (1<<6))  == 0) //while button is pressed
		{
			mode = FTLMODE; //Sets current mode to FTL
			LCD_AllSegments(FALSE);
			LCD_WriteDigit('F', 3); //Displays "FTL" to LCD for user feedback
			LCD_WriteDigit('T', 4);
			LCD_WriteDigit('L', 5);
			_delay_ms(100);
		}
	}
	else if(((PINB & (1<<7))  == 0) &&
	((p7Prev & (1<<7))  != 0)) //when on DOWN button status being newly pressed, but not when it is released
	{
		while((PINB & (1<<7))  == 0) //while button is pressed
		{
			mode = ATLMODE; //Sets current mode to ATL
			LCD_AllSegments(FALSE);
			LCD_WriteDigit('A', 3); //Displays "ATL" to LCD for user feedback
			LCD_WriteDigit('T', 4);
			LCD_WriteDigit('L', 5);
			_delay_ms(100);
		}
	}
	p6Prev = (PINB); //save UP button status
	p7Prev = (PINB); //save DOWN button status
}

//called for PE3 (right)
ISR(PCINT0_vect) //remember this is called on pin change 0->1 and 1->0
{
	static uint8_t p3Prev=1; //for storing previous value of PE3 to detect
	
	//when on LEFT button status being newly pressed, but not when it is released
	if(((PINE & (1<<3))  == 0) && ((p3Prev & (1<<3))  != 0)) //when on RIGHT button status being newly pressed, but not when it is released
	{
		while((PINE & (1<<3))  == 0) //while button is pressed
		{
			if(mode==ATLMODE) //Check for what current mode is. Then executes correct mode function.
			{
				OCR1A = STARTPOS;
				ATL();
			}
			else if(mode==FTLMODE)
			{
				OCR1A = STARTPOS;
				FTL();
			}
			else
			{
				//nothing happens if mode is not selected
			}
		}
	}
	
	p3Prev = (PINE); //save RIGHT button status
}

//Mode functions
void FTL() //Follow the light
{
	//Fullsweep
	cli(); //Disable interrupts during fullsweep
	LCD_AllSegments(FALSE);
	LCD_WriteDigit('S', 0); //Displays to LCD that servo/sensor is currently "SWEEP"ing
	LCD_WriteDigit('W', 1);
	LCD_WriteDigit('E', 2);
	LCD_WriteDigit('E', 3);
	LCD_WriteDigit('P', 4);
	int primAng = 0;	//used to hold the angle of the servo with the lowest ADC value
	int primADC = 1000; //used to hold the lowest adc value. Light resistor value does not go above 1000, so all measured values 0-180 will have a value less than initial value.
	int currADC = 0;	//used to hold the measured ADC value atr each angle
	for(int i=0; i<10; i++) //0 to 180 in 20 degree increments
	{
		currADC = ADC_read();
		if(currADC <= primADC) //as light increases, ADC values decreases, so condition for looking for LOWEST value
		{
			primAng =  20*i; //Multiplies by 20 since full sweep increments by 20 degrees
			primADC = currADC;
		}
		if(i!=0) //Prevents rotation for first read at 0 degrees
			PWM_Rotate(FULLSWEEP,primAng);
		_delay_ms(1000);
	}
	
	//Displays the primary angle and mode to LCD
	char temp = primAng/100 + '0'; 
	LCD_WriteDigit(temp, 0);
	temp = primAng%100/10 + '0';
	LCD_WriteDigit(temp, 1);
	temp = primAng%10 + '0';
	LCD_WriteDigit(temp, 2);
	LCD_WriteDigit('F',3);
	LCD_WriteDigit('T',4);
	LCD_WriteDigit('L',5);
	_delay_ms(100);
	sei(); //reenable interrupts
	
	//localsweep
	int startAng = primAng;//Start angle is the primary angle after full sweep
	currADC = 0;
	while(1)//loops until next fullsweep
	{
		PWM_Rotate(LOCSWEEPS,startAng); //Sets servo position to primary angle at start of each cycle
		_delay_ms(1000);
		primADC = 1000; //Set to 1000 again b/c ADC value never goes above 1000
		if(startAng != 0) //Check if prim. angle is 0, if so, dont read for -10
		{
			//Read prim. angle - 10
			PWM_Rotate(LOCSWEEPM,startAng); //Moves servo to Prim. Angle - 10 position
			currADC = ADC_read();
			if(currADC <= primADC) //as light increases, ADC values decreases, so condition for looking for LOWEST value
			{
				primAng = startAng - 10;
				primADC = currADC;
			}
			_delay_ms(1000);
		}
		if(startAng != 180) //Check if prim. angle is 180, if so dont read for 190
		{
			//Read prim. angle + 10
			PWM_Rotate(LOCSWEEPP,startAng); //Moves servo to Prim. Angle + 10 position
			currADC = ADC_read();
			if(currADC <= primADC) //as light increases, ADC values decreases, so condition for looking for LOWEST value
			{
				primAng = startAng + 10;
				primADC = currADC;
			}
			_delay_ms(1000);
		}
		
		//Read prim. angle
		PWM_Rotate(LOCSWEEPS,startAng); //Moves servo back to start angle position
		currADC = ADC_read();
		if(currADC <= primADC) //as light increases, ADC values decreases, so condition for looking for LOWEST value
		{
			primAng = startAng;
			primADC = currADC;
		}
		_delay_ms(1000);
		
		//Displays new prim. angle and mode to LCD
		temp = primAng/100 + '0';
		LCD_WriteDigit(temp, 0);
		temp = primAng%100/10 + '0';
		LCD_WriteDigit(temp, 1);
		temp = primAng%10 + '0';
		LCD_WriteDigit(temp, 2);
		LCD_WriteDigit('F',3);
		LCD_WriteDigit('T',4);
		LCD_WriteDigit('L',5);
		_delay_ms(1000);
		
		startAng = primAng; //set new start angle for next local sweep cycle
	}
	return;
}

void ATL() //Avoid the light
{
	cli(); //Disable interrupts during fullsweep
	LCD_AllSegments(FALSE); 
	LCD_WriteDigit('S', 0); //Displays to LCD that servo/sensor is currently "SWEEP"ing
	LCD_WriteDigit('W', 1);
	LCD_WriteDigit('E', 2);
	LCD_WriteDigit('E', 3);
	LCD_WriteDigit('P', 4);
	int primAng = 0; //used to hold the angle of the servo with the lowest ADC value
	int primADC = 0; //used to hold the highest ADC value. Light resistor value does not go below 0, so all measured values 0-180 will have a value greater than initial value.
	int currADC = 0; //used to hold the measured ADC value atr each angle
	for(int i=0; i<10; i++) //0 to 180 in 20 degree increments
	{
		currADC = ADC_read();
		if(currADC >= primADC) //as light decreases, ADC values increases, so condition for looking for HIGHEST value
		{
			primAng =  20*i; //Multiplies by 20 since full sweep increments by 20 degrees
			primADC = currADC;
		}
		if(i!=0) //Prevents rotation for first read at 0 degrees
			PWM_Rotate(FULLSWEEP, primAng);
		_delay_ms(1000);
	}
	
	//Displays the primary angle and mode to LCD
	char temp = primAng/100 + '0';
	LCD_WriteDigit(temp, 0);
	temp = primAng%100/10 + '0';
	LCD_WriteDigit(temp, 1);
	temp = primAng%10 + '0';
	LCD_WriteDigit(temp, 2);
	LCD_WriteDigit('A', 3);
	LCD_WriteDigit('T', 4);
	LCD_WriteDigit('L', 5);
	_delay_ms(100);
	sei(); //reenable interrupts
	
	//localsweep
	int startAng = primAng;//Start angle is the prime angle after full sweep
	currADC = 0;
	while(1)//loops until next fullsweep
	{
		PWM_Rotate(LOCSWEEPS,startAng); //Set position to primary angle
		_delay_ms(1000);
		primADC = 0; 
		if(startAng != 0) //Check if prim. angle is 0, if so dont read for -10
		{
			//Read prim. angle - 10
			PWM_Rotate(LOCSWEEPM,startAng); //Moves servo to Prim. Angle - 10 position
			currADC = ADC_read();
			if(currADC >= primADC) //as light decreases, ADC values increases, so condition for looking for HIGHEST value
			{
				primAng = startAng - 10;
				primADC = currADC;
			}
			_delay_ms(1000);
		}
		if(startAng != 180) //Check if prim. angle is 180, if so dont read for 190
		{
			//Read prim. angle + 10
			PWM_Rotate(LOCSWEEPP,startAng); //Moves servo to Prim. Angle + 10 position
			currADC = ADC_read();
			if(currADC >= primADC) //as light decreases, ADC values increases, so condition for looking for HIGHEST value
			{
				primAng = startAng + 10;
				primADC = currADC;
			}
			_delay_ms(1000);
		}
			
		//Read prim. angle
		PWM_Rotate(LOCSWEEPS,startAng); //Moves servo back to start angle position
		currADC = ADC_read();
		if(currADC >= primADC) //as light decreases, ADC values increases, so condition for looking for HIGHEST value
		{
			primAng = startAng;
			primADC = currADC;
		}
		_delay_ms(1000);
		
		//Displays new prim. angle and mode to LCD
		temp = primAng/100 + '0';
		LCD_WriteDigit(temp, 0);
		temp = primAng%100/10 + '0';
		LCD_WriteDigit(temp, 1);
		temp = primAng%10 + '0';
		LCD_WriteDigit(temp, 2);
		LCD_WriteDigit('A',3);
		LCD_WriteDigit('T',4);
		LCD_WriteDigit('L',5);
		_delay_ms(1000);
		
		startAng = primAng; //set new start angle for next local sweep
 	}
	return;
}