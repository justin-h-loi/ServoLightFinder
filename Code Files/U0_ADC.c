/*
 * U0_ADC.c
 *
 * Created: 11/2/2021 5:05:36 PM
 *  Author: Justin Loi - FL23090
 */ 
#include "U0_ADC.h"
#include <avr/io.h>

//input: None
//output: none
//Setups the ADC for reading
void ADC_init()
{
	//Refer to Section 21 in the datasheet for info on how ADC operates
	
	// AVR Butterfly Board Info:
	// The Neg. Temp. Coeff. resistor (NTC) is on ADC channel 0
	// The Board Edge Voltage Input Reading (VR) is on ADC channel 1
	// The Light Dependant Resistor (LDR) is on ADC channel 2

	//Refer to Section 21.9.1 in the data sheet for info on registers
    /* Setup ADC to use int 1.1V reference 
    and select LDR sensor channel */
    ADMUX = (1<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (0<<MUX3) | (0<<MUX2) | (1<<MUX1) | (0<<MUX0);

	//MUX3 - MUX0 are used to select the ADC channels
	//REFS1, REFS0 are used to select the reference voltage. In this case, we have selected the internal 1.1v

    /* Set conversion time to 
    //112usec = [(1/(8Mhz / 64)) * (14 ADC clocks  per conversion)]
     and enable the ADC*/
    ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADEN) | (0<<ADIE) | (1<<ADATE);
	//ADIE enables ADC interrupts
	//ADATE enables ADC auto trigger
	
	/* Perform Dummy Conversion to complete ADC init */
	ADCSRA |= (1<<ADSC);
	//Turning on ADSC starts a conversion, it turns off when it completes a conversion
	
	return;
}

//input: none
//output: int - returns the ADC value after reading
//reads and gets the full 10-bits of ADC
int ADC_read()
{
	int ldrLow_ADCR; //LDR value to hold ADCL
	int ldrHigh_ADCR; //LDR value to hold ADCH
	
	ADCSRA |= (1 << ADSC); //this is automatically cleared
	ldrLow_ADCR = ADCL;
	ldrHigh_ADCR = ADCH;
	
	return (ldrHigh_ADCR << 8 | ldrLow_ADCR); //Shifts bits from ADCH left 8 times, then combines with ADCL to create full 10 bit result
}