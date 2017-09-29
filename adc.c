#include <avr/io.h>
#include <avr/interrupt.h>
#include "config.h"
#include "adc.h"
#include "structs.h"
#include "registers.h"
#include "pid.h"
#include "pwm.h"

volatile ADC_Status_t ADCS;

void ADC_Init(void){

	unsigned char sreg_saved;

	sreg_saved = SREG;
	cli();

	ADCS.Halt = FALSE; // Enable consecutive runs of ADC.

	DDRA&=~((1<<WIPER)|(0<<TSENSE));

	ADCS.Flag = FALSE;
	ADCS.MUX = 0x04;
	ADCS.touchState= 0x01;
	ADCS.tavgp = 0;
	ADCS.touch = 0;
	for (int i = 0; i < AVG_SIZE; ++i)
	{
		ADCS.touchavg[i]=0;
	}


	ADMUX =  ADCS.MUX;
	ADCSRB &= ~((1<<ADTS2) | (1<<ADTS1) | (1<<ADTS0)); 

	// Re-enable the ADC and ISR.
	ADCSRA=(1<<ADEN)|(1<<ADSC)|(1<<ADIE)|ADC_PRESCALER;
	sei();

	// Get a complete cycle of data before returning.
	ADC_Wait();

	SREG = sreg_saved;
} 


uint16_t ADC_Read( uint8_t channel ){
	uint16_t ret =0;
	switch (channel){

		case 0x03:
		ret = ADCS.rawTouch;
		break;

		case 0x04:
			ret = ADCS.rawWiper;
		break;


		default:  // Should not happen. (Invalid MUX-channel)
			ret = 0;
		break;
	}
return ret;                 //Returns the result (16-bit)
}

/*! \brief Waits for two full cycles of ADC-conversions to occur.
 *
 * This function clears the cycle complete-flag, then waits for it to be set
 * again. This is then repeated once before the function exits.
 *
 */
void ADC_Wait(void)
{
	// Clear ADC flag and wait for cycle to complete.
	ADCS.Flag = FALSE;
	do {
	} while (ADCS.Flag == FALSE);

	// Repeat, so we are sure the data belongs to the same cycle.
	ADCS.Flag = FALSE;
	do {
	} while (ADCS.Flag == FALSE);
}


ISR(ADC_vect)
{
		
unsigned char  Next,NextTouch, Signed;

	Signed = FALSE;  // Presume next conversion is unipolar.
	ADCSRA &= ~(1<<ADEN);  // Stop conversion before handling. This makes all
	  // conversions take at least 25 ADCCLK. (It is restarted later)

	// Handle the conversion, depending on what channel it is from, then
	// switch to the next channel in the sequence.
		switch (ADCS.MUX){
		// MUX = 0b000001 => ADC1 (PA1) = NTC
		case 0x03:
			switch (ADCS.touchState){
				// MUX = 0b000001 => ADC1 (PA1) = NTC
				case 0x01:
					PORTA ^= (0 << TSENSE)|(1 << TCHARGE); //pullup on
					Next = 0x1F; //discarge sampling cap
					NextTouch = 0x02;
				break;
				
				case 0x02:
					PORTA &= ~((0<<TSENSE)|(1 << TCHARGE));//Pullup off
					Next = 0x1F; //discarge sampling cap
					NextTouch = 0x03;
				break;

				case 0x03:
					ADCS.rawTouch = ADCW;
					ADCS.touchavg[ADCS.tavgp]=ADCS.rawTouch;
					ADCS.tavgp++;
					if(ADCS.tavgp > AVG_SIZE-1) ADCS.tavgp=0;
					long avg=0;
					for (int i = 0; i < AVG_SIZE; ++i)
					{
						if((ADCS.avgTouch > DEFAULT_TOUCH_MIN) && (ADCS.avgTouch < DEFAULT_TOUCH_MAX))
							avg++;
					}
					if( avg >= 5 ){ //&& (ADCS.rawTouch < DEFAULT_TOUCH_MAX)){
						ADCS.touch = 1;
					}
					else{
						ADCS.touch = 0;
					}
					registers_write_word(REG_TOUCH_RAW_HI, REG_TOUCH_RAW_LO, ADCS.rawTouch);
					registers_write_byte(REG_TOUCH,ADCS.touch);
					NextTouch = 0x01;
					Next=0x04;
				break;

				default:
					NextTouch = 0x01;
					Next=0x04;
				break;
			}
			ADCS.touchState = NextTouch;
		//	registers_write_word(REG_TOUCH_RAW_HI, REG_TOUCH_RAW_LO, ADCS.rawTouch);
		break;


		// MUX = 0b000010 => ADC2 (PA2) = RID
		case 0x04:
			ADCS.rawWiper = ADCW;
			registers_write_word(REG_POSITION_HI, REG_POSITION_LO, ADCS.rawWiper);
			registers_write_byte(REG_POSITION, ADCS.rawWiper >> 2);
			int16_t pwm = pid_position_to_pwm(ADCS.rawWiper);
			pwm_update(ADCS.rawWiper, pwm);
			ADCS.Flag = TRUE; //END OF CYCLE
			Next=0x03;
		break;

		case 0x1F: // READ GND
			Next=0x03;
		break;

		default:  // Should not happen. (Invalid MUX-channel)
			Next=0x03;  // Start at the beginning of sequence.
		break;
	}

	// Update MUX to next channel in sequence, set a bipolar conversion if
	// this has been flagged.
	ADCS.MUX = Next;
	ADMUX =  ADCS.MUX;

	if (Signed)	{
		ADCSRB |= (1<<BIN);
	} else {
		ADCSRB &= ~(1<<BIN);
	}

	// Re-enable the ADC unless a halt has been flagged and a conversion
	// cycle has completed.
	if (!((ADCS.Halt))) {
	//if (!((ADCS.Halt) && (ADCS.Flag))) {
		ADCSRA |= (1<<ADEN)|(1<<ADSC);
	}
}
