#ifndef STRUCTS_H
#define STRUCTS_H

#define AVG_SIZE 8
struct ADC_Status_struct
{
	unsigned char MUX : 5;  // Corresponds to ADMUX low bits MUX4..0. ADC state machine
	unsigned char Flag : 1;  // ADC cycle complete (TRUE/FALSE).
	unsigned char Halt : 1;  // Stop A/D-conversions (TRUE/FALSE).
	unsigned int rawWiper;  //  Raw, unconditioned Resistor Wiper sdata.
	unsigned int rawTouch;  // Raw, unconditioned touch data.
	unsigned int touch;
	unsigned int touchState; 
	unsigned int avgTouch;
	unsigned int touchavg[AVG_SIZE];
	unsigned int tavgp;
};
typedef struct ADC_Status_struct ADC_Status_t; //!< For convenience.
#endif // STRUCTS_H

