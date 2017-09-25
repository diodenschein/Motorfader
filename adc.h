#ifndef _ADC_H_
#define _ADC_H_ 1
#include "structs.h"

uint16_t ADC_Read( uint8_t channel );

#define ADC_PRESCALER     0x05

extern volatile ADC_Status_t ADCS;

void ADC_Wait(void);
void ADC_Init(void);

#endif // _ADC_H_