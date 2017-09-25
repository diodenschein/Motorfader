#include "led.h"
#include "config.h"
#include <avr/io.h>


void leds_init(void){
 DDRA |= (1 << LEDGR)|(1 << LEDY);
}

void led_g_on(void){
    PORTA|=(1<<LEDGR);
}
void led_g_off(){
    PORTA&=~(1<<LEDGR);
}
void led_g_toggle(){
	PORTA ^= (1 << LEDGR);
}

void led_y_toggle(){
	PORTA ^= (1 << LEDY);
}
void led_y_on(void){
    PORTA|=(1<<LEDY); 
}
void led_y_off(void){
    PORTA&=~(1<<LEDY);
}

