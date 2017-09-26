#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "config.h"
#include "pwm.h"
#include "led.h"
#include "registers.h"

// Determine the top value for timer/counter1 from the frequency divider.
#define PWM_TOP_VALUE(div)      ((uint16_t) div << 4) - 1;

// Determines the compare value associated with the duty cycle for timer/counter1.
#define PWM_OCRN_VALUE(div,pwm) (uint16_t) (((uint32_t) pwm * (((uint32_t) div << 4) - 1)) / 255)

// Flags that indicate PWM output in A and B direction.
static uint8_t pwm_0;

// Pwm frequency divider value.
static uint16_t pwm_div;

#define MAX_POSITION  (1023)

//
// The delay_loop function is used to provide a delay. The purpose of the delay is to
// allow changes asserted at the AVRs I/O pins to take effect in the H-bridge (for example
// turning on or off one of the MOSFETs). This is to prevent improper states from occurring
// in the H-bridge and leading to brownouts or "crowbaring" of the supply. This was more
// of a problem, prior to the introduction of the delay, with the faster processor clock
// rate that was introduced with the V3 boards (20MHz) than it was with the older8Hhz V2.1
// boards- there was still a problem with the old code, with that board, it was just less
// pronounced.
//
// NOTE: Lower impedance motors may increase the lag of the H-bridge and thus require longer
//       delays.
//


void pwm_registers_defaults(void)
// Initialize the PWM algorithm related register values.  This is done
// here to keep the PWM related code in a single file.
{
    // PWM divider is a value between 1 and 1024.  This divides the fundamental
    // PWM frequency (500 kHz for 8MHz clock, 1250 kHz for 20MHz clock) by a
    // constant value to produce a PWM frequency suitable to drive a motor.  A
    // small motor with low inductance and impedance
    // will typically use a divider value between 16 and 64.  A larger
    // motor with higher inductance and impedance may require a greater divider.
    pwm_enable();
    registers_write_word(REG_PWM_FREQ_DIVIDER_HI, REG_PWM_FREQ_DIVIDER_LO, DEFAULT_PWM_FREQ_DIVIDER);
}


void pwm_init(void)
// Initialize the PWM module for controlling a DC motor.
{
    // Initialize the pwm frequency divider value.
    pwm_div = registers_read_word(REG_PWM_FREQ_DIVIDER_HI, REG_PWM_FREQ_DIVIDER_LO);

	DDRB |= (1 << MOTDIR)|(1 << MOTGO);
	PORTB&=~(1<<MOTGO);
    // Set timer top value.
 //   ICR1 = PWM_TOP_VALUE(pwm_div);

    // Set the PWM duty cycle to zero.
    OCR1A = 0;
    OCR1B = 0;

    // Configure timer 1 for PWM, Phase and Frequency Correct operation, but leave outputs disabled.
  	// Clear OC1B on compare match, enable PWM on comparator OCR1B.
	TCCR1A = (1<<COM1B1)|(0<<COM1B0)|(1<<PWM1B);

	// Non-inverted PWM, T/C stopped.
	TCCR1B = 0;

	// Copy shadow bits, disconnect OC1D.
	TCCR1C = (TCCR1A & 0xF0);

	// No fault protection, use phase & frequency correct PWM.
	TCCR1D = (0<<WGM11)|(1<WGM10);

	// TOP value for PWM, f(PWM) = 64MHz / 255 = 251kHz.
	OCR1C = 0xFF;
	// No dead time values.
	DT1 = 0;


	// CLK PCK = 8MHz / 2 = 4MHz.
	TCCR1B |= (0<<CS13)|(0<<CS12)|(0<<CS11)|(1<<CS10);

    // Update the pwm values.
    registers_write_byte(REG_PWM, 0);
    TIMSK |= (1<<TOV1);
    sei();
}


void pwm_update(uint16_t position, int16_t pwm)
// Update the PWM signal being sent to the motor.  The PWM value should be
// a signed integer in the range of -255 to -1 for clockwise movement,
// 1 to 255 for counter-clockwise movement or zero to stop all movement.
// This function provides a sanity check against the position and
// will prevent the from being driven past a minimum and maximum
// position.
{
	//registers_write_byte(REG_REVERSE_SEEK,0); 
    uint8_t pwm_width = 0;
    uint16_t min_position;
    uint16_t max_position;

    // Quick check to see if the frequency divider changed.  If so we need to
    // configure a new top value for timer/counter1.  This value should only
    // change infrequently so we aren't too elegant in how we handle updating
    // the value.  However, we need to be careful that we don't configure the
    // top to a value lower than the counter and compare values.
    if (registers_read_word(REG_PWM_FREQ_DIVIDER_HI, REG_PWM_FREQ_DIVIDER_LO) != pwm_div)
    {
        // Hold EN_A (PD2) and EN_B (PD3) low.
    //    PORTD &= ~((1<<PD2) | (1<<PD3));

        // Give the H-bridge time to respond to the above, failure to do so or to wait long
        // enough will result in brownouts as the power is "crowbarred" to varying extents.
        // The delay required is also dependant on factors which may affect the speed with
        // which the MOSFETs can respond, such as the impedance of the motor, the supply
        // voltage, etc.
        //
        // Experiments (with an "MG995") have shown that 5microseconds should be sufficient
        // for most purposes.
        //
    //    delay_loop(DELAYLOOP);

        // Disable OC1B outputs.
      //  TCCR1A &= ~((1<<COM1B1) | (1<<COM1B0));

        // Make sure that PWM_A (PB1/OC1A) and PWM_B (PB2/OC1B) are held low.
       // PORTB &= ~((1<<MOTGO) | (1<<MOTDIR));

        // Reset the A and B direction flags.
		pwm_0 = 0;

        // Update the pwm frequency divider value.
        pwm_div = registers_read_word(REG_PWM_FREQ_DIVIDER_HI, REG_PWM_FREQ_DIVIDER_LO);

        // Update the timer top value.
       // ICR1 = PWM_TOP_VALUE(pwm_div);

        // Reset the counter and compare values to prevent problems with the new top value.
        TCNT1 = 0;
        OCR1A = 0;
        OCR1B = 0;
    }

    // Are we reversing the seek sense?
    if (registers_read_byte(REG_REVERSE_SEEK) != 0)
    {
        // Yes. Swap the minimum and maximum position.
        // Get the minimum and maximum seek position.
        min_position = registers_read_word(REG_MAX_SEEK_HI, REG_MAX_SEEK_LO);
        max_position = registers_read_word(REG_MIN_SEEK_HI, REG_MIN_SEEK_LO);

        // Make sure these values are sane 10-bit values.
        if (min_position > MAX_POSITION) min_position = MAX_POSITION;
        if (max_position > MAX_POSITION) max_position = MAX_POSITION;

        // Adjust the values because of the reverse sense.
        min_position = MAX_POSITION - min_position;
        max_position = MAX_POSITION - max_position;
    }
    else
    {
        // No. Use the minimum and maximum position as is.
        // Get the minimum and maximum seek position.
        min_position = registers_read_word(REG_MIN_SEEK_HI, REG_MIN_SEEK_LO);
        max_position = registers_read_word(REG_MAX_SEEK_HI, REG_MAX_SEEK_LO);

        // Make sure these values are sane 10-bit values.
        if (min_position > MAX_POSITION) min_position = MAX_POSITION;
        if (max_position > MAX_POSITION) max_position = MAX_POSITION;
    }

    // Disable clockwise movements when position is below the minimum position.
    if ((position < min_position) && (pwm < 0)){
    	    	
    	    	pwm = 0;
    } 

    // Disable counter-clockwise movements when position is above the maximum position.
    if ((position > max_position) && (pwm > 0)) pwm = 0;

    // Determine if PWM is disabled in the registers.
    if (!(registers_read_byte(REG_FLAGS_LO) & (1<<FLAGS_LO_PWM_ENABLED))) pwm = 0;

    // Determine direction of motor movement or stop.
    if (pwm < -150)
    {
        // Less than zero. Turn clockwise.

        // Get the PWM width from the PWM value.
        pwm_width = (uint8_t) -pwm;

        // Turn clockwise.
#if SWAP_PWM_DIRECTION_ENABLED
        motor_up();
#else
        motor_down();
#endif
		OCR1B = pwm_width;
    }
    else if (pwm > 150)
    {
        // More than zero. Turn counter-clockwise.

        // Get the PWM width from the PWM value.
        pwm_width = (uint8_t) pwm;
        // Turn counter-clockwise.
#if SWAP_PWM_DIRECTION_ENABLED
        motor_down();
#else
        motor_up();
#endif
        OCR1B = pwm_width;

    }
    else
    {
        // Stop all PWM activity to the motor.
        pwm_stop();
    }
        // Save the pwm A and B duty values.
    registers_write_byte(REG_PWM, pwm_width);
}


void pwm_stop(void)
// Stop all PWM signals to the motor.
{
    // Disable interrupts.
    cli();



    // Make sure that PWM_A (PB1/OC1A) and PWM_B (PB2/OC1B) are held low.
    PORTB &= ~((1<<MOTDIR) | (1<<MOTGO));

    // Reset the A and B direction flags.
    pwm_0 = 0;

    // Set the PWM duty cycle to zero.
    OCR1B = 0;

    // Restore interrupts.
    sei();

    // Save the pwm A and B duty values.
    registers_write_byte(REG_PWM, pwm_0);
}


volatile uint8_t count = 0;
volatile uint8_t count1 = 0;

ISR(TIMER1_OVF_vect) 
{
	count++;
if(count>=250){
	count1++;
	count=0;
}
if(count1>=20){
	led_y_toggle();	
	count=0;
	count1=0;
}
}










void motor_init(void){
	DDRB |= (1 << MOTDIR)|(1 << MOTGO);
}


void motor_stop(void){
	PORTB&=~(1<<MOTGO);
	PORTB&=~(1<<MOTDIR);
}

void motor_down(void){
	PORTB&=~(1<<MOTDIR);
	    registers_write_byte(REG_PWM_DIR, 0);
}
void motor_up(void){
	PORTB|=(1<<MOTDIR);
	    registers_write_byte(REG_PWM_DIR, 1);
}
void motor_go(void){
	PORTB|=(1<<MOTGO);
}
