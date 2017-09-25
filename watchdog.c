#include <inttypes.h>
#include <avr/io.h>

#include "config.h"
#include "pwm.h"

void watchdog_init(void)
// Initialize the watchdog module.
{
    // Clear WDRF in MCUSR.
    MCUSR &= ~(1<<WDRF);

    // Write logical one to WDCE and WDE.
    WDTCR |= (1<<WDCE) | (1<<WDE);

    // Turn off WDT.
    WDTCR = 0x00;
}


void watchdog_reset_hard(void)
// Reset the device using the watchdog timer.
{
    // Disable PWM to the servo motor.
    pwm_disable();

    // Enable the watchdog.
    WDTCR = (1<<WDIF) |                                     // Reset any interrupt.
             (0<<WDIE) |                                     // Disable interrupt.
             (1<<WDE) |                                      // Watchdog enable.
             (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);  // Minimum prescaling - 16mS.

    // Wait for reset to occur.
    for (;;);
}

