
#ifndef _CONFIG_H_
#define _CONFIG_H_ 1

#include <inttypes.h>

#define F_CPU 		8000000UL

#define FALSE 0
#define TRUE 1

#define LEDY PA6
#define LEDGR PA7
#define MOTGO PB3
#define MOTDIR PB4

#define WIPER PA5
#define ADC_CHANNEL_WIPER 4

#define TCHARGE PA3
#define TSENSE PA4
#define ADC_CHANNEL_TOUCH 3


// Default pwm frequency divider.
#define DEFAULT_PWM_FREQ_DIVIDER        0x0040

// Device type and subtype.  
#define MFADER_DEVICE_TYPE           2
#define MFADER_DEVICE_SUBTYPE        2

// Software major and minor version.  
#define SOFTWARE_VERSION_MAJOR          0
#define SOFTWARE_VERSION_MINOR          0

// The default TWI address. 
#define REG_DEFAULT_TWI_ADDR        0x10

//Change motor direction
#define SWAP_PWM_DIRECTION_ENABLED  0

#define DEFAULT_PID_PGAIN               0x2100
#define DEFAULT_PID_DGAIN               0x0900
#define DEFAULT_PID_IGAIN               0x0000
#define DEFAULT_PID_DEADBAND            0x00

// Specify default mininimum and maximum seek positions.  
#define DEFAULT_MIN_SEEK                0x0000
#define DEFAULT_MAX_SEEK                0x03FF


// Enable (1) or disable (0) checksuming
#define TWI_CHECKED_ENABLED         0


#define DEFAULT_TOUCH_MIN   600
#define DEFAULT_TOUCH_MAX   1023


//
// Utility functions.
//

// Disable interrupts and returns SREG value used to restore interrupts.
inline static uint8_t disable_interrupts(void)
{
    uint8_t sreg;

    asm volatile (
        "in %0,__SREG__\n\t"
        "cli\n\t"
        : "=r" ((uint8_t) sreg)
        :
    );

    return sreg;
}

// Restore interrupts Enables interrupts according to the SREG.
inline static void restore_interrupts(uint8_t sreg)
{
    asm volatile (
        "out __SREG__,%0\n\t"
        :
        : "r" ((uint8_t) sreg)
    );
}

#endif // _F_CONFIG_H_
