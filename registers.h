
#ifndef _REGISTERS_H_
#define _REGISTERS_H_ 1

#include "config.h"
#include <inttypes.h>

//
// Control registers.

// TWI read/only status registers.  Writing
// values to these registers has no effect.

#define REG_DEVICE_TYPE             0x00
#define REG_DEVICE_SUBTYPE          0x01
#define REG_VERSION_MAJOR           0x02
#define REG_VERSION_MINOR           0x03
#define REG_FLAGS_HI                0x04
#define REG_FLAGS_LO                0x05
#define REG_POSITION_HI             0x06
#define REG_POSITION_LO             0x07
#define REG_VELOCITY_HI             0x08
#define REG_VELOCITY_LO             0x09
#define REG_PWM                     0x0A
#define REG_PWM_DIR                 0x0B
#define REG_TOUCH                   0x0C
#define REG_TOUCH_RAW_HI            0x0D
#define REG_TOUCH_RAW_LO            0x0E


// TWI read/write registers.  
#define REG_SEEK_POSITION_HI        0x0F
#define REG_SEEK_POSITION_LO        0x10
#define REG_SEEK_VELOCITY_HI        0x11
#define REG_SEEK_VELOCITY_LO        0x12

// TWI safe read/write registers. 

#define REG_TWI_ADDRESS             0x13
#define REG_PID_DEADBAND            0x14
#define REG_PID_PGAIN_HI            0x15
#define REG_PID_PGAIN_LO            0x16
#define REG_PID_DGAIN_HI            0x17
#define REG_PID_DGAIN_LO            0x18
#define REG_PID_IGAIN_HI            0x19
#define REG_PID_IGAIN_LO            0x1A

#define REG_PWM_FREQ_DIVIDER_HI     0x1B
#define REG_PWM_FREQ_DIVIDER_LO     0x1C
#define REG_MIN_SEEK_HI             0x1D
#define REG_MIN_SEEK_LO             0x1E
#define REG_MAX_SEEK_HI             0x1F
#define REG_MAX_SEEK_LO             0x20
#define REG_REVERSE_SEEK            0x21
#define REG_RESERVED_2F             0x22


//
// Define the register ranges.
//
#define MIN_READ_ONLY_REGISTER      0x00
#define MAX_READ_ONLY_REGISTER      0x0E

#define MIN_READ_WRITE_REGISTER     0x0F
#define MAX_READ_WRITE_REGISTER     0x12

#define MIN_WRITE_PROTECT_REGISTER  0x13
#define MAX_WRITE_PROTECT_REGISTER  0x1A

#define MIN_UNUSED_REGISTER         0x1B
#define MAX_UNUSED_REGISTER         0x22

#define MIN_REDIRECT_REGISTER       0x60
#define MAX_REDIRECT_REGISTER       0x6F
#define MIN_REDIRECTED_REGISTER     0x70
#define MAX_REDIRECTED_REGISTER     0x7F

// Define the total number of registers define.  This includes
// all registers except unused and redirected registers.
#define REGISTER_COUNT              (MIN_UNUSED_REGISTER + 16)

// Define the number of write protect registers.
#define WRITE_PROTECT_REGISTER_COUNT    (MAX_WRITE_PROTECT_REGISTER - MIN_WRITE_PROTECT_REGISTER + 1)

// Define the number of redirect registers.
#define REDIRECT_REGISTER_COUNT         (MAX_REDIRECT_REGISTER - MIN_REDIRECT_REGISTER + 1)

//
// Define the flag register REG_FLAGS_HI and REG_FLAGS_LO bits.
//

#define FLAGS_HI_RESERVED_07        0x07
#define FLAGS_HI_RESERVED_06        0x06
#define FLAGS_HI_RESERVED_05        0x05
#define FLAGS_HI_RESERVED_04        0x04
#define FLAGS_HI_RESERVED_03        0x03
#define FLAGS_HI_RESERVED_02        0x02
#define FLAGS_HI_RESERVED_01        0x01
#define FLAGS_HI_RESERVED_00        0x00

#define FLAGS_LO_RESERVED_07        0x07
#define FLAGS_LO_RESERVED_06        0x06
#define FLAGS_LO_RESERVED_05        0x05
#define FLAGS_LO_RESERVED_04        0x04
#define FLAGS_LO_POS_CHANGED        0x03
#define FLAGS_LO_TOUCH_CHANGED      0x02
#define FLAGS_LO_WRITE_ENABLED      0x01
#define FLAGS_LO_PWM_ENABLED        0x00

// Global register array.  Note: to minimize memory the register count doesn't
// include the unused and redirected registers.  For this reason care must be
// taken when referencing the redirect registers which come after the unused
// registers in this array.
extern uint8_t registers[REGISTER_COUNT];

// Register functions.

void registers_init(void);
void registers_defaults(void);
uint16_t registers_read_word(uint8_t address_hi, uint8_t address_lo);
void registers_write_word(uint8_t address_hi, uint8_t address_lo, uint16_t value);

// Register in-line functions.

// Read a single byte from the registers.
inline static uint8_t registers_read_byte(uint8_t address)
{
    return registers[address];
}


// Write a single byte to the registers.
inline static void registers_write_byte(uint8_t address, uint8_t value)
{
    registers[address] = value;
}


inline static void registers_write_enable(void)
{
    uint8_t flags_lo = registers_read_byte(REG_FLAGS_LO);

    // Enable PWM to the servo motor.
    registers_write_byte(REG_FLAGS_LO, flags_lo | (1<<FLAGS_LO_WRITE_ENABLED));
}


inline static void registers_write_disable(void)
{
    uint8_t flags_lo = registers_read_byte(REG_FLAGS_LO);

    // Disable PWM to the servo motor.
    registers_write_byte(REG_FLAGS_LO, flags_lo & ~(1<<FLAGS_LO_WRITE_ENABLED));
}


inline static uint8_t registers_is_write_enabled(void)
{
    return (registers_read_byte(REG_FLAGS_LO) & (1<<FLAGS_LO_WRITE_ENABLED)) ? 1 : 0;
}


inline static uint8_t registers_is_write_disabled(void)
{
    return (registers_read_byte(REG_FLAGS_LO) & (1<<FLAGS_LO_WRITE_ENABLED)) ? 0 : 1;
}


#endif // _OS_REGISTERS_H_

