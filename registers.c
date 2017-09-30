#include <inttypes.h>
#include <string.h>

//#include "openservo.h"
#include "config.h"
#include "eeprom.h"
#include "ipd.h"
#include "pwm.h"
#include "registers.h"

// Register values.
uint8_t registers[REGISTER_COUNT];

void registers_init(void)
// Function to initialize all registers.
{
    // Initialize all registers to zero.
    memset(&registers[0], 0, REGISTER_COUNT);

    // Set device and software identification information.
    registers_write_byte(REG_DEVICE_TYPE, MFADER_DEVICE_TYPE);
    registers_write_byte(REG_DEVICE_SUBTYPE, MFADER_DEVICE_SUBTYPE);
    registers_write_byte(REG_VERSION_MAJOR, SOFTWARE_VERSION_MAJOR);
    registers_write_byte(REG_VERSION_MINOR, SOFTWARE_VERSION_MINOR);

    // Restore the read/write protected registers from EEPROM.  If the
    // EEPROM fails checksum this function will return zero and the
    // read/write protected registers should be initialized to defaults.
    if (!eeprom_restore_registers())
    {
        // Reset read/write protected registers to zero.
      //  memset(&registers[MIN_WRITE_PROTECT_REGISTER], WRITE_PROTECT_REGISTER_COUNT + REDIRECT_REGISTER_COUNT, REGISTER_COUNT);
//HÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ?
        // Initialize read/write protected registers to defaults.
        registers_defaults();
    }
}


void registers_defaults(void)
// Reset safe read/write registers to defaults.
{
    // Initialize read/write protected registers to defaults.

    // Default TWI address.
    registers_write_byte(REG_TWI_ADDRESS, REG_DEFAULT_TWI_ADDR);

    // Call the PWM module to initialize the PWM related default values.
    pwm_registers_defaults();

    // Call the PID module to initialize the PID related default values.
    ipd_registers_defaults();
}


uint16_t registers_read_word(uint8_t address_hi, uint8_t address_lo)
// Read a 16-bit word from the registers.
// Interrupts are disabled during the read.
{
    uint8_t sreg;
    uint16_t value;


    // Clear interrupts.
    asm volatile ("in %0,__SREG__\n\tcli\n\t" : "=&r" (sreg));

    // Read the registers.
    value = (registers[address_hi] << 8) | registers[address_lo];

    // Restore status.
    asm volatile ("out __SREG__,%0\n\t" : : "r" (sreg));

    return value;
}


void registers_write_word(uint8_t address_hi, uint8_t address_lo, uint16_t value)
// Write a 16-bit word to the registers.
// Interrupts are disabled during the write.
{
    uint8_t sreg;

    // Clear interrupts.
    asm volatile ("in %0,__SREG__\n\tcli\n\t" : "=&r" (sreg));

    // Write the registers.
    registers[address_hi] = value >> 8;
    registers[address_lo] = value;

    // Restore status.
    asm volatile ("out __SREG__,%0\n\t" : : "r" (sreg));
}


