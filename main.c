#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include "config.h"
#include "led.h"
#include "adc.h"
#include "pwm.h"
#include "registers.h"
#include "twi.h"
#include "watchdog.h"
#include "eeprom.h"
#include "pid.h"
//#include "fuse8mhz.h"


static void handle_twi_command(void)
{
    uint8_t command;

    // Get the command from the receive buffer.
    command = twi_receive_byte();
    switch (command)
    {
        case TWI_CMD_RESET:
            // Reset the servo.
            watchdog_reset_hard();
            break;

        case TWI_CMD_PWM_ENABLE:
            // Enable PWM to the servo motor.
            pwm_enable();
            break;

        case TWI_CMD_PWM_DISABLE:
            // Disable PWM to the servo motor.
            pwm_disable();
            break;

        case TWI_CMD_WRITE_ENABLE:
            // Enable write to read/write protected registers.
            registers_write_enable();
            break;

        case TWI_CMD_WRITE_DISABLE:

            // Disable write to read/write protected registers.
            registers_write_disable();

            break;

        case TWI_CMD_REGISTERS_SAVE:

            // Save register values into EEPROM.
            eeprom_save_registers();

            break;

        case TWI_CMD_REGISTERS_RESTORE:

            // Restore register values into EEPROM.
            eeprom_restore_registers();

            break;

        case TWI_CMD_REGISTERS_DEFAULT:

            // Restore register values to factory defaults.
            registers_defaults();
            break;

        case TWI_CMD_EEPROM_ERASE:

            // Erase the EEPROM.
            eeprom_erase();

            break;

        case TWI_CMD_VOLTAGE_READ:

            // Request a voltage reading.
        //      adc_read_voltage();

            break;


        default:

            // Ignore unknown command.
            break;
    }
}



int main(void){
  watchdog_init();
  leds_init();
  registers_init();
  pid_init();
  ADC_Init();
  twi_slave_init(registers_read_byte(REG_TWI_ADDRESS));
  pid_registers_defaults();
  pwm_init();


while(1){
  if(ADCS.touch){
	 led_g_on();
  }
  else{
    led_g_off();
  }
  //registers_write_word(REG_SEEK_POSITION_HI, REG_SEEK_POSITION_LO, ADCS.avgTouch);
  //registers_write_word(REG_SEEK_POSITION_HI, REG_SEEK_POSITION_LO,   ADCS.rawTouch); 
        //if (usiTwiDataInReceiveBuffer())
  if (twi_data_in_receive_buffer()){
      // Handle any TWI command.
      handle_twi_command();
    }
}
}
