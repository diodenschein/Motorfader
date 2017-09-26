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

#define TWI_CMD_CHECKED_TXN             0x81        // Read/Write registers with simple checksum
#define TWI_CMD_WRITE_ENABLE            0x84        // Enable write of safe read/write registers
#define TWI_CMD_WRITE_DISABLE           0x85        // Disable write of safe read/write registers
#define TWI_CMD_REGISTERS_SAVE          0x86        // Save safe read/write registers fo EEPROM
#define TWI_CMD_REGISTERS_RESTORE       0x87        // Restore safe read/write registers from EEPROM
#define TWI_CMD_REGISTERS_DEFAULT       0x88        // Restore safe read/write registers to defaults
#define TWI_CMD_EEPROM_ERASE            0x89        // Erase the EEPROM.
#define TWI_CMD_POS_READ                0x90 
#define TWI_CMD_POS_READ_RAW            0x91 
#define TWI_CMD_TOUCH_READ              0x92 
#define TWI_CMD_TOUCH_READ_RAW          0x93 
#define TWI_CMD_VOLTAGE_READ            0x94 
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
  leds_init();
  registers_init();
  pid_init();
  ADC_Init();
  twi_slave_init(registers_read_byte(REG_TWI_ADDRESS));
  pid_registers_defaults();
  pwm_init();

  //So good so far 
//led_g_toggle();
  /*  HERE BE DRAGONS */
//This doesnt make a lot of sense just for testing

_delay_ms(500);
registers_write_word(REG_SEEK_POSITION_HI, REG_SEEK_POSITION_LO, 1020);
registers_write_word(REG_SEEK_VELOCITY_HI, REG_SEEK_VELOCITY_LO, 0);
_delay_ms(500);
registers_write_word(REG_SEEK_POSITION_HI, REG_SEEK_POSITION_LO, 512);
registers_write_word(REG_SEEK_VELOCITY_HI, REG_SEEK_VELOCITY_LO, 0);
_delay_ms(500);
registers_write_word(REG_SEEK_POSITION_HI, REG_SEEK_POSITION_LO, 5);
registers_write_word(REG_SEEK_VELOCITY_HI, REG_SEEK_VELOCITY_LO, 0);
uint16_t pos=0;
_delay_ms(5000);
  //    watchdog_hard_reset();
// while(1){
//   if(index>4)index=0;
// registers_write_word(REG_SEEK_POSITION_HI, REG_SEEK_POSITION_LO, posis[index]);
// index++;
// _delay_ms(1000);
// }

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
        if (twi_data_in_receive_buffer())
        {
            // Handle any TWI command.
            handle_twi_command();
        }
 // _delay_ms(100);
}
while(1){
  //registers_write_word(REG_SEEK_POSITION_HI, REG_SEEK_POSITION_LO, ADCS.avgTouch);
  _delay_ms(1000);
}
while(1){
  if(pos<4)pos=1023;
registers_write_word(REG_SEEK_POSITION_HI, REG_SEEK_POSITION_LO, pos);
pos -= 3;
_delay_ms(1);
if(pos == 510)_delay_ms(2000);
if(pos == 1020)_delay_ms(2000);
if(pos == 4)_delay_ms(2000);
}





 	//motor_go();
  //OCR1B = 222;
int del =0;
   while(1) {
      if(ADC_Read(4)<500){
        OCR1B=240;
        motor_up();
        del =0;
      }
      else if (ADC_Read(4)>520)
      {
        OCR1B=240;
        motor_down();
        del =0;
      }
      else{
        pwm_stop();
        del=1;
      }
  
   	 if(del)_delay_ms(5000);
   }
}
