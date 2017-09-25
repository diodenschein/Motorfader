/********************************************************************************

Header file for the USI TWI Slave driver.

Created by Donald R. Blake
donblake at worldnet.att.net

---------------------------------------------------------------------------------

Created from Atmel source files for Application Note AVR312: Using the USI Module
as an I2C slave.

This program is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

---------------------------------------------------------------------------------

Change Activity:

    Date       Description
   ------      -------------
  15 Mar 2007  Created.

********************************************************************************/



#ifndef _USI_TWI_SLAVE_H_
#define _USI_TWI_SLAVE_H_



/********************************************************************************

                                    includes

********************************************************************************/

#include <stdbool.h>



#define TWI_CMD_RESET                   0x80        // Reset command
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


#define TWI_CMD_PWM_ENABLE              0x82        // Enable PWM to motors
#define TWI_CMD_PWM_DISABLE             0x83        // Disable PWM to servo motors

/********************************************************************************

                                   prototypes

********************************************************************************/

void    usiTwiSlaveInit( uint8_t );
void    usiTwiTransmitByte( uint8_t );
uint8_t usiTwiReceiveByte( void );
bool    usiTwiDataInReceiveBuffer( void );
void    (*_onTwiDataRequest)(void);
bool    usiTwiDataInTransmitBuffer(void);
uint8_t usiTwiAmountDataInReceiveBuffer(void);
// on_XXX handler pointers
void    (*usi_onRequestPtr)(void);
void    (*usi_onReceiverPtr)(uint8_t);


/********************************************************************************

                           driver buffer definitions

********************************************************************************/

// permitted RX buffer sizes: 1, 2, 4, 8, 16, 32, 64, 128 or 256

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE  ( 16 )
#endif
#define TWI_RX_BUFFER_MASK  ( TWI_RX_BUFFER_SIZE - 1 )

#if ( TWI_RX_BUFFER_SIZE & TWI_RX_BUFFER_MASK )
#  error TWI RX buffer size is not a power of 2
#endif

// permitted TX buffer sizes: 1, 2, 4, 8, 16, 32, 64, 128 or 256

#ifndef TWI_TX_BUFFER_SIZE
#define TWI_TX_BUFFER_SIZE ( 16 )
#endif
#define TWI_TX_BUFFER_MASK ( TWI_TX_BUFFER_SIZE - 1 )

#if ( TWI_TX_BUFFER_SIZE & TWI_TX_BUFFER_MASK )
#  error TWI TX buffer size is not a power of 2
#endif



#endif  // ifndef _USI_TWI_SLAVE_H_
