
#ifndef _EEPROM_H_
#define _EEPROM_H_ 1

#define EEPROM_VERSION      0x01

uint8_t eeprom_erase(void);
uint8_t eeprom_restore_registers(void);
uint8_t eeprom_save_registers(void);

#endif // _EEPROM_H_
