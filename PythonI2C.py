
from machine import I2C
from machine import Pin
i2c = I2C( sda=Pin(26), scl=Pin(27), freq=400000 )
i2c.scan()
i2c.writeto(0x10, b'\x80') // reset 
i2c.writeto(0x10, b'\x83') // disable PWM 
i2c.writeto(0x10, b'\x82') // enable PWM 
i2c.writeto_mem(0x10, 0x13, b'\x03\xff')
i2c.readfrom_mem(0x10, 0x08, 11)