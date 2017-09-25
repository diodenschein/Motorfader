
from machine import I2C
from machine import pin
i2c = I2C( sda=Pin(26), scl=Pin(27), freq=400000 )
i2c.scan()
i2c.writeto(0x10, b'\x80') // reset 
i2c.writeto(0x10, b'\x83') // disable PWM 
i2c.writeto(0x10, b'\x82') // enable PWM 