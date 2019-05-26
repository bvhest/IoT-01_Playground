## test i2c devices

from machine import Pin, I2C

i2c = I2C(sda=Pin(4), scl=Pin(5))

i2c.scan()
