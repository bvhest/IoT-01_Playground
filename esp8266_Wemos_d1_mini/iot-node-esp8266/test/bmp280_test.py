from machine import Pin, I2C
import bmp280


i2cbus = I2C(sda=Pin(4), scl=Pin(5))

i2cbus.scan()

bmp = bmp280.BMP280(address=0x76, i2c=i2cbus)

print("temperatuur: " + str(bmp.read_temperature()))
print("luchtdruk: " + str(bmp.read_pressure()))
