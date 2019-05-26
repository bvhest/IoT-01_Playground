#
# meteo-station obv BME280 Digitale Barometer Druk en Vochtigheid Sensor Module
#
# bron: https://www.tinytronics.nl/shop/nl/sensoren/temperatuur-lucht-vochtigheid/bme280-digitale-barometer-druk-en-vochtigheid-sensor-module
# Een zeer compacte barometer die werkt via I2C of SPI. De BME280 is een 3-in-1 module
# die temperatuur, druk en vochtigheid kan meten.
#
# De module kan alleen gevoed worden met 3.3VDC. De I2C/SPI werkt dus ook met 3.3V en
# je hebt dus een level converter nodig bij gebruik van bijv. een 5V Arduino Uno.
#
# Het standaard I2C adres van deze module is 0x76. Dit moet mogelijk in de
# voorbeeldcode/library veranderd worden van 0x77 naar 0x76. Indien je de SDO pin
# verbind met Vcc, dan wordt het I2C adres 0x77.
#
# (Arduino) project met de ESP2866 en BME280 (nuttig voor aansluitingen) is te vinden op
#   https://core-electronics.com.au/projects/thingspeak-temperature-pressure-logger
#
# MicroPython library voor de BME280 en ESP2866 gevonden op GitHub:
#   https://github.com/robert-hh/BME280
#
# BvH, 25-05-2019
#


# LIBRARIES:
#
# We start by importing the Pin class from the machine library, as this will enable us to use
# the GPIO pins. We need to use a wait-time in the loop and import the sleep function from the
# time library.
from machine import Pin, I2C
from time import sleep
#
# this script assumes the default connection of the I2C bus
# On Wymos D1 mini devices that is SCL-to-D1 (pin5), SDA-to-D2 (pin4).
#
import bme280_i2c

# INITIALISATIE:
#
# Next we create an object called led which will store the GPIO pin that we wish to use, and
# whether it is an input or an output. In this case it is an output as we wish to light up the LED.
#
# see pinout on https://escapequotes.net/esp8266-wemos-d1-mini-pins-and-diagram/
# pin 16 = D0 (naar LED)
led = Pin(16, Pin.OUT)

# single blink
led.on()
sleep(0.5)
led.off()

# Initialise the i2c interface.
# pin 5 (= D1) SCL naar BME280-SCL.
# pin 4 (= D2) SDA naar BME280-SDA.
i2c = I2C(sda=Pin(4), scl=Pin(5))
bme = BME280_I2C(i2c=i2c)

# double blink
led.on()
sleep(0.5)
led.off()
led.on()
sleep(0.5)
led.off()

# All in an endless loop:
while True:
    # So now we need to turn on the LED, and it is as easy as this!
    led.on()
    # show BME280-measurements
    print(bme.get_measurement())
    # better version:
    #values = read_compensated_data(result = None)
    # wait
    sleep(0.5)
    # and turn off the LED
    led.off()
    # wait and measure approx. every 10 secs
    sleep(9.5)
