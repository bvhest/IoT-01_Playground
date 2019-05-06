#
# Voorbeeld temperature.py om een temperatuur-sensor uit te lezen.
# Deze opzet werkt voor de "BME280 Digitale Barometer Druk en Vochtigheid Sensor
#   Module - Model: BME280MOD"
#
# Bron:
#   - https://github.com/robert-hh/BME280
#
# Notes
#  - the BME280 uses I2C which should be hardware independent. A possible cause
#    of problems is that I2C requires pullup resistors on the two I2C wires. I
#    don't know whether PyCom provide these on their boards or whether you have
#    to supply them. You could try fitting 4.7KΩ resistors to 3.3V.
#
# BvH 2019-05-06
#
import time
from machine import Pin
from onewire import OneWire
from ds18x20 import DS18X20

class TemperatureSensor:
    """
    The class TemperatureSensor represents a Temperature sensor.
    """
    def __init__(self, pin):
        """
        Finds address of single DS18B20 on bus specified by `pin`
        :param pin: 1-Wire bus pin
        :type pin: int
        """
        self.ds = DS18X20(OneWire(Pin(pin)))
        addrs = self.ds.scan()
        if not addrs:
            raise Exception('no DS18B20 found at bus on pin %d' % pin)
        # save what should be the only address found
        self.addr = addrs.pop()

    def read_temp(self, fahrenheit=False):
        """
        Reads temperature from a single DS18X20
        :param fahrenheit: Whether or not to return value in Fahrenheit
        :type fahrenheit: bool
        :return: Temperature
        :rtype: float
        """
        self.ds.convert_temp()
        time.sleep_ms(750)
        temp = self.ds.read_temp(self.addr)
        if fahrenheit:
            return self.c_to_f(temp)
        return temp

    @staticmethod
    def c_to_f(c):
        """
        Converts Celsius to Fahrenheit
        :param c: Temperature in Celsius
        :type c: float
        :return: Temperature in Fahrenheit
        :rtype: float
        """
        return (c * 1.8) + 32

# example
#
# this script assumes the default connection of the I2C bus
# On pycom devuces that is P9 = SDA, P10 = scl
#
import machine
import bme280_float

i2c = machine.I2C()
bme = bme280_float.BME280(i2c=i2c)

print(bme.values)
