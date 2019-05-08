# See https://docs.pycom.io for more information regarding library specifics

# technical documentation for Pysense-sensore in folder "IoT/pycom/documentation/"
from pysense import Pysense
# MPL3115A2 - I2C Precision Altimeter
from MPL3115A2 import MPL3115A2,ALTITUDE,PRESSURE
# SI7006A20 - I2C Humidity & Temperature sensor
from SI7006A20 import SI7006A20
# LTR-329ALS-01 - I2C Optical sensor
from LTR329ALS01 import LTR329ALS01
# LIS2HH12 - MEMS digital output motion sensor: ultra-low-power high-performance 3-axis "pico" accelerometer
from LIS2HH12 import LIS2HH12


# initialise board and sensors:
py = Pysense()
mpp = MPL3115A2(py, mode = PRESSURE) # Returns pressure in Pa. Mode may also be set to ALTITUDE, returning a value in meters
si = SI7006A20(py)
lt = LTR329ALS01(py)
li = LIS2HH12(py)

# on-board battery voltage:
print("Battery voltage: " + str(py.read_battery_voltage()))

# MPL3115A2-sensor: temperature & pressure reading, next pressure converted into altitude:
print("MPL3115A2 temperature: " + str(mpp.temperature()))
print("Pressure: " + str(mpp.pressure()))
mp = MPL3115A2(py, mode = ALTITUDE) # Returns height in meters. Mode may also be set to PRESSURE, returning a value in Pascals
print("Altitude: " + str(mp.altitude()))

# SI7006A20-sensor: temperature & humidity reading, next (derived) dewpoint and some unknown 'Humidity Ambient':
print("Temperature: " + str(si.temperature()) + " deg C")
print("Relative Humidity: " + str(si.humidity()) + " %RH")
print("Dew point: " + str(si.dew_point()) + " deg C")
t_ambient = 24.4
print("Humidity Ambient for " + str(t_ambient) + " deg C is " + str(si.humid_ambient(t_ambient)) + "%RH")

# LTR329ALS01-sensor: light intensity (lux) for red & blue channels:
print("Light (channel Blue lux, channel Red lux): " + str(lt.light()))

# LIS2HH12-sensor: acceleration, roll and pitch:
print("Acceleration: " + str(li.acceleration()))
print("Roll: " + str(li.roll()))
print("Pitch: " + str(li.pitch()))
