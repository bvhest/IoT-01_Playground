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
import time
# using mqtt for exchanging data
from umqttsimple import MQTTClient
#
# this script assumes the default connection of the I2C bus
# On pycom devuces that is P9 = SDA, P10 = scl
#
import bme280_float

# Functies:
def do_blink(n=3):
    # tripple blink
    for x in range(n):
        led.on()
        time.sleep(0.5)
        led.off()

def get_BME280_measurements():
    return(bme.values)

def get_SZYTF_measurements():
    # SZYTF_capacitieve_bodem_vochtigheidssensor
    # Capacitieve vochtigheidsensor calibratie:
    value = 127.5415 - 0.2025 * adc.read()
    if value > 100:
        value = 100
    if value < 0:
        value = 0
    # return dict
    return {
       "soilmoisture": round(value, 2)
    }

# INITIALISATIE:
#
# Next we create an object called led which will store the GPIO pin that we wish to use, and
# whether it is an input or an output. In this case it is an output as we wish to light up the LED.
#
# see pinout on https://escapequotes.net/esp8266-wemos-d1-mini-pins-and-diagram/
# pin 16 = D0 (naar LED)
led = Pin(16, Pin.OUT)
# show succesfull
do_blink()

############################################
# Capacitieve vochtigheidsensor calibratie #
############################################

# initialiseer ADC op ADC0 (gpio2)
adc = machine.ADC(0)
# show succesfull
do_blink()

# Initialise the i2c interface. We use SCL-to-D1, SDA-to-D2.
# pin 5 (= D1) SCL naar BME280-SCL.
# pin 4 (= D2) SDA naar BME280-SDA.
i2c = I2C(sda=Pin(4), scl=Pin(5))
bme = bme280_float.BME280(i2c=i2c)
# show succesfull
do_blink()

# setup MQTT connection
def mqtt_connect_and_subscribe():
    global client_id, mqtt_server, topic_sub
    client = MQTTClient(client_id, mqtt_server)
    client.connect()
    print('Verbonden met %s mqtt-broker.' % mqtt_server)
    return client

def mqtt_send_message(msg):
    try:
        mqtt_client.publish(topic_pub, msg)
    except OSError as e:
        mqtt_restart_and_reconnect()

def mqtt_restart_and_reconnect():
    time.sleep(10)
    machine.reset()

try:
    mqtt_client = mqtt_connect_and_subscribe()
except OSError as e:
    print('Connectie met mqtt-broker is gefaald. Trigger machine.herstart. Error=' + e)
    mqtt_restart_and_reconnect()

# show succesfull
do_blink()

# oneindige loop om sensoren uit te lezen en meetresultaten door te geven:
while True:
    # ophalen RH-temp-druk-metingen:
    humPresTemp = get_BME280_measurements()
    # show succesfull
    do_blink(1)

    # ophalen bodemvochtigheids-metingen:
    soilMoisture = get_SZYTF_measurements()
    # show succesfull
    do_blink(1)

    # toon BME280- and SZYTF-meetresultaten
    payload = str(humPresTemp['temperature']) + ',' + str(humPresTemp['humidity']) + ',' + str(humPresTemp['pressure']) + ',' + str(soilMoisture['soilmoisture'])
    print('measurements : ' + payload)

    # verstuur meetresultaten naar mqtt-broker als wachttijd is verstreken
    if (time.time() - time_last_message) > message_interval:
        mqtt_send_message(payload)
        time_last_message = time.time()

    # zet wachttijd
    time.sleep(measure_interval-1)
