from umqtt.simple import MQTTClient
import ubinascii
import time
import ujson
import config
import machine


import network
wlan = network.WLAN(network.STA_IF)
# WiFI connection in boot.py

mac_addr = ubinascii.hexlify(wlan.config('mac')).decode("utf-8")
clientname = "client" + mac_addr[-6:]

mqtt = MQTTClient(clientname, config.mqtt_server)

topic = "node/" + mac_addr[-4:] + "/sensors"

while not wlan.isconnected():
    time.sleep(2)
    wlan.active(True)
    wlan.connect(config.wifi_network, config.wifi_passwd)

# sensors
from machine import Pin, I2C, ADC
import bmp280
import dht

i2cbus = I2C(sda=Pin(4), scl=Pin(5))
bmp = bmp280.BMP280(address=0x76, i2c=i2cbus)

dht22 = dht.DHT22(Pin(14)) # is D5 on NodeMCU

ldr = ADC(0)

#
def send_sensordata():
    data = {}
    dht22.measure()
    data["dhtTemp"] = dht22.temperature()
    data["dhtHum"] = dht22.humidity()

    data["bmpTemp"] = bmp.read_temperature()
    data["bmpPres"] = bmp.read_pressure()

    data["light"] = ldr.read()

    data["id"] = mac_addr[-4:]

    data["localtime"] = time.ticks_ms()

    mqtt.publish(topic, ujson.dumps(data))

mqtt.connect()
machine.Pin(2).value(1) # built-in LED off (active low)

while True:
    send_sensordata()
#   check_msg()
    time.sleep(60)
