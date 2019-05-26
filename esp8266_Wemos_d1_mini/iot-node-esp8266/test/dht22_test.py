# test bmp280 and dht22

from machine import Pin
import dht

dht22 = dht.DHT22(Pin(14)) # is D5 on NodeMCU

dht22.measure()
print("temp: " + str(dht22.temperature()))
print("hum: " + str(dht22.humidity()))
