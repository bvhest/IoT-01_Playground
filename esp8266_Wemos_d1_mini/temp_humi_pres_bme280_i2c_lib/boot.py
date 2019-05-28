#
# compact en misschien vervanging van deel van onderstaande code
#
#import connectWifi
#connectWiFi.connect()

# Complete project details at
#   https://randomnerdtutorials.com/micropython-mqtt-esp32-esp8266/
import machine
#import time
from ubinascii import hexlify
#import micropython
import network
#import esp
#esp.osdebug(None)
#import gc
#gc.collect()

ssid = 'onsVHifi'
password = 'FF41A972T3'
mqtt_server =  '37.187.106.16' # 'test.mosquitto.org', port = 1883
#EXAMPLE IP ADDRESS
#mqtt_server = '192.168.1.144'

client_id = hexlify(machine.unique_id())
topic_pub = b'achthoeven/meteo'
topic_sub = b'voorspelling'

last_message = 0
measure_interval = 15
message_interval = 60
counter = 0

print('Setting ip connection')
station = network.WLAN(network.STA_IF)
station.active(True)
station.connect(ssid, password)

while station.isconnected() == False:
    pass

print('Connection successful')
print(station.ifconfig())
