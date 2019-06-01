# Complete project details at
#   https://randomnerdtutorials.com/micropython-mqtt-esp32-esp8266/
import machine
#import time
from ubinascii import hexlify
#import micropython
import network

# variables
ssid = 'onsVHifi'
password = 'FF41A972T3'

#mqtt_server =  '37.187.106.16' # 'test.mosquitto.org', port = 1883
mqtt_server = 'broker.hivemq.com' # TCP Port: 1883 , Websocket Port: 8000

client_id = hexlify(machine.unique_id())
topic_pub = b'achthoeven/meteo'
topic_sub = b'voorspelling'

time_last_message = 0
measure_interval = 60
message_interval = 900 # één keer per 15 minuten

# setup WiFi connection:
station = network.WLAN(network.STA_IF)
station.active(True)
station.connect(ssid, password)

while station.isconnected() == False:
    pass

print('WiFi connection successful: ipconfig=')
print(station.ifconfig())
