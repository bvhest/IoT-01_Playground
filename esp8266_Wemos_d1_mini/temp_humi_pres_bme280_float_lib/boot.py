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

client_id = hexlify(machine.unique_id())
#mqtt_server =  '37.187.106.16' # 'test.mosquitto.org', port = 1883
#mqtt_server = 'broker.hivemq.com' # TCP Port: 1883 , Websocket Port: 8000
#topic_pub = b'achthoeven/meteo'

# see https://learn.adafruit.com/adafruit-io/mqtt-api
mqtt_server = b'io.adafruit.com' # TCP Port: 1883 , Websocket Port: 8000
adafruit_user = b'uden'
adafruit_iokey = b'51eb40c2e2284473ac689ef1e4351b9e'
topic_pub = b'weerdata'

time_last_message = 0
measure_interval = 30
message_interval = 300 # 900 = één keer per 15 minuten

# setup WiFi connection:
station = network.WLAN(network.STA_IF)
station.active(True)
station.connect(ssid, password)

while station.isconnected() == False:
    pass

print('WiFi connection successful: ipconfig=')
print(station.ifconfig())
