#
# code taken from https://core-electronics.com.au/tutorials/pycom-pysense-getting-started.html
#

from network import WLAN      # For operation of WiFi network
import time                   # Allows use of time.sleep() for delays
import pycom                  # Base library for Pycom devices
from umqtt import MQTTClient  # For use of MQTT protocol to talk to Adafruit IO
import ubinascii              # Needed to run any MicroPython code
import machine                # Interfaces with hardware components
import micropython            # Needed to run any MicroPython code

from pysense import Pysense   # Needed to operate anything on the Pysense expansion
#from LIS2HH12 import LIS2HH12 # Removed to minimise memory consumption
from SI7006A20 import SI7006A20
from LTR329ALS01 import LTR329ALS01
# from MPL3115A2 import MPL3115A2,ALTITUDE,PRESSURE # Removed to minimise memory consumption

# BEGIN SETTINGS
# These need to be change to suit your environment
# Randoms were sent to Adafruit IO for testing
RANDOMS_INTERVAL = 5000     # milliseconds
last_random_sent_ticks = 0  # milliseconds

# Wireless network
WIFI_SSID = "IoT"
WIFI_PASS = "Core123698745+" # No this is not our regular password. :)

# Adafruit IO (AIO) configuration
AIO_SERVER = "io.adafruit.com"
AIO_PORT = 1883
AIO_USER = "CoreChris"
AIO_KEY = "7b2d0601a9694589a52fb2d614122cff"
AIO_CLIENT_ID = ubinascii.hexlify(machine.unique_id())  # Can be anything
AIO_CONTROL_FEED = "CoreChris/feeds/lights"
AIO_RANDOMS_FEED = "CoreChris/feeds/randoms"
AIO_FEEDS = {'temp': 'CoreChris/feeds/temperature', 'humi': 'CoreChris/feeds/humidity'}

do_temp = True

py = Pysense()
# mp = MPL3115A2(py,mode=ALTITUDE) # Returns height in meters. Mode may also be set to PRESSURE, returning a value in Pascals
si = SI7006A20(py)
lt = LTR329ALS01(py)
# li = LIS2HH12(py)

# END SETTINGS

# RGBLED
# Disable the on-board heartbeat (blue flash every 4 seconds)
# We'll use the LED to respond to messages from Adafruit IO
pycom.heartbeat(False)
time.sleep(0.1) # Workaround for a bug.
                # Above line is not actioned if another
                # process occurs immediately afterwards
pycom.rgbled(0xff0000)  # Status red = not working

# WIFI
# We need to have a connection to WiFi for Internet access
# Code source: https://docs.pycom.io/chapter/tutorials/all/wlan.html

wlan = WLAN(mode=WLAN.STA)
wlan.connect(WIFI_SSID, auth=(WLAN.WPA2, WIFI_PASS), timeout=5000)

while not wlan.isconnected():    # Code waits here until WiFi connects
    machine.idle()

print("Connected to Wifi")
pycom.rgbled(0xffd7000) # Status orange: partially working

# FUNCTIONS

# Function to respond to messages from Adafruit IO
# def sub_cb(topic, msg):          # sub_cb means "callback subroutine"
#     print((topic, msg))          # Outputs the message that was received. Debugging use.
#     if msg == b"ON":             # If message says "ON" ...
#         pycom.rgbled(0xffffff)   # ... then LED on
#     elif msg == b"OFF":          # If message says "OFF" ...
#         pycom.rgbled(0x000000)   # ... then LED off
#     else:                        # If any other message is received ...
#         print("Unknown message") # ... do nothing but output that it happened.

# def random_integer(upper_bound):
#     return machine.rng() % upper_bound
#
# def send_random():
#     global last_random_sent_ticks
#     global RANDOMS_INTERVAL
#
#     if ((time.ticks_ms() - last_random_sent_ticks) < RANDOMS_INTERVAL):
#         return; # Too soon since last one sent.
#
#     some_number = random_integer(100)
#     print("Publishing: {0} to {1} ... ".format(some_number, AIO_RANDOMS_FEED), end='')
#     try:
#         client.publish(topic=AIO_RANDOMS_FEED, msg=str(some_number))
#         print("DONE")
#     except Exception as e:
#         print("FAILED")
#     finally:
#         last_random_sent_ticks = time.ticks_ms()
#

# Send temperature and humidity to Adafruit alternately to stay under rate limit
def send_sensors():
    temp = si.temperature()         # Temperature on Pysense
    humi = si.humid_ambient(temp)   # Calculated relative humidity given current temperature
    global do_temp                  # Use a variable defined for the whole program
    try:
        if do_temp:
            client.publish(topic=AIO_FEEDS['temp'], msg=str(temp))  # Send to Adafruit IO
            print("Temp sent")
        else:
            client.publish(topic=AIO_FEEDS['humi'], msg=str(humi))  # Send to Adafruit IO
            print("Humi sent")
        do_temp = not do_temp   # Aternate true/false value
    except Exception as e:
        print("FAILED")

# Use the MQTT protocol to connect to Adafruit IO
client = MQTTClient(AIO_CLIENT_ID, AIO_SERVER, AIO_PORT, AIO_USER, AIO_KEY)

# Subscribed messages will be delivered to this callback
# client.set_callback(sub_cb)
client.connect()    # Connects to Adafruit IO using MQTT
# client.subscribe(AIO_CONTROL_FEED)
# print("Connected to %s, subscribed to %s topic" % (AIO_SERVER, AIO_CONTROL_FEED))

pycom.rgbled(0x00ff00)    # Status green: online to Adafruit IO

# try:                      # Code between try: and finally: may cause an error
                          # so ensure the client disconnects the server if
                          # that happens.
while 1:              # Repeat this loop forever
    client.check_msg()# Action a message if one is received. Non-blocking.
    # send_random()     # Send a random number to Adafruit IO if it's time.
    send_sensors()      # Send temp and humidity to Adafruit IO
    for x in range (0, 5):
        time.sleep_ms(1000)
        blue, red = lt.light()
        print('{:5}-Blue  {:5}-Red    Range(0-65535)'.format(blue, red))


# finally:                  # If an exception is thrown ...
client.disconnect()   # ... disconnect the client and clean up.
client = None
wlan.disconnect()
wlan = None
pycom.rgbled(0x000022)# Status blue: stopped
print("Disconnected from Adafruit IO.")
