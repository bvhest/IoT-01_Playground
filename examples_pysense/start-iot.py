#
# taken from https://startiot.telenor.com/learning/pysense-quick-start-guide/
#

from network import LoRa
import socket
import time
import binascii
import pycom

from lib.MPL3115A2 import MPL3115A2
from lib.LTR329ALS01 import LTR329ALS01
from lib.SI7006A20 import SI7006A20
from lib.LIS2HH12 import LIS2HH12

pycom.heartbeat(False)
pycom.rgbled(0x000000)

# Initialize LoRa in LORAWAN mode.
lora = LoRa(mode=LoRa.LORAWAN, adr=True)

# create an OTA authentication params (obv TheThingsNetwork account obv hestbv-user)
dev_eui = binascii.unhexlify('70B3D5499AB2EE1E')
app_eui = binascii.unhexlify('70B3D57ED001C09E')
app_key = binascii.unhexlify('8D4DFBB4DE149AC387D43EBBC51B1A28')

# join a network using OTAA (Over the Air Activation)
lora.join(activation=LoRa.OTAA, auth=(dev_eui, app_eui, app_key), timeout=0)

# wait until the module has joined the network
count = 0
while not lora.has_joined():
    pycom.rgbled(0xff0000)
    time.sleep(2.5)
    pycom.rgbled(0x000000)
    print("Not yet joined count is:" ,  count)
    count = count + 1

# create a LoRa socket
pycom.rgbled(0x0000ff)
time.sleep(0.1)
pycom.rgbled(0x000000)
time.sleep(0.1)
pycom.rgbled(0x0000ff)
time.sleep(0.1)
pycom.rgbled(0x000000)

s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

# set the LoRaWAN data rate
s.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)

# make the socket non-blocking
s.setblocking(False)

# create a raw LoRa socket
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
s.setblocking(True)

# init the libraries
temp = MPL3115A2()
lux = LTR329ALS01()
multi = SI7006A20()
accel = LIS2HH12()

while True:
    accel.read()
    # Read data from the libraries and place into string
    data = "%.2f %.2f %.2f %.2f %.2f %.2f %.2f" % (temp.temp(), multi.temp(), lux.lux()[0], multi.humidity(), accel.roll(), accel.pitch(), accel.yaw())
    print("Sending %s" % data)
    # send the data over LPWAN network
    s.send(data)
    time.sleep(3)
