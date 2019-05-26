from umqtt.simple import MQTTClient
from machine import Pin
import ubinascii
import machine
import micropython
import config


# ESP8266 ESP-12 modules have blue, active-low LED on GPIO2, replace
# with something else if needed.
led = Pin(2, Pin.OUT, value=1)

# Default MQTT server to connect to: config.mqtt_server
mac_addr = ubinascii.hexlify(wlan.config('mac')).decode("utf-8")
clientname = "client" + mac_addr[-6:]
topic = "node/" + mac_addr[-4:] + "/led"
print("client: " + clientname)
print("topic: " + topic)


state = 0

def sub_cb(topic, msg):
    global state
    print((topic, msg))
    if msg == b"on":
        led.value(0)
        state = 1
    elif msg == b"off":
        led.value(1)
        state = 0
    elif msg == b"toggle":
        # LED is inversed, so setting it to current state
        # value will make it toggle
        led.value(state)
        state = 1 - state


def main(server=config.mqtt_server):
    c = MQTTClient(clientname, server)
    # Subscribed messages will be delivered to this callback
    c.set_callback(sub_cb)
    c.connect()
    c.subscribe(topic)
    print("Connected to %s, subscribed to %s topic" % (server, topic))

    try:
        while 1:
            #micropython.mem_info()
            c.wait_msg()
    finally:
        c.disconnect()

if __name__ == "__main__":
    main()
