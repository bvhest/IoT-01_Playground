import sys, os, machine
if not "test" in sys.path:
    sys.path.append("/test")

import config

import network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(config.wifi_network, config.wifi_passwd)
signal_led = machine.Pin(2, machine.Pin.OUT) # built-in LED on D4
signal_led.value(0) # led on: active low

# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)

import gc
gc.collect()

#import webrepl
#webrepl.start()
