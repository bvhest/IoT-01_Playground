from machine import Pin
import time
import random

# (blue) LED on board:
led = Pin(2, Pin.OUT)

# Green LED 1  on GPIO16, pin 25.
# Green LED 2  on GPIO17, pin 27.
# Yellow LED 1 on GPIO18, pin 35.
# Red LED 1    on GPIO19, pin 38.
led1 = Pin(16, Pin.OUT)
led2 = Pin(17, Pin.OUT)
led3 = Pin(18, Pin.OUT)
led4 = Pin(19, Pin.OUT)

leds = [led1, led2, led3, led4]

while True:
    randomValue = random.randint(0,4)
    led.value(not led.value())
    print("randomValue " + str(randomValue))
    led.value(not led.value())
    
    for i in range(0, len(leds)): 
        if randomValue > i:
            print("switching on LED number " + str(i))
            leds[i].value(1)
        else:
            print("switching off LED number " + str(i))
            leds[i].value(0)
        time.sleep(0.05)
    time.sleep(0.25)
