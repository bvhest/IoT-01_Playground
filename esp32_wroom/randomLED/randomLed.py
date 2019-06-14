#from machine import Pin
import time
import random

led1 = 1 #Pin(12, Pin.OUT)
led2 = 2 #Pin(13, Pin.OUT)
led3 = 3 #Pin(14, Pin.OUT)
led4 = 4 #Pin(15, Pin.OUT)

leds = [led1, led2, led3, led4]

while True:
    randomValue = random.randint(0,4)
    print("randomValue " + str(randomValue))
    
    for i in range(0, len(leds)): 
        if randomValue > i:
            print("switching on LED number " + str(i))
            #leds[i].value(1)
        else:
            print("switching off LED number " + str(i))
            #leds[i].value(0)
    time.sleep(1)
