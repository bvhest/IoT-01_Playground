from machine import Pin, ADC
import time

print("Initialiseer ADC: adc2, ch0")
# see
#   - https://docs.micropython.org/en/latest/esp32/quickref.html
#   - https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo/wiki/adc
#
# esp32: Vout from microphone/pre-amp to ADC2 ch0 = GPIO4, pin 24.
#        Note: channel 2 kan alleen worden gebruikt als WiFi is uitgeschakeld.
adc = ADC(Pin(36, Pin.IN), unit = 1) # ADC1, ch0
# configureer bereik van 0-3.3 V (3.6 max)
adc.atten(adc.ATTN_11DB)
# 12 bits resolutie
adc.width(adc.WIDTH_12BIT)
print(adc.read())
print("ADC klaar voor gebruik")

# (blue) LED on board:
led = Pin(2, Pin.OUT)

# Green LED 1  on GPIO16, pin 25.
# Green LED 2  on GPIO17, pin 27.
# Yellow LED 1 on GPIO18, pin 35.
# Red LED 1    on GPIO19, pin 38.
led1 = Pin(25, Pin.OUT)
led2 = Pin(26, Pin.OUT)
led3 = Pin(27, Pin.OUT)
led4 = Pin(14, Pin.OUT)

leds = [led1, led2, led3, led4]

while True:
    led.value(not led.value())
    v = adc.read()
    print("ADC output = " + str(v))
#    adc.collect(18000 ,len=9000)
#    v = adc.collected()
#    print("ADC output = " + str(v))
    scaledV = int(round(v / 1024))
    print("scaled output = " + str(scaledV))
    led.value(not led.value())
#    
#    for i in range(0, len(leds)): 
#        if scaledV > i:
#            print("switching on LED number " + str(i))
#            leds[i].value(1)
#        else:
#            print("switching off LED number " + str(i))
#            leds[i].value(0)
#        time.sleep(0.05)
    time.sleep(1)
