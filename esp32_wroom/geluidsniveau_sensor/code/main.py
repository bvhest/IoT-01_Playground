from machine import Pin, ADC
import time

print("Initialiseer ADC: adc2, ch0")
# esp32: Vout from microphone/pre-amp to ADC2 ch0 = GPIO4, pin 24.
#        Note: channel 2 kan alleen worden gebruikt als WiFi is uitgeschakeld.

adc = ADC(Pin(4),unit=2)
# esp8266
#adc = ADC(0)
#adc_c = adc.channel(pin='P34')
adc_c()
print("ADC klaar voor gebruik")


while True:
    v = adc_c.value()
    print("ADC output = " + str(v))

# zie https://docs.pycom.io/firmwareapi/pycom/machine/adc.html
adc = machine.ADC()             # create an ADC object
apin = adc.channel(pin='P16')   # create an analog pin on P16
val = apin()


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
