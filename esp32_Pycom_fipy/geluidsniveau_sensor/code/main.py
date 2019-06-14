from machine import Pin, ADC

print("Initialiseer ADC: adc1, ch0")
# esp32
adc = ADC(Pin(34))
# esp8266
#adc = ADC(0)
#adc_c = adc.channel(pin='P34')
adc_c()
print("ADC klaar voor gebruik")


while True:
    v = adc_c.value()
    print("ADC output = " + str(v))

# zie https://docs.pycom.io/firmwareapi/pycom/machine/adc.html
import machine
adc = machine.ADC()             # create an ADC object
apin = adc.channel(pin='P16')   # create an analog pin on P16
val = apin()
