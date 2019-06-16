import machine
import time

n = 10

# (blue) on-board LED:
led = machine.Pin(2, machine.Pin.OUT)

for i in range(n):
    led.value(not led.value())
    time.sleep(0.5)

# één keer aan en uit per seconde: frequentie = 1 Hz