import machine
import time

# (blue) on-board LED:
led = machine.Pin(2, machine.Pin.OUT)

N = 100000

t0 = time.ticks_us()

# recht-toe-reacht-aan loopje met functie-aanroep om LED aan/uit te zetten:
for i in range(N):
    led.value(not led.value())


t1 = time.ticks_us()
dt = time.ticks_diff(t1, t0)
fmt = "{:5.3f} s, {:6.3f} uSec/blink : {:8.2f} kHz/s"
print(fmt.format(dt * 1e-6, dt/N, N/dt * 1e3))

# 4.202 s, 42.019 uSec/blink :    23.80 kHz/s