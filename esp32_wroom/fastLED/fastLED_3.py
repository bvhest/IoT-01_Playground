import machine
import time

# (blue) on-board LED:
led = machine.Pin(2, machine.Pin.OUT)

N = 100000

t0 = time.ticks_us()

# recht-toe-reacht-aan loopje: LED direct aan en uit zetten.
for i in range(N):
    led.on()
    led.off()


t1 = time.ticks_us()
dt = time.ticks_diff(t1, t0)
fmt = "{:5.3f} s, {:6.3f} uSec/blink : {:8.2f} kHz/s"
print(fmt.format(dt * 1e-6, dt/N, N/dt * 1e3))

# 3.548 s, 35.483 uSec/blink :    28.18 kHz/s