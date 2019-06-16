import machine
import time

# (blue) on-board LED:
led = machine.Pin(2, machine.Pin.OUT)

N = 100000

# LED direct aan en uit via directe method aanroep.
def blink_simple(n):
    for i in range(n):
        led.on()
        led.off()

def timer(f, n):
    t0 = time.ticks_us()
    f(n)
    t1 = time.ticks_us()
    dt = time.ticks_diff(t1, t0)
    fmt = "{:5.3f} s, {:6.3f} uSec/blink : {:8.2f} kHz/s"
    print(fmt.format(dt * 1e-6, dt/N, N/dt * 1e3))


timer(blink_simple, N)

# 3.218 s, 32.181 uSec/blink :    31.07 kHz/s