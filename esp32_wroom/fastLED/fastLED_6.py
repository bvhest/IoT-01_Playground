import machine
import time

# (blue) on-board LED:
led = machine.Pin(2, machine.Pin.OUT)

N = 100000

# LED direct aan en uit zetten obv functie
#   mét voorgeladen functie-aanroepen
#   én 'uitgeschreven' loop.
def blink_preload_unrolled8(n):
    n //= 8
    aan = led.on
    uit = led.off
    r = range(n)
    for i in r:
        aan()
        uit()
        aan()
        uit()
        aan()
        uit()
        aan()
        uit()
        aan()
        uit()
        aan()
        uit()
        aan()
        uit()
        aan()
        uit()

def timer(f, n):
    t0 = time.ticks_us()
    f(n)
    t1 = time.ticks_us()
    dt = time.ticks_diff(t1, t0)
    fmt = "{:5.3f} s, {:6.3f} uSec/blink : {:8.2f} kHz/s"
    print(fmt.format(dt * 1e-6, dt/N, N/dt * 1e3))

timer(blink_preload_unrolled8, N)

# 0.378 s,  3.777 uSec/blink :   264.76 kHz/s